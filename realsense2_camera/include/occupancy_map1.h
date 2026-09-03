// Copyright 2026 RealSense, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Self-contained, header-only parser for the MAP1 self-describing occupancy-grid
// wire format (librealsense#15557 / device-mgr interface/MappingEp12.h). Ported
// from the reviewed implementation in librealsense
// (src/proc/occupancy-map1.h/.cpp, branch niv/occupancy-map1-consume) with the
// same semantics; this copy has no dependency on librealsense internals
// (rsutils, synthetic-stream, metadata-parser) -- the ROS node discriminates the
// MAP1 payload purely by its bytes, not by SDK-side metadata/enums.

#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace realsense2_camera {
namespace map1 {

#pragma pack(push, 1)
struct frame_header {
    uint32_t magic;              // 0x3150414D "MAP1"
    uint16_t version;
    uint8_t  data_type;          // 2 = occupancy grid
    uint8_t  flags;              // bit 0: crc32 field is valid
    uint32_t payload_size;
    uint16_t profile_id;
    uint16_t stream_generation;
    uint32_t crc32;              // CRC-32/ISO-HDLC over payload_size bytes
};
struct occg_header {
    uint16_t width;              // cells along +X (forward)
    uint16_t height;             // cells along +Y (left); camera at height/2
    uint16_t resolution_mm;
    uint16_t cell_stride;
    int32_t  origin_x_mm;
    int32_t  origin_y_mm;
    uint32_t source_frame_id;
    uint64_t timestamp_us;
    uint32_t cell_count;
};
#pragma pack(pop)
static_assert(sizeof(frame_header) == 20, "MAP1 frame header ABI");
static_assert(sizeof(occg_header) == 32, "MAP1 OCCG header ABI");

constexpr uint32_t MAGIC = 0x3150414DU;
constexpr uint8_t DATA_TYPE_OCCG = 2U;
constexpr uint8_t FLAG_CRC32 = 1U;

enum class parse_result { ok, not_map1, not_occg, truncated, bad_crc, bad_geometry, bad_version };

struct occg_view {
    occg_header header;
    const int8_t* cells;
};

inline bool is_map1(const uint8_t* data, size_t size)
{
    if (data == nullptr || size < sizeof(frame_header)) return false;
    uint32_t magic;
    std::memcpy(&magic, data, sizeof(magic));
    return magic == MAGIC;
}

// CRC-32/ISO-HDLC (poly 0xEDB88320, reflected, init/xorout 0xFFFFFFFF) -- same
// algorithm/result as the standard zlib CRC-32. Table-free bit-at-a-time loop
// (no dependency on librealsense's rsutils table implementation); mathematically
// identical output for the same input.
inline uint32_t crc32_iso_hdlc(const void* data, size_t size)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= p[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            const uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

inline parse_result parse_occg(const uint8_t* data, size_t size, occg_view* out)
{
    if (!is_map1(data, size)) return parse_result::not_map1;
    frame_header fh;
    std::memcpy(&fh, data, sizeof(fh));
    if (fh.data_type != DATA_TYPE_OCCG) return parse_result::not_occg;
    // Accept any minor revision of major version 1 (0x01xx); reject anything else
    // (e.g. a future incompatible 0x02xx) rather than silently misinterpreting the
    // rest of the frame under this struct layout.
    if ((fh.version >> 8) != 0x01) return parse_result::bad_version;
    // Widen to uint64_t before adding: fh.payload_size / oh.cell_count are
    // device-controlled uint32_t values, and on a 32-bit size_t build
    // `sizeof(...) + <uint32_t>` could wrap around and bypass these checks.
    if ((uint64_t)size < (uint64_t)sizeof(fh) + fh.payload_size ||
        fh.payload_size < sizeof(occg_header))
        return parse_result::truncated;
    const uint8_t* payload = data + sizeof(fh);
    // Production occupancy (data_type OCCG) frames are expected to set FLAG_CRC32.
    // When it is clear the CRC is skipped and the frame is trusted on the geometry/size
    // checks below alone: a payload bit-flip then goes undetected, but still cannot
    // overflow a consumer buffer (the cell_count cross-check guarantees that).
    if ((fh.flags & FLAG_CRC32) &&
        crc32_iso_hdlc(payload, fh.payload_size) != fh.crc32)
        return parse_result::bad_crc;
    occg_header oh;
    std::memcpy(&oh, payload, sizeof(oh));
    if ((uint64_t)fh.payload_size < (uint64_t)sizeof(oh) + oh.cell_count)
        return parse_result::truncated;
    // A zero-sized grid or an unsupported cell layout (cell_stride is currently
    // always 1 per the wire spec; any other value means a layout this parser
    // doesn't know how to interpret) are both malformed geometry.
    if (oh.width == 0 || oh.height == 0 || oh.cell_stride != 1)
        return parse_result::bad_geometry;
    // width/height/cell_count are independently device-controlled fields; without
    // this check a frame with a small width/height (small allocated destination
    // buffer) and a large cell_count (large memcpy length) passes every check
    // above -- especially with FLAG_CRC32 clear, where the CRC never runs -- and
    // overflows whatever fixed-size buffer a consumer sized from width*height.
    // width,height <= 0xFFFF so the product fits in uint32_t without overflow.
    if (oh.cell_count != (uint32_t)oh.width * (uint32_t)oh.height)
        return parse_result::bad_geometry;
    out->header = oh;
    out->cells = reinterpret_cast<const int8_t*>(payload + sizeof(oh));
    return parse_result::ok;
}

}  // namespace map1
}  // namespace realsense2_camera
