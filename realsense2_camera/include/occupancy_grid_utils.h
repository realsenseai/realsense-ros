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

#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>

namespace realsense2_camera {
namespace occupancy {

// Result of validating the device-reported pure-payload occupancy geometry.
enum class GeometryStatus
{
    Ok,
    NonPositiveDims,   // rows or cols <= 0
    SizeOverflow,      // rows * cols would wrap size_t
    PayloadTooSmall,   // fewer payload bytes than rows * cols cells
    NonPositiveCellSize,  // cell size <= 0 -> resolution 0, a malformed grid
};

// Validated geometry + unit-converted map fields for a pure-payload occupancy frame.
struct Geometry
{
    uint32_t width = 0;         // = cols  (cells along +X / forward)
    uint32_t height = 0;        // = rows  (cells along +Y / left)
    float    resolution_m = 0;  // cell size, cm -> m
    double   origin_x_m = 0;    // mm -> m
    double   origin_y_m = 0;    // mm -> m
    size_t   n = 0;             // rows * cols (cell count)
};

// Validate the raw occupancy metadata integers against the payload size and convert
// units. cell_size_cm / origin_*_mm are the raw metadata values; raw_size is the
// payload byte count. `out` is written only when the result is Ok; on any failure it
// is left untouched and the caller drops the frame. Kept header-only + free of ROS/SDK
// types so it is unit-testable in isolation, like splitCells below.
inline GeometryStatus computeGeometry(int cols, int rows, int cell_size_cm,
                                      int origin_x_mm, int origin_y_mm,
                                      size_t raw_size, Geometry& out)
{
    if (rows <= 0 || cols <= 0)
        return GeometryStatus::NonPositiveDims;
    // rows, cols are positive but device-controlled: reject a product that would wrap
    // size_t before it sizes the grid.
    if (static_cast<size_t>(rows) > SIZE_MAX / static_cast<size_t>(cols))
        return GeometryStatus::SizeOverflow;
    const size_t n = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    if (raw_size < n)
        return GeometryStatus::PayloadTooSmall;
    // A zero/negative cell size would publish resolution 0, which nav_msgs consumers
    // reject or misread - drop the frame rather than forward a malformed grid.
    if (cell_size_cm <= 0)
        return GeometryStatus::NonPositiveCellSize;

    out.width = static_cast<uint32_t>(cols);
    out.height = static_cast<uint32_t>(rows);
    out.resolution_m = static_cast<float>(cell_size_cm) / 100.0f;   // cm -> m
    out.origin_x_m = static_cast<double>(origin_x_mm) / 1000.0;     // mm -> m
    out.origin_y_m = static_cast<double>(origin_y_mm) / 1000.0;     // mm -> m
    out.n = n;
    return GeometryStatus::Ok;
}

// CRC-32/ISO-HDLC (poly 0xEDB88320, reflected, init/xorout 0xFFFFFFFF) - same result
// as zlib crc32. This is what the FW reports in RS2_FRAME_METADATA_CRC for the
// pure-payload occupancy frame, computed over the whole payload.
inline uint32_t crc32IsoHdlc(const void* data, size_t size)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= p[i];
        for (int k = 0; k < 8; ++k)
        {
            const uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// MAP1 cell ladder -> the two published data arrays (see file docs in the PR).
// Values in [1, occupied_threshold) publish as unknown on the binary grid:
// evidence exists, so "free" would be false, but the bar for "occupied" is not
// met -- unknown is the only honest value.
inline void splitCells(const int8_t* cells, size_t n, int8_t occupied_threshold,
                       std::vector<int8_t>& occupancy_out,
                       std::vector<int8_t>& certainty_out)
{
    occupancy_out.resize(n);
    certainty_out.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
        const int8_t v = cells[i];
        certainty_out[i] = v;
        if (v <= 0)
            occupancy_out[i] = v;                       // -1 unknown, 0 free
        else
            occupancy_out[i] = (v >= occupied_threshold) ? int8_t{100} : int8_t{-1};
    }
}

}  // namespace occupancy
}  // namespace realsense2_camera
