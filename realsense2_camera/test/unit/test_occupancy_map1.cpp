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

#include <gtest/gtest.h>
#include "occupancy_map1.h"
#include <cstring>
#include <vector>

using namespace realsense2_camera::map1;

namespace {
std::vector<uint8_t> make_frame(uint16_t w, uint16_t h, bool valid_crc = true,
                                uint8_t data_type = DATA_TYPE_OCCG,
                                uint16_t version = 0x0100, uint16_t cell_stride = 1)
{
    occg_header oh{};
    oh.width = w; oh.height = h; oh.resolution_mm = 50; oh.cell_stride = cell_stride;
    oh.origin_x_mm = 0; oh.origin_y_mm = -(int32_t)(h * 50 / 2);
    oh.source_frame_id = 7; oh.timestamp_us = 123456789ULL;
    oh.cell_count = (uint32_t)w * h;
    std::vector<uint8_t> payload(sizeof(oh) + oh.cell_count);
    std::memcpy(payload.data(), &oh, sizeof(oh));
    auto* cells = (int8_t*)(payload.data() + sizeof(oh));
    std::memset(cells, -1, oh.cell_count);
    if (oh.cell_count >= 3) { cells[0] = 0; cells[1] = 100; cells[2] = 37; }

    frame_header fh{};
    fh.magic = MAGIC; fh.version = version; fh.data_type = data_type;
    fh.flags = FLAG_CRC32; fh.payload_size = (uint32_t)payload.size();
    fh.profile_id = 0x0201; fh.stream_generation = 1;
    fh.crc32 = crc32_iso_hdlc(payload.data(), payload.size());
    if (!valid_crc) fh.crc32 ^= 0xDEADBEEFU;

    std::vector<uint8_t> frame(sizeof(fh) + payload.size());
    std::memcpy(frame.data(), &fh, sizeof(fh));
    std::memcpy(frame.data() + sizeof(fh), payload.data(), payload.size());
    return frame;
}
}  // namespace

TEST(OccupancyMap1, GoodOccgFrameParsesToHeaderAndCells)
{
    auto f = make_frame(320, 256);
    occg_view v{};
    ASSERT_EQ(parse_occg(f.data(), f.size(), &v), parse_result::ok);
    EXPECT_EQ(v.header.width, 320);
    EXPECT_EQ(v.header.height, 256);
    EXPECT_EQ(v.header.resolution_mm, 50);
    EXPECT_EQ(v.header.origin_y_mm, -6400);
    EXPECT_EQ(v.header.cell_count, 81920u);
    EXPECT_EQ(v.cells[0], 0);
    EXPECT_EQ(v.cells[1], 100);
    EXPECT_EQ(v.cells[2], 37);
    EXPECT_EQ(v.cells[3], -1);
}

TEST(OccupancyMap1, NonMap1BytesAreNotMap1)
{
    std::vector<uint8_t> legacy(1063, 0xAB);   // D585S bit-packed grid, no magic
    occg_view v{};
    EXPECT_FALSE(is_map1(legacy.data(), legacy.size()));
    EXPECT_EQ(parse_occg(legacy.data(), legacy.size(), &v), parse_result::not_map1);
}

TEST(OccupancyMap1, BadCrcIsRejected)
{
    auto f = make_frame(320, 256, /*valid_crc=*/false);
    occg_view v{};
    EXPECT_EQ(parse_occg(f.data(), f.size(), &v), parse_result::bad_crc);
}

TEST(OccupancyMap1, TruncatedPayloadIsRejected)
{
    auto f = make_frame(320, 256);
    occg_view v{};
    EXPECT_EQ(parse_occg(f.data(), f.size() - 100, &v), parse_result::truncated);
}

TEST(OccupancyMap1, CellCountInconsistentWithWidthHeightIsRejected)
{
    // width/height (which size the destination buffer) and cell_count (which sizes
    // the memcpy out of the parsed view) are independent device-controlled fields.
    // Craft a frame that is otherwise well-formed -- payload big enough to satisfy
    // the truncated check, FLAG_CRC32 clear so the mismatch isn't masked by/coupled
    // to a CRC failure -- purely to isolate the geometry cross-check.
    const uint16_t w = 4, h = 4;               // declares a 16-cell (4x4) grid...
    occg_header oh{};
    oh.width = w; oh.height = h; oh.resolution_mm = 50; oh.cell_stride = 1;
    oh.origin_x_mm = 0; oh.origin_y_mm = 0;
    oh.source_frame_id = 1; oh.timestamp_us = 1;
    oh.cell_count = 1000;                      // ...but claims 1000 cells of payload.
    std::vector<uint8_t> payload(sizeof(oh) + oh.cell_count, 0);
    std::memcpy(payload.data(), &oh, sizeof(oh));

    frame_header fh{};
    fh.magic = MAGIC; fh.version = 0x0100; fh.data_type = DATA_TYPE_OCCG;
    fh.flags = 0;  // FLAG_CRC32 clear: the CRC path must not be what catches this.
    fh.payload_size = (uint32_t)payload.size();
    fh.profile_id = 0x0201; fh.stream_generation = 1;
    fh.crc32 = 0;

    std::vector<uint8_t> frame(sizeof(fh) + payload.size());
    std::memcpy(frame.data(), &fh, sizeof(fh));
    std::memcpy(frame.data() + sizeof(fh), payload.data(), payload.size());

    occg_view v{};
    EXPECT_EQ(parse_occg(frame.data(), frame.size(), &v), parse_result::bad_geometry);
}

TEST(OccupancyMap1, CrcReferenceVector)
{
    // CRC-32/ISO-HDLC ("123456789") == 0xCBF43926 -- pins the polynomial/reflection.
    EXPECT_EQ(crc32_iso_hdlc("123456789", 9), 0xCBF43926u);
}

TEST(OccupancyMap1, GoldenFrameDecodesCellsInRowMajorOrder)
{
    // Golden frame: a small 4(forward/width) x 3(lateral/height) grid with a distinct
    // value at each corner + one interior cell, so the byte<->position contract and the
    // axis convention (X forward along width, Y lateral, data[cy*width+cx] row-major) are
    // pinned. A future struct-packing/offset/axis change breaks this loudly, catching the
    // "valid CRC but mirrored/rotated grid" failure mode CRC/bounds checks cannot.
    const uint16_t W = 4, H = 3;
    occg_header oh{};
    oh.width = W; oh.height = H; oh.resolution_mm = 50; oh.cell_stride = 1;
    oh.origin_x_mm = 0; oh.origin_y_mm = -(int32_t)(H * 50 / 2);   // lateral-centered = -75
    oh.source_frame_id = 7; oh.timestamp_us = 42ULL;
    oh.cell_count = (uint32_t)W * H;                               // 12

    int8_t grid[W * H];
    std::memset(grid, -1, sizeof(grid));                          // default unknown
    grid[0 * W + 0] = 0;      // (cx=0, cy=0) free
    grid[0 * W + 3] = 100;    // (cx=3, cy=0) occupied
    grid[2 * W + 0] = 42;     // (cx=0, cy=2) graded
    grid[2 * W + 3] = 7;      // (cx=3, cy=2) graded
    grid[1 * W + 2] = 0;      // (cx=2, cy=1) interior free

    std::vector<uint8_t> payload(sizeof(oh) + oh.cell_count);
    std::memcpy(payload.data(), &oh, sizeof(oh));
    std::memcpy(payload.data() + sizeof(oh), grid, sizeof(grid));

    frame_header fh{};
    fh.magic = MAGIC; fh.version = 0x0100; fh.data_type = DATA_TYPE_OCCG;
    fh.flags = FLAG_CRC32; fh.payload_size = (uint32_t)payload.size();
    fh.profile_id = 0x0201; fh.stream_generation = 1;
    fh.crc32 = crc32_iso_hdlc(payload.data(), payload.size());

    std::vector<uint8_t> frame(sizeof(fh) + payload.size());
    std::memcpy(frame.data(), &fh, sizeof(fh));
    std::memcpy(frame.data() + sizeof(fh), payload.data(), payload.size());

    occg_view v{};
    ASSERT_EQ(parse_occg(frame.data(), frame.size(), &v), parse_result::ok);
    // geometry + the frozen wire convention
    EXPECT_EQ(v.header.width, W);
    EXPECT_EQ(v.header.height, H);
    EXPECT_EQ(v.header.origin_x_mm, 0);
    EXPECT_EQ(v.header.origin_y_mm, -75);
    EXPECT_EQ(v.header.cell_count, 12u);
    // cells preserved byte-for-byte in row-major data[cy*width+cx] order
    EXPECT_EQ(v.cells[0 * W + 0], 0);
    EXPECT_EQ(v.cells[0 * W + 3], 100);
    EXPECT_EQ(v.cells[2 * W + 0], 42);
    EXPECT_EQ(v.cells[2 * W + 3], 7);
    EXPECT_EQ(v.cells[1 * W + 2], 0);
    EXPECT_EQ(v.cells[0 * W + 1], -1);   // an untouched cell stays unknown
}
