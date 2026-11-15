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
#include "occupancy_grid_utils.h"

using realsense2_camera::occupancy::splitCells;
using realsense2_camera::occupancy::computeGeometry;
using realsense2_camera::occupancy::Geometry;
using realsense2_camera::occupancy::GeometryStatus;
using realsense2_camera::occupancy::crc32IsoHdlc;

TEST(Crc32IsoHdlc, StandardCheckValue)
{
    // The standard CRC-32 check value for "123456789" (same as zlib crc32).
    const char msg[] = "123456789";
    EXPECT_EQ(0xCBF43926u, crc32IsoHdlc(msg, 9));
}

TEST(Crc32IsoHdlc, DetectsSingleByteCorruption)
{
    std::vector<int8_t> payload(320 * 256, -1);
    const uint32_t good = crc32IsoHdlc(payload.data(), payload.size());
    payload[1234] = 100;
    EXPECT_NE(good, crc32IsoHdlc(payload.data(), payload.size()));
}

TEST(SplitCells, LadderWithDefaultThreshold)
{
    const int8_t cells[] = {-1, 0, 1, 37, 99, 100};
    std::vector<int8_t> occ, cert;
    splitCells(cells, 6, 100, occ, cert);
    EXPECT_EQ((std::vector<int8_t>{-1, 0, -1, -1, -1, 100}), occ);
    EXPECT_EQ((std::vector<int8_t>{-1, 0, 1, 37, 99, 100}), cert);
}

TEST(SplitCells, LowerThresholdMovesTheBar)
{
    const int8_t cells[] = {-1, 0, 49, 50, 100};
    std::vector<int8_t> occ, cert;
    splitCells(cells, 5, 50, occ, cert);
    EXPECT_EQ((std::vector<int8_t>{-1, 0, -1, 100, 100}), occ);
}

TEST(SplitCells, ReusesBuffersWithoutGrowth)
{
    const int8_t cells[] = {0, 0, 0};
    std::vector<int8_t> occ(3), cert(3);
    auto* occ_ptr = occ.data();
    splitCells(cells, 3, 100, occ, cert);
    EXPECT_EQ(occ_ptr, occ.data());   // hot path: no realloc when sized right
}

TEST(ComputeGeometry, ValidGridConvertsUnits)
{
    // The live D555 case: 320x256, 5 cm cells, origin (0, -6400 mm), 81920-byte payload.
    Geometry geo;
    EXPECT_EQ(GeometryStatus::Ok,
              computeGeometry(/*cols*/320, /*rows*/256, /*cell_cm*/5,
                              /*ox_mm*/0, /*oy_mm*/-6400, /*raw_size*/81920, geo));
    EXPECT_EQ(320u, geo.width);
    EXPECT_EQ(256u, geo.height);
    EXPECT_EQ(81920u, geo.n);
    EXPECT_FLOAT_EQ(0.05f, geo.resolution_m);   // cm -> m
    EXPECT_DOUBLE_EQ(0.0, geo.origin_x_m);      // mm -> m
    EXPECT_DOUBLE_EQ(-6.4, geo.origin_y_m);     // mm -> m
}

TEST(ComputeGeometry, RejectsNonPositiveDims)
{
    Geometry geo;
    EXPECT_EQ(GeometryStatus::NonPositiveDims, computeGeometry(0, 256, 5, 0, 0, 81920, geo));
    EXPECT_EQ(GeometryStatus::NonPositiveDims, computeGeometry(320, 0, 5, 0, 0, 81920, geo));
    EXPECT_EQ(GeometryStatus::NonPositiveDims, computeGeometry(-1, 256, 5, 0, 0, 81920, geo));
}

TEST(ComputeGeometry, RejectsPayloadSmallerThanGrid)
{
    Geometry geo;
    // One byte short of 320*256 cells must be dropped, not read out of bounds.
    EXPECT_EQ(GeometryStatus::PayloadTooSmall, computeGeometry(320, 256, 5, 0, 0, 81919, geo));
    // Exactly the grid size is accepted (boundary).
    EXPECT_EQ(GeometryStatus::Ok, computeGeometry(320, 256, 5, 0, 0, 81920, geo));
}

TEST(ComputeGeometry, RejectsNonPositiveCellSize)
{
    Geometry geo;
    // cell size <= 0 would publish resolution 0 - a malformed grid; drop instead.
    EXPECT_EQ(GeometryStatus::NonPositiveCellSize, computeGeometry(320, 256, 0, 0, 0, 81920, geo));
    EXPECT_EQ(GeometryStatus::NonPositiveCellSize, computeGeometry(320, 256, -5, 0, 0, 81920, geo));
}

TEST(ComputeGeometry, LeavesOutputUntouchedOnFailure)
{
    Geometry geo;   // default-constructed: all zero
    EXPECT_EQ(GeometryStatus::PayloadTooSmall, computeGeometry(320, 256, 5, 0, 0, 10, geo));
    EXPECT_EQ(0u, geo.width);   // not written on failure - caller drops the frame
    EXPECT_EQ(0u, geo.n);
}
