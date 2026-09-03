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
