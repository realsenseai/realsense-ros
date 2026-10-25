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
#include <cmath>
#include <cstring>
#include "device_pointcloud_utils.h"

using realsense2_camera::devicepcl::repackXyz;
using realsense2_camera::devicepcl::POINT_STEP;

namespace
{
    // x@0, y@4, z@8 of point `index` in the repacked data
    float field(const std::vector<uint8_t>& data, size_t index, size_t offset)
    {
        float value;
        std::memcpy(&value, data.data() + index * POINT_STEP + offset, sizeof(float));
        return value;
    }
    uint32_t padding(const std::vector<uint8_t>& data, size_t index)
    {
        uint32_t value;
        std::memcpy(&value, data.data() + index * POINT_STEP + 12, sizeof(uint32_t));
        return value;
    }

    // 2x2 cloud: valid, z = 0 (invalid), valid far, NaN z (invalid)
    const float kCloud[] = {
        1.f, 2.f, 3.f,
        4.f, 5.f, 0.f,
        6.f, 7.f, 8.f,
        9.f, 10.f, std::nanf("")
    };
    const size_t kCount = 4;
}

TEST(DeviceXyzRepack, PointStepIsSixteen)
{
    EXPECT_EQ(16u, POINT_STEP);
}

TEST(DeviceXyzRepack, UnorderedDropsInvalidPoints)
{
    std::vector<uint8_t> data;
    const size_t n = repackXyz(kCloud, kCount, false, -1.f, data);
    ASSERT_EQ(2u, n);
    ASSERT_EQ(2u * POINT_STEP, data.size());
    EXPECT_EQ(1.f, field(data, 0, 0));
    EXPECT_EQ(2.f, field(data, 0, 4));
    EXPECT_EQ(3.f, field(data, 0, 8));
    EXPECT_EQ(0u, padding(data, 0));
    EXPECT_EQ(6.f, field(data, 1, 0));
    EXPECT_EQ(7.f, field(data, 1, 4));
    EXPECT_EQ(8.f, field(data, 1, 8));
    EXPECT_EQ(0u, padding(data, 1));
}

TEST(DeviceXyzRepack, OrderedKeepsEverySlotInvalidAsZero)
{
    std::vector<uint8_t> data;
    const size_t n = repackXyz(kCloud, kCount, true, -1.f, data);
    ASSERT_EQ(kCount, n);
    ASSERT_EQ(kCount * POINT_STEP, data.size());
    EXPECT_EQ(3.f, field(data, 0, 8));
    for (size_t offset : {0, 4, 8})
    {
        EXPECT_EQ(0.f, field(data, 1, offset));   // z == 0
        EXPECT_EQ(0.f, field(data, 3, offset));   // z == NaN
    }
    EXPECT_EQ(6.f, field(data, 2, 0));
    EXPECT_EQ(8.f, field(data, 2, 8));
}

TEST(DeviceXyzRepack, UnorderedClipRemovesFarPoints)
{
    std::vector<uint8_t> data;
    const size_t n = repackXyz(kCloud, kCount, false, 5.f, data);   // z = 8 is beyond 5 m
    ASSERT_EQ(1u, n);
    ASSERT_EQ(POINT_STEP, data.size());
    EXPECT_EQ(3.f, field(data, 0, 8));
}

TEST(DeviceXyzRepack, OrderedClipZeroesFarPointsKeepingTheSlot)
{
    std::vector<uint8_t> data;
    const size_t n = repackXyz(kCloud, kCount, true, 5.f, data);
    ASSERT_EQ(kCount, n);
    EXPECT_EQ(3.f, field(data, 0, 8));
    for (size_t offset : {0, 4, 8})
        EXPECT_EQ(0.f, field(data, 2, offset));
}

TEST(DeviceXyzRepack, ClipIsInclusiveAndNonPositiveDisablesIt)
{
    std::vector<uint8_t> data;
    EXPECT_EQ(2u, repackXyz(kCloud, kCount, false, 8.f, data));    // z == clip_distance is kept
    EXPECT_EQ(2u, repackXyz(kCloud, kCount, false, 0.f, data));    // 0: disabled
    EXPECT_EQ(2u, repackXyz(kCloud, kCount, false, -2.f, data));   // negative: disabled (launch default is -2)
}

TEST(DeviceXyzRepack, ReusedBufferIsFullyRewritten)
{
    std::vector<uint8_t> data(1000, 0xAB);
    EXPECT_EQ(2u, repackXyz(kCloud, kCount, false, -1.f, data));
    EXPECT_EQ(2u * POINT_STEP, data.size());
    EXPECT_EQ(0u, padding(data, 0));
    EXPECT_EQ(kCount, repackXyz(kCloud, kCount, true, -1.f, data));
    EXPECT_EQ(0.f, field(data, 1, 0));
}

TEST(DeviceXyzRepack, EmptyCloud)
{
    std::vector<uint8_t> data(8, 1);
    EXPECT_EQ(0u, repackXyz(nullptr, 0, false, -1.f, data));
    EXPECT_TRUE(data.empty());
    EXPECT_EQ(0u, repackXyz(nullptr, 0, true, -1.f, data));
    EXPECT_TRUE(data.empty());
}
