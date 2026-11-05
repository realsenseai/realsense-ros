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
#include <cstring>
#include <vector>

namespace realsense2_camera {
namespace devicepcl {

// Layout of the XYZ PointCloud2 published for the device point cloud (pointcloud.source=device).
// Same as the host "xyz" cloud: x@0, y@4, z@8 FLOAT32, 4 bytes of padding up to point_step 16.
constexpr uint32_t POINT_STEP = 16;
constexpr size_t   VERTEX_SIZE = 3 * sizeof(float);   // the SDK device cloud is a bare float3 array

// Repack `point_count` float3 vertices (12 B/pt, as delivered by the SDK for RS2_STREAM_POINT_CLOUD)
// into `out` as 16 B/pt PointCloud2 data.
//  * A point is valid when z > 0 and, if clip_distance > 0, z <= clip_distance.
//  * ordered == false: invalid points are dropped; returns the number of valid points and `out`
//    holds exactly that many points.
//  * ordered == true: every slot is kept (point_count points, returned) and an invalid point is
//    written as 0,0,0, so the W x H geometry of the cloud holds.
// A non-positive clip_distance means clipping is disabled (the node default is -1, like for Depth).
// Kept header-only + free of ROS/SDK types so it is unit-testable in isolation.
inline size_t repackXyz(const float* xyz, size_t point_count, bool ordered, float clip_distance,
                        std::vector<uint8_t>& out)
{
    // Zero-filled: the padding bytes and the 0,0,0 slots of an ordered cloud.
    out.assign(point_count * POINT_STEP, 0);
    uint8_t* dst = out.data();
    size_t valid_count = 0;
    for (size_t i = 0; i < point_count; ++i, xyz += 3)
    {
        const float z = xyz[2];
        const bool valid = (z > 0.f) && (clip_distance <= 0.f || z <= clip_distance);   // NaN z is invalid
        if (!valid)
            continue;
        const size_t slot = ordered ? i : valid_count;
        std::memcpy(dst + slot * POINT_STEP, xyz, VERTEX_SIZE);
        ++valid_count;
    }
    if (ordered)
        return point_count;
    out.resize(valid_count * POINT_STEP);
    return valid_count;
}

} // namespace devicepcl
} // namespace realsense2_camera
