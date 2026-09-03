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
