// Copyright 2023 Intel Corporation. All Rights Reserved.
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


#include <iostream>
#include <vector>
#include "ros_node_base.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>


namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA0{RS2_STREAM_INFRARED, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
    const stream_index_pair MOTION{RS2_STREAM_MOTION, 0};

    bool isValidCharInName(char c);

    std::string rs2_to_ros(std::string rs2_name);
    std::string ros_stream_to_string(rs2_stream stream);
    std::string create_graph_resource_name(const std::string &original_name);
    const rmw_qos_profile_t qos_string_to_qos(std::string str);
    const std::string list_available_qos_strings();
    rs2_format string_to_rs2_format(std::string str);
    std::string vectorToJsonString(const std::vector<uint8_t>& vec);
}

