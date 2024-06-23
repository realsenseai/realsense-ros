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

#include <ros_utils.h>
#include <algorithm>
#include <map>
#include <cctype>

namespace realsense2_camera
{
template <typename K, typename V>
std::ostream& operator<<(std::ostream& os, const std::map<K, V>& m)
{
    os << '{';
    for (const auto& kv : m)
    {
        os << " {" << kv.first << ": " << kv.second << '}';
    }
    os << " }";
    return os;
}

/**
 * Same as ros::names::isValidCharInName, but re-implemented here because it's not exposed.
 */
bool isValidCharInName(char c)
{
    return std::isalnum(c) || c == '/' || c == '_';
}

/**
 * ROS Graph Resource names don't allow spaces and hyphens (see http://wiki.ros.org/Names),
 * so we replace them here with underscores.
 */

std::string rs2_to_ros(std::string rs2_name)
{
    static std::map<std::string, std::string> libname_to_rosname = {
        {"Infrared", "Infra"},
        {"Stereo Module", "Depth Module"},
        {"L500 Depth Sensor", "Depth Module"} ,
        {"Pointcloud (SSE3)", "Pointcloud"},
    {"Pointcloud (CUDA)", "Pointcloud"},
        {"Align (SSE3)", "Align Depth"},
    {"Align (CUDA)", "Align Depth"},
        {"Depth to Disparity", "disparity filter"},
        {"Depth Visualization", "colorizer"}
    };
    // std::cout << "rs2_name: " << rs2_name << std::endl;
    auto name_iter = libname_to_rosname.find(rs2_name);
    if (name_iter == libname_to_rosname.end())
        return rs2_name;
    else
        return name_iter->second;
}

std::string ros_stream_to_string(rs2_stream stream)
{
    return rs2_to_ros(rs2_stream_to_string(stream));
}

std::string create_graph_resource_name(const std::string &original_name)
{
    std::string fixed_name = original_name;
    std::transform(fixed_name.begin(), fixed_name.end(), fixed_name.begin(),
                    [](unsigned char c) { return std::tolower(c); });
    std::replace_if(fixed_name.begin(), fixed_name.end(), [](const char c) { return !(realsense2_camera::isValidCharInName(c)); },
                    '_');
    return fixed_name;
}

rs2_format string_to_rs2_format(std::string str)
{
    rs2_format format = RS2_FORMAT_ANY;

    for (int i = 0; i < RS2_FORMAT_COUNT; i++)
    {
        transform(str.begin(), str.end(), str.begin(), ::toupper);
        if (str.compare(rs2_format_to_string((rs2_format)i)) == 0)
        {
            format = (rs2_format)i;
            break;
        }
    }
    return format;
}

static const rmw_qos_profile_t rmw_qos_profile_latched =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

const rmw_qos_profile_t qos_string_to_qos(std::string str)
{
    if (str == "UNKNOWN")
        return rmw_qos_profile_unknown;
    if (str == "SYSTEM_DEFAULT")
        return rmw_qos_profile_system_default;
    if (str == "DEFAULT")
        return rmw_qos_profile_default;
    if (str == "PARAMETER_EVENTS")
        return rmw_qos_profile_parameter_events;
    if (str == "SERVICES_DEFAULT")
        return rmw_qos_profile_services_default;
    if (str == "PARAMETERS")
        return rmw_qos_profile_parameters;
    if (str == "SENSOR_DATA")
        return rmw_qos_profile_sensor_data;
    throw std::runtime_error("Unknown QoS string " + str);
}

const std::string list_available_qos_strings()
{
    std::stringstream res;
    res << "UNKNOWN" << "\n"
        << "SYSTEM_DEFAULT" << "\n"
        << "DEFAULT" << "\n"
        << "PARAMETER_EVENTS" << "\n"
        << "SERVICES_DEFAULT" << "\n"
        << "PARAMETERS" << "\n"
        << "SENSOR_DATA";
    return res.str();
}

std::string vectorToJsonString(const std::vector<uint8_t>& vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        oss << static_cast<int>(vec[i]);
        if (i < vec.size() - 1) {
            oss << ",";
        }
    }
    oss << "]";
    return oss.str();
}

}
