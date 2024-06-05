// Copyright 2024 Intel Corporation. All Rights Reserved.
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

#include "../include/base_realsense_node.h"

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::setSafetySensorIfAvailable()
{
    // Find if the Safety Sensor is available.
    auto iter = std::find_if(_dev_sensors.begin(), _dev_sensors.end(),
                            [](rs2::sensor sensor){return sensor.is<rs2::safety_sensor>();});
    if (iter != _dev_sensors.end())
    {
        _safety_sensor = &(*iter);
    }
}

void BaseRealSenseNode::publishSafetyServices()
{
    _safety_preset_read_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyPresetRead>(
            "~/safety_preset_read",
            [&](const realsense2_camera_msgs::srv::SafetyPresetRead::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::SafetyPresetRead::Response::SharedPtr res)
                        {SafetyPresetReadService(req, res);});

    _safety_preset_write_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyPresetWrite>(
            "~/safety_preset_write",
            [&](const realsense2_camera_msgs::srv::SafetyPresetWrite::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::SafetyPresetWrite::Response::SharedPtr res)
                        {SafetyPresetWriteService(req, res);});

    _safety_interface_config_read_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyInterfaceConfigRead>(
            "~/safety_interface_config_read",
            [&](const realsense2_camera_msgs::srv::SafetyInterfaceConfigRead::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::SafetyInterfaceConfigRead::Response::SharedPtr res)
                        {SafetyInterfaceConfigReadService(req, res);});

    _safety_interface_config_write_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite>(
            "~/safety_interface_config_write",
            [&](const realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite::Response::SharedPtr res)
                        {SafetyInterfaceConfigWriteService(req, res);});

}

void BaseRealSenseNode::SafetyPresetReadService(const realsense2_camera_msgs::srv::SafetyPresetRead::Request::SharedPtr req,
    realsense2_camera_msgs::srv::SafetyPresetRead::Response::SharedPtr res){
    try
    {
        rs2_safety_preset sp = _safety_sensor->as<rs2::safety_sensor>().get_safety_preset(req->index);
        res->preset= std::string(_safety_sensor->as<rs2::safety_sensor>().safety_preset_to_json_string(sp));
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::SafetyPresetWriteService(const realsense2_camera_msgs::srv::SafetyPresetWrite::Request::SharedPtr req,
    realsense2_camera_msgs::srv::SafetyPresetWrite::Response::SharedPtr res){
    try
    {
        rs2_safety_preset sp = _safety_sensor->as<rs2::safety_sensor>().json_string_to_safety_preset(req->preset);
        _safety_sensor->as<rs2::safety_sensor>().set_safety_preset(req->index, sp);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::SafetyInterfaceConfigReadService(const realsense2_camera_msgs::srv::SafetyInterfaceConfigRead::Request::SharedPtr req,
    realsense2_camera_msgs::srv::SafetyInterfaceConfigRead::Response::SharedPtr res){
    try
    {
        rs2_calib_location location = static_cast<rs2_calib_location>(req->calib_location);
        rs2_safety_interface_config sic = _safety_sensor->as<rs2::safety_sensor>().get_safety_interface_config(location);
        res->safety_interface_config= std::string(_safety_sensor->as<rs2::safety_sensor>().safety_interface_config_to_json_string(sic));
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::SafetyInterfaceConfigWriteService(const realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite::Request::SharedPtr req,
    realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite::Response::SharedPtr res){
    try
    {
        rs2_safety_interface_config sic = _safety_sensor->as<rs2::safety_sensor>().json_string_to_safety_interface_config(req->safety_interface_config);
        _safety_sensor->as<rs2::safety_sensor>().set_safety_interface_config(sic);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}
