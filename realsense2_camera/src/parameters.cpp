// Copyright 2023 RealSense, Inc. All Rights Reserved.
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
#include <ros_utils.h>
#include <iomanip>

using namespace realsense2_camera;

void BaseRealSenseNode::getParameters()
{
    ROS_INFO("getParameters...");

    std::string param_name;

    param_name = std::string("camera_name");
    _camera_name = _parameters->setParam<std::string>(param_name, "camera");
    _parameters_names.push_back(param_name);

    param_name = std::string("publish_tf");
    _publish_tf = _parameters->setParam<bool>(param_name, PUBLISH_TF);
    _parameters_names.push_back(param_name);

    param_name = std::string("tf_publish_rate");
    _parameters->setParamT(param_name, _tf_publish_rate, [this](const rclcpp::Parameter& )
            {
                startDynamicTf();
            });
    _parameters_names.push_back(param_name);

    param_name = std::string("diagnostics_period");
    _diagnostics_period = _parameters->setParam<double>(param_name, DIAGNOSTICS_PERIOD);
    _parameters_names.push_back(param_name);

    param_name = std::string("enable_sync");
    _parameters->setParamT(param_name, _sync_frames);
    _parameters_names.push_back(param_name);

    param_name = std::string("enable_rgbd");
    _parameters->setParamT(param_name, _enable_rgbd, [this](const rclcpp::Parameter& )
    {
        {
            std::lock_guard<std::mutex> lock_guard(_profile_changes_mutex);
            _is_profile_changed = true;
        }
        _cv_mpc.notify_one();
    });
    _parameters_names.push_back(param_name);

    param_name = std::string("json_file_path");
    _json_file_path = _parameters->setParam<std::string>(param_name, "");
    _parameters_names.push_back(param_name);

    param_name = std::string("clip_distance");
    _clipping_distance = _parameters->setParam<double>(param_name, -1.0);
    _parameters_names.push_back(param_name);

    param_name = std::string("linear_accel_cov");
    _linear_accel_cov = _parameters->setParam<double>(param_name, 0.01);
    _parameters_names.push_back(param_name);

    param_name = std::string("angular_velocity_cov");
    _angular_velocity_cov = _parameters->setParam<double>(param_name, 0.01);
    _parameters_names.push_back(param_name);
   
    param_name = std::string("hold_back_imu_for_frames");
    _hold_back_imu_for_frames = _parameters->setParam<bool>(param_name, HOLD_BACK_IMU_FOR_FRAMES);
    _parameters_names.push_back(param_name);

    param_name = std::string("base_frame_id");
    _base_frame_id = _parameters->setParam<std::string>(param_name, DEFAULT_BASE_FRAME_ID);
    _base_frame_id = (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << _base_frame_id)).str();
    _parameters_names.push_back(param_name);

    param_name = std::string("tf_prefix");
    _tf_prefix = _parameters->setParam<std::string>(param_name, "");
    _parameters_names.push_back(param_name);

#if defined (ACCELERATE_GPU_WITH_GLSL)
    param_name = std::string("accelerate_gpu_with_glsl");
     _parameters->setParam<bool>(param_name, false, 
                    [this](const rclcpp::Parameter& parameter)
                    {
                        bool temp_value = parameter.get_value<bool>();
                        if (_accelerate_gpu_with_glsl != temp_value)
                        {
                            _accelerate_gpu_with_glsl = temp_value;
                            std::lock_guard<std::mutex> lock_guard(_profile_changes_mutex);
                            _is_accelerate_gpu_with_glsl_changed = true;
                        }
                        _cv_mpc.notify_one();
                    });
    _parameters_names.push_back(param_name);
#endif

}

void BaseRealSenseNode::setDynamicParams()
{
    // Set default values:
    _imu_sync_method = imu_sync_method::NONE;

    auto imu_sync_method_string = [](imu_sync_method value) 
    { 
        switch (value)
        {
        case imu_sync_method::COPY:
            return "COPY";
        case imu_sync_method::LINEAR_INTERPOLATION:
            return "LINEAR_INTERPOLATION";
        default:
            return "NONE";
        }
    };

    // Register ROS parameter:
    std::string param_name("unite_imu_method");

    std::vector<std::pair<std::string, int> > enum_vec;
    size_t longest_desc(0);
    for (int i=0; i<=int(imu_sync_method::LINEAR_INTERPOLATION); i++)
    {
        std::string enum_str(imu_sync_method_string(imu_sync_method(i)));
        enum_vec.push_back(std::make_pair(enum_str, i));
        longest_desc = std::max(longest_desc, enum_str.size());
    }
    sort(enum_vec.begin(), enum_vec.end(), [](std::pair<std::string, int> e1, std::pair<std::string, int> e2){return (e1.second < e2.second);});
    std::stringstream enum_str_values;
    for (auto vec_iter : enum_vec)
    {
        enum_str_values << std::setw(longest_desc+6) << std::left << vec_iter.first << " : " << vec_iter.second << std::endl;
    }

    rcl_interfaces::msg::ParameterDescriptor crnt_descriptor;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = int(imu_sync_method::NONE);
    range.to_value = int(imu_sync_method::LINEAR_INTERPOLATION);
    crnt_descriptor.integer_range.push_back(range);
    std::stringstream desc;
    desc << "Available options are:" << std::endl << enum_str_values.str();
    crnt_descriptor.description = desc.str();
    _parameters->setParam<int>(param_name, int(imu_sync_method::NONE), 
                            [this](const rclcpp::Parameter& parameter)
                            {
                                _imu_sync_method = imu_sync_method(parameter.get_value<int>());
                                ROS_WARN("For the 'unite_imu_method' param update to take effect, "
                                         "re-enable either gyro or accel stream.");
                            }, crnt_descriptor);
    _parameters_names.push_back(param_name);
}

void BaseRealSenseNode::clearParameters()
{
    while ( !_parameters_names.empty() )
    {
        auto name = _parameters_names.back();
        _parameters->removeParam(name);
        _parameters_names.pop_back();        
    }
}

