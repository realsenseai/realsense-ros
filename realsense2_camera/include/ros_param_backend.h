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

#include "ros_node_base.h"

namespace realsense2_camera
{
    class ParametersBackend
    {
        public:
            ParametersBackend(RosNodeBase& node) : 
                _node(node),
                _logger(node.get_logger())
                {}
            ~ParametersBackend();


            #if defined( RCLCPP_HAS_OnSetParametersCallbackType )
                using ros2_param_callback_type = rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
            #else
                using ros2_param_callback_type = rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;
            #endif

            void add_on_set_parameters_callback(ros2_param_callback_type callback);


        private:
            RosNodeBase& _node;
            rclcpp::Logger _logger;
            std::shared_ptr<void> _ros_callback;
    };
}
