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
#include <librealsense2/rs.hpp>
#include "ros_node_base.h"
#include <dynamic_params.h>

namespace realsense2_camera
{
    class SensorParams
    {
        public:
            SensorParams(std::shared_ptr<Parameters> parameters, rclcpp::Logger logger):
                _logger(logger),
                _parameters(parameters) {};
            ~SensorParams();
            void registerDynamicOptions(rs2::options sensor, const std::string& module_name);
            void clearParameters();
            std::shared_ptr<Parameters> getParameters() {return _parameters;};

        public:
            rclcpp::Logger _logger;

        private:
            double float_to_double(float val, rs2::option_range option_range);
            template<class T>
            rcl_interfaces::msg::ParameterDescriptor get_parameter_descriptor(const std::string& option_name, rs2::option_range option_range,
                T option_value, const std::string& option_description, const std::string& description_addition);
            template<class T>
            void set_parameter(rs2::options sensor, rs2_option option, const std::string& module_name, const std::string& description_addition="");

        private:
            std::shared_ptr<Parameters> _parameters;
            std::vector<std::string> _parameters_names;
    };
}
