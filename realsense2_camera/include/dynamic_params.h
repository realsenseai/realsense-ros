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
#include <ros_utils.h>
#include "constants.h"
#include <deque>
#include "ros_param_backend.h"

namespace realsense2_camera
{
    class Parameters
    {
        public:
            Parameters(RosNodeBase& node);
            ~Parameters();
            template <class T>
            T setParam(std::string param_name, const T& initial_value, 
                                    std::function<void(const rclcpp::Parameter&)> func = std::function<void(const rclcpp::Parameter&)>(), 
                                    rcl_interfaces::msg::ParameterDescriptor descriptor=rcl_interfaces::msg::ParameterDescriptor());

            template <class T>
            T readAndDeleteParam(std::string param_name, const T& initial_value);

            template <class T>
            void setParamT(std::string param_name, T& param,
                           std::function<void(const rclcpp::Parameter&)> func = std::function<void(const rclcpp::Parameter&)>(),
                           rcl_interfaces::msg::ParameterDescriptor descriptor=rcl_interfaces::msg::ParameterDescriptor());
            template <class T>
            void setParamValue(T& param, const T& value); // function updates the parameter value both locally and in the parameters server

            void setRosParamValue(const std::string param_name, void const* const value); // function updates the parameters server
            void removeParam(std::string param_name);
            void pushUpdateFunctions(std::vector<std::function<void()> > funcs);

            template <class T>
            void queueSetRosValue(const std::string& param_name, const T value);
       
            template<typename T>
            T getOrDeclareParameter(const std::string param_name, const T& initial_value);

            template <class T>
            T getParam(std::string param_name);

        private:
            void monitor_update_functions();

        private:
            RosNodeBase& _node;
            rclcpp::Logger _logger;
            std::map<std::string, std::function<void(const rclcpp::Parameter&)> > _param_functions;
            std::map<void*, std::string> _param_names;
            ParametersBackend _params_backend;
            std::condition_variable _update_functions_cv;
            bool _is_running;
            std::shared_ptr<std::thread> _update_functions_t;
            std::deque<std::function<void()> > _update_functions_v;
            std::list<std::string> self_set_parameters;
            std::mutex _mu;

    };
}
