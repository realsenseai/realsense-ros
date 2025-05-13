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

#include <dynamic_params.h>

namespace realsense2_camera
{
    Parameters::Parameters(RosNodeBase& node) :
    _node(node),
    _logger(node.get_logger()),
    _params_backend(node),
    _is_running(true)
    {
        _params_backend.add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) 
                { 
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;
                    for (const auto & parameter : parameters) 
                    {
                        try
                        {
                            const auto& func_iter = _param_functions.find(parameter.get_name());
                            if (func_iter != _param_functions.end())
                            {
                                std::list<std::string>::iterator name_iter(std::find(self_set_parameters.begin(), self_set_parameters.end(), parameter.get_name()));
                                if (name_iter != self_set_parameters.end())
                                {
                                    self_set_parameters.erase(name_iter);
                                }
                                else
                                {
                                    (func_iter->second)(parameter);
                                }
                            }
                        }
                        catch(const std::exception& e)
                        {
                            result.successful = false;
                            result.reason = e.what();
                            ROS_WARN_STREAM("Set parameter {" << parameter.get_name()
                                                            << "} failed: " << e.what());
                        }                            
                    }

                    return result;
                });
        monitor_update_functions(); // Start parameters update thread
    }

    // pushUpdateFunctions:
    // Cannot update ros parameters from within ros parameters callback.
    // This function is used by the parameter callback function to update other ros parameters.
    void Parameters::pushUpdateFunctions(std::vector<std::function<void()> > funcs)
    {
        _update_functions_v.insert(_update_functions_v.end(), funcs.begin(), funcs.end());
        _update_functions_cv.notify_one();
    }

    void Parameters::monitor_update_functions()
    {
        int time_interval(1000);
        std::function<void()> func = [this, time_interval](){
            std::unique_lock<std::mutex> lock(_mu);
            while(_is_running) {
                _update_functions_cv.wait_for(lock, std::chrono::milliseconds(time_interval), [&]{return !_is_running || !_update_functions_v.empty();});
                while (!_update_functions_v.empty())
                {
                    _update_functions_v.front()();
                    _update_functions_v.pop_front();
                }
            }
        };
        _update_functions_t = std::make_shared<std::thread>(func);
    }

    Parameters::~Parameters()
    {
        _is_running = false;
        if (_update_functions_t && _update_functions_t->joinable())
            _update_functions_t->join();
        for (auto const& param : _param_functions)
        {
            _node.undeclare_parameter(param.first);
        }
        // remove_on_set_parameters_callback(_params_backend);
    }

    template <class T>
    T Parameters::readAndDeleteParam(std::string param_name, const T& initial_value)
    {
        // Function is meant for reading parameters needed in initialization but should not be declared by the app.
        T result_value = setParam(param_name, initial_value);
        removeParam(param_name);
        return result_value;
    }

    template <class T>
    T Parameters::setParam(std::string param_name, const T& initial_value, 
                              std::function<void(const rclcpp::Parameter&)> func, 
                              rcl_interfaces::msg::ParameterDescriptor descriptor)
    {
        T result_value(initial_value);
        try
        {
            ROS_DEBUG_STREAM("setParam::Setting parameter: " << param_name);
#if defined(FOXY)
            //do nothing for old versions (foxy)
#else
            descriptor.dynamic_typing=true;
#endif
            if (!_node.get_parameter(param_name, result_value))
            {
                result_value = _node.declare_parameter(param_name, initial_value, descriptor);
            }
        }
        catch(const std::exception& e)
        {
            std::stringstream range;
            for (auto val : descriptor.floating_point_range)
            {
                range << val.from_value << ", " << val.to_value;
            }
            for (auto val : descriptor.integer_range)
            {
                range << val.from_value << ", " << val.to_value;
            }
            ROS_WARN_STREAM("Could not set param: " << param_name << " with " << 
                             initial_value << 
                            " Range: [" << range.str() << "]" <<
                             ": " << e.what());
            return initial_value;
        }
        
        if (_param_functions.find(param_name) != _param_functions.end())
            ROS_DEBUG_STREAM("setParam::Replace function for : " << param_name);
        
        if (func)
            _param_functions[param_name] = func;
        else
            _param_functions[param_name] = [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("Parameter can not be changed in runtime.");
            };
        if (result_value != initial_value && func)
        {
            try
            {
                func(rclcpp::Parameter(param_name, result_value));
            }
            catch(const std::exception& e)
            {
                ROS_WARN_STREAM("Set parameter {" << param_name << "} failed: " << e.what());
            } 
        }
        return result_value;
    }

    // setParamT: Used to automatically update param based on its parallel ros parameter.
    // Notice: param must remain alive as long as the callback is active - 
    //      if param is destroyed the behavior of the callback is undefined.
    template <class T>
    void Parameters::setParamT(std::string param_name, T& param, 
                              std::function<void(const rclcpp::Parameter&)> func,
                              rcl_interfaces::msg::ParameterDescriptor descriptor)

    {
    param = setParam<T>(param_name, param, 
                          [&param, func](const rclcpp::Parameter& parameter)
                                        {
                                            param = parameter.get_value<T>();
                                            if (func) func(parameter);
                                        }, descriptor);
    }

    template <class T>
    void Parameters::setParamValue(T& param, const T& value)
    {
        // setParamValue updates a variable and its parallel in the parameters server.
        // NOTICE: <param> must have the same address it was declared with.
        param = value;
        try
        {
            std::string param_name = _param_names.at(&param);
            
            rcl_interfaces::msg::SetParametersResult results = _node.set_parameter(rclcpp::Parameter(param_name, value));
            if (!results.successful)
            {
                ROS_WARN_STREAM("Parameter: " << param_name << " was not set:" << results.reason);
            }
        }
        catch(const std::out_of_range& e)
        {
            ROS_WARN_STREAM("Parameter was not internally declared.");
        }
        catch(const rclcpp::exceptions::ParameterNotDeclaredException& e)
        {
            std::string param_name = _param_names.at(&param);
            ROS_WARN_STREAM("Parameter: " << param_name << " was not declared:" << e.what());
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << e.what());
        }                            
    }

    // setRosParamValue - Used to set ROS parameter back to a valid value if an invalid value was set by user.
    void Parameters::setRosParamValue(const std::string param_name, void const* const value)
    {
        // setRosParamValue sets a value to a parameter in the parameters server.
        // The callback for the specified parameter is NOT called.
        self_set_parameters.push_back(param_name);            

        rclcpp::ParameterType param_type = _node.get_parameter(param_name).get_type();
        rcl_interfaces::msg::SetParametersResult results;
        switch(param_type)
        {
            case rclcpp::PARAMETER_BOOL:
                ROS_DEBUG_STREAM("Set " << param_name << " to " << *(bool*)value);
                results = _node.set_parameter(rclcpp::Parameter(param_name, *(bool*)value));
                break;
            case rclcpp::PARAMETER_INTEGER:
                ROS_DEBUG_STREAM("Set " << param_name << " to " << *(int*)value);
                results = _node.set_parameter(rclcpp::Parameter(param_name, *(int*)value));
                break;
            case rclcpp::PARAMETER_DOUBLE:
                ROS_DEBUG_STREAM("Set " << param_name << " to " << *(double*)value);
                results = _node.set_parameter(rclcpp::Parameter(param_name, *(double*)value));
                break;
            case rclcpp::PARAMETER_STRING:
                ROS_DEBUG_STREAM("Set " << param_name << " to " << *(std::string*)value);
                results = _node.set_parameter(rclcpp::Parameter(param_name, *(std::string*)value));
                break;
            default:
                ROS_ERROR_STREAM("Setting parameter of type " <<  _node.get_parameter(param_name).get_type_name() << " is not implemented.");
        }
        if (!results.successful)
        {
            ROS_WARN_STREAM("Parameter: " << param_name << " was not set:" << results.reason);
            self_set_parameters.pop_back();
        }
    }

    // queueSetRosValue - Set parameter in queue to be pushed to ROS parameter by monitor_update_functions
    template <class T>
    void Parameters::queueSetRosValue(const std::string& param_name, const T value)
    {
        std::vector<std::function<void()> > funcs;
        funcs.push_back([this, param_name, value](){setRosParamValue(param_name, &value);});
        pushUpdateFunctions(funcs);
    }


    void Parameters::removeParam(std::string param_name)
    {
        if (_node.has_parameter(param_name))
        {
            _node.undeclare_parameter(param_name);
        }
        _param_functions.erase(param_name);
    }

    /**
     * @brief Retrieves an existing parameter or declares it with an initial value if not already declared.
     *
     * This function ensures that a parameter exists within the node. If the parameter is already
     * declared, its current value is retrieved. Otherwise, the function declares the parameter 
     * with the provided `initial_value` and then returns it.
     *
     * @tparam T The type of the parameter.
     * @param param_name The name of the parameter.
     * @param initial_value The default value to declare if the parameter is not already declared.
     * @return The value of the parameter after declaration or retrieval.
     *
     * @note This function is useful for ensuring that required parameters are available without 
     * explicitly checking for their existence beforehand.
     */
    template<class T>
    T Parameters::getOrDeclareParameter(const std::string param_name, const T& initial_value)
    {
        // Declare parameter if not already declared
        if (!_node.has_parameter(param_name))
        {
            _node.declare_parameter(param_name, rclcpp::ParameterValue(initial_value));
        }

        // Retrieve the parameter value safely
        return getParam<T>(param_name);
    }

    template <class T>
    T Parameters::getParam(std::string param_name)
    {
        return _node.get_parameter(param_name).get_value<T>();
    }

    template void Parameters::setParamT<bool>(std::string param_name, bool& param, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);
    template void Parameters::setParamT<int>(std::string param_name, int& param, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);
    template void Parameters::setParamT<double>(std::string param_name, double& param, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);

    template bool Parameters::setParam<bool>(std::string param_name, const bool& initial_value, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);
    template int Parameters::setParam<int>(std::string param_name, const int& initial_value, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);
    template double Parameters::setParam<double>(std::string param_name, const double& initial_value, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);
    template std::string Parameters::setParam<std::string>(std::string param_name, const std::string& initial_value, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);

    template void Parameters::setParamValue<int>(int& param, const int& value);
    template void Parameters::setParamValue<bool>(bool& param, const bool& value);
    template void Parameters::setParamValue<double>(double& param, const double& value);

    template void Parameters::queueSetRosValue<std::string>(const std::string& param_name, const std::string value);
    template void Parameters::queueSetRosValue<int>(const std::string& param_name, const int value);

    template int Parameters::readAndDeleteParam<int>(std::string param_name, const int& initial_value);

    template int Parameters::getOrDeclareParameter<int>(const std::string param_name, const int& initial_value);
    template bool Parameters::getOrDeclareParameter<bool>(const std::string param_name, const bool& initial_value);
    template double Parameters::getOrDeclareParameter<double>(const std::string param_name, const double& initial_value);
    template std::string Parameters::getOrDeclareParameter<std::string>(const std::string param_name, const std::string& initial_value);

    template bool Parameters::getParam<bool>(std::string param_name);
}
