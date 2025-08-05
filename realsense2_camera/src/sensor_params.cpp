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

#include <sensor_params.h>
#include <constants.h>
#include <iomanip>

namespace realsense2_camera
{
bool is_checkbox(rs2::options sensor, rs2_option option)
{
    rs2::option_range op_range = sensor.get_option_range(option);
    return op_range.max == 1.0f &&
        op_range.min == 0.0f &&
        op_range.step == 1.0f;
}

bool is_enum_option(rs2::options sensor, rs2_option option)
{
    static const int MAX_ENUM_OPTION_VALUES(100);
    static const float EPSILON(0.05);
    
    rs2::option_range op_range = sensor.get_option_range(option);
    if (abs((op_range.step - 1)) > EPSILON || (op_range.max > MAX_ENUM_OPTION_VALUES)) return false;
    for (auto i = op_range.min; i <= op_range.max; i += op_range.step)
    {
        if (sensor.get_option_value_description(option, i) == nullptr)
            continue;
        return true;
    }
    return false;
}

bool is_int_option(rs2::options sensor, rs2_option option)
{
    rs2::option_range op_range = sensor.get_option_range(option);
    return (op_range.step == 1.0);
}

std::map<std::string, int> get_enum_method(rs2::options sensor, rs2_option option)
{
    std::map<std::string, int> dict; // An enum to set size
    if (is_enum_option(sensor, option))
    {
        rs2::option_range op_range = sensor.get_option_range(option);
        const auto op_range_min = int(op_range.min);
        const auto op_range_max = int(op_range.max);
        const auto op_range_step = int(op_range.step);
        for (auto val = op_range_min; val <= op_range_max; val += op_range_step)
        {
            if (sensor.get_option_value_description(option, val) == nullptr)
                continue;
            dict[sensor.get_option_value_description(option, val)] = val;
        }
    }
    return dict;
}

template<class T>
void param_set_option(rs2::options sensor, rs2_option option, const rclcpp::Parameter& parameter)
{ 
    sensor.set_option(option, parameter.get_value<T>());
}

void SensorParams::clearParameters()
{
    while ( !_parameters_names.empty() )
    {
        auto name = _parameters_names.back();
        _parameters->removeParam(name);
        _parameters_names.pop_back();        
    }
}

SensorParams::~SensorParams()
{
    clearParameters();
}

double SensorParams::float_to_double(float val, rs2::option_range option_range)
{
    // casting from float to double, while trying to increase precision
    // in order to be comply with the FloatingPointRange configurations
    // and to succeed the rclcpp NodeParameters::declare_parameter call
    // for more info, see https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp
    // rcl_interfaces::msg::SetParametersResult and __are_doubles_equal methods
    float range = option_range.max - option_range.min;
    int steps = range / option_range.step;
    double step = static_cast<double>(range) / steps;
    double rounded_div = std::round((val - option_range.min) / step);
    return option_range.min + rounded_div * step;
}

//for more info, see https://docs.ros2.org/galactic/api/rcl_interfaces/msg/ParameterDescriptor.html
template<class T>
rcl_interfaces::msg::ParameterDescriptor SensorParams::get_parameter_descriptor(const std::string& option_name,
    rs2::option_range option_range, T option_value, const std::string& option_description, const std::string& description_addition)
{
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;

    // set descriptor name to be the option name in order to get more detailed warnings/errors from rclcpp
    parameter_descriptor.name = option_name;

    // create description stream, and start filling it.
    std::stringstream description_stream;
    description_stream << option_description << std::endl << description_addition;
    if(description_addition.length() > 0)
         description_stream << std::endl;

    if (std::is_same<T, double>::value)
    {
        parameter_descriptor.type = rclcpp::PARAMETER_DOUBLE;

        // set option range values (floats)
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = float_to_double(option_range.min, option_range);
        range.to_value = float_to_double(option_range.max, option_range);
        range.step = float_to_double(option_range.step, option_range);
        parameter_descriptor.floating_point_range.push_back(range);

        // add the default value of this option to the description stream
        description_stream << "Default value: " << static_cast<double>(option_range.def);        

        ROS_DEBUG_STREAM("Declare: DOUBLE::" << option_name << "=" << option_value << "  [" << option_range.min << ", " << option_range.max << "]");
    }
    else // T is bool or int
    {
        // set option range values (integers) (suitable for boolean ranges also: [0,1])
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = static_cast<int64_t>(option_range.min);
        range.to_value = static_cast<int64_t>(option_range.max);
        range.step = static_cast<uint64_t>(option_range.step);
        parameter_descriptor.integer_range.push_back(range);

        if (std::is_same<T, bool>::value) 
        {
            parameter_descriptor.type = rclcpp::PARAMETER_BOOL;

            // add the default value of this option to the description stream
            description_stream << "Default value: " << (option_range.def == 0 ? "False" : "True");

            ROS_DEBUG_STREAM("Declare: BOOL::" << option_name << "=" << (option_range.def == 0 ? "False" : "True") << "  [" << option_range.min << ", " << option_range.max << "]");
        }
        else
        {
            parameter_descriptor.type = rclcpp::PARAMETER_INTEGER;

            // add the default value of this option to the description stream
            description_stream << "Default value: " << static_cast<int64_t>(option_range.def);

            ROS_DEBUG_STREAM("Declare: INT::" << option_name << "=" << option_value << "  [" << option_range.min << ", " << option_range.max << "]");
        }
    }
    parameter_descriptor.description = description_stream.str();
    return parameter_descriptor;
}

template<class T>
void SensorParams::set_parameter(rs2::options sensor, rs2_option option, const std::string& module_name, const std::string& description_addition)
{
    // set the option name, for example: depth_module.exposure
    const std::string option_name(module_name + "." + create_graph_resource_name(rs2_option_to_string(option)));

    // get option current value, and option range values from the sensor
    T option_value;
    rs2::option_range option_range;
    try
    {
        option_range = sensor.get_option_range(option);
        float current_val = sensor.get_option(option);
        if(std::is_same<T, double>::value)
        {
            option_value = float_to_double(current_val, option_range);
        }
        else
        {
            option_value = static_cast<T>(current_val);
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An error has occurred while calling sensor for: " << option_name << ":" << ex.what());
        return;
    }

    // get parameter descriptor for this option
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor = 
        get_parameter_descriptor(option_name, option_range, option_value, sensor.get_option_description(option), description_addition);
    
    T new_val;
    try
    {
        new_val = (_parameters->setParam<T>(option_name, option_value, [option, sensor](const rclcpp::Parameter& parameter)
                    {
                        param_set_option<T>(sensor, option, parameter);
                    }, parameter_descriptor));
        _parameters_names.push_back(option_name);
    }
    catch(const rclcpp::exceptions::InvalidParameterValueException& e)
    {
        ROS_WARN_STREAM("Failed to set parameter:" << option_name << " = " << option_value << "[" << option_range.min << ", " << option_range.max << "]\n" << e.what());
        return;
    }
    
    if (new_val != option_value)
    {
        try
        {
            sensor.set_option(option, new_val);
        }
        catch(std::exception& e)
        {
            ROS_WARN_STREAM("Failed to set value to sensor: " << option_name << " = " << option_value << "[" << option_range.min << ", " << option_range.max << "]\n" << e.what());            
        }
    }
}

void SensorParams::registerDynamicOptions(rs2::options sensor, const std::string& module_name)
{
    for (auto i = 0; i < RS2_OPTION_COUNT; i++)
    {
        rs2_option option = static_cast<rs2_option>(i);
        const std::string option_name(module_name + "." + create_graph_resource_name(rs2_option_to_string(option)));
        if (!sensor.supports(option) || sensor.is_option_read_only(option))
        {
            continue;
        }
        if (is_checkbox(sensor, option))
        {
            set_parameter<bool>(sensor, option, module_name);
            continue;
        }
        const auto enum_dict = get_enum_method(sensor, option);
        if (enum_dict.empty())
        {
            if (is_int_option(sensor, option))
            {
                set_parameter<int>(sensor, option, module_name);
            }
            else
            {
                if (i == RS2_OPTION_DEPTH_UNITS)
                {
                    rs2::option_range op_range = sensor.get_option_range(option);
                    if (ROS_DEPTH_SCALE >= op_range.min && ROS_DEPTH_SCALE <= op_range.max)
                    {
                        try
                        {
                            sensor.set_option(option, ROS_DEPTH_SCALE);
                        }
                        catch(const std::exception& e)
                        {
                            std::cout << "Failed to set value: " << e.what() << std::endl;
                        }
                        op_range.min = ROS_DEPTH_SCALE;
                        op_range.max = ROS_DEPTH_SCALE;
                    }
                }
                else
                {
                    set_parameter<double>(sensor, option, module_name);
                }
            }
        }
        else
        {
            std::vector<std::pair<std::string, int> > enum_vec;
            size_t longest_desc(0);
            for (auto enum_iter : enum_dict)
            {
                enum_vec.push_back(std::make_pair(enum_iter.first, enum_iter.second));
                longest_desc = std::max(longest_desc, enum_iter.first.size());
            }
            sort(enum_vec.begin(), enum_vec.end(), [](std::pair<std::string, int> e1, std::pair<std::string, int> e2){return (e1.second < e2.second);});
            std::stringstream description;
            for (auto vec_iter : enum_vec)
            {
                description << std::setw(longest_desc+6) << std::left << vec_iter.first << " : " << vec_iter.second << std::endl;
            }
            set_parameter<int>(sensor, option, module_name, description.str());
        }
    }
}
}
