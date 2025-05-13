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

#include <named_filter.h>
#include <fstream>
#include <sensor_msgs/point_cloud2_iterator.hpp>


using namespace realsense2_camera;

NamedFilter::NamedFilter(std::shared_ptr<rs2::filter> filter, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled, bool is_set_parameters):
    _filter(filter), _is_enabled(is_enabled), _params(parameters, logger), _logger(logger)
{
    if (is_set_parameters)
        setParameters();
}

void NamedFilter::setParameters(std::function<void(const rclcpp::Parameter&)> enable_param_func)
{
    std::stringstream module_name_str;
    std::string module_name = create_graph_resource_name(rs2_to_ros(_filter->get_info(RS2_CAMERA_INFO_NAME)));
    module_name_str << module_name;
    _params.registerDynamicOptions(*(_filter.get()), module_name_str.str());
    module_name_str << ".enable";

    _params.getParameters()->setParamT(module_name_str.str(), _is_enabled, enable_param_func);
    _parameters_names.push_back(module_name_str.str());
}

void NamedFilter::clearParameters()
{
    while ( !_parameters_names.empty() )
    {
        auto name = _parameters_names.back();
        _params.getParameters()->removeParam(name);
        _parameters_names.pop_back();
    }
}

rs2::frameset NamedFilter::Process(rs2::frameset frameset)
{
    if (_is_enabled)
    {
        return _filter->process(frameset);
    }
    else
    {
        return frameset;
    }
}

rs2::frame NamedFilter::Process(rs2::frame frame)
{
    if (_is_enabled)
    {
        return _filter->process(frame);
    }
    else
    {
        return frame;
    }
}
