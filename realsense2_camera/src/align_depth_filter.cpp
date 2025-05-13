// Copyright 2025 Intel Corporation. All Rights Reserved.
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

#include <align_depth_filter.h>


using namespace realsense2_camera;


AlignDepthFilter::AlignDepthFilter(std::shared_ptr<rs2::filter> filter,
    std::function<void(const rclcpp::Parameter&)> update_align_depth_func,
    std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled):
    NamedFilter(filter, parameters, logger, is_enabled, false)
{
    _params.registerDynamicOptions(*(_filter.get()), "align_depth");
    _params.getParameters()->setParamT("align_depth.enable", _is_enabled, update_align_depth_func);
    _parameters_names.push_back("align_depth.enable");
}
