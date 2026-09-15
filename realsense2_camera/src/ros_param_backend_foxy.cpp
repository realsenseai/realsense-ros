// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#include "ros_param_backend.h"

namespace realsense2_camera
{
    void ParametersBackend::add_on_set_parameters_callback(ros2_param_callback_type callback)
    {
        _ros_callback = _node.add_on_set_parameters_callback(callback);
    }

    ParametersBackend::~ParametersBackend()
    {
        if (_ros_callback)
        {
            // Explicit cast to void* or standard handle depending on the compiler's rigorous type matching
            _node.remove_on_set_parameters_callback(reinterpret_cast<rclcpp::node_interfaces::OnSetParametersCallbackHandle*>(_ros_callback.get()));
            _ros_callback.reset();
        }
    }
}
