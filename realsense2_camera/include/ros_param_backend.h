// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <functional>

namespace realsense2_camera
{
    class ParametersBackend
    {
        public:
            ParametersBackend(rclcpp::Node& node) : 
                _node(node),
                _logger(node.get_logger())
                {};
            ~ParametersBackend();

// --- JAZZY & HUMBLE COMPATIBILITY LAYER ---
#if defined(JAZZY)
    // Forced fallback for Jazzy systems using our custom CMake flag
    using ros2_param_callback_type = std::function<rcl_interfaces::msg::SetParametersResult(const std::vector<rclcpp::Parameter> &)>;
#elif defined(RCLCPP_HAS_OnSetParametersCallbackType)
    // Modern ROS 2 fallback alias
    using ros2_param_callback_type = rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
#else
    // Legacy ROS 2 (Foxy, Humble) alias
    using ros2_param_callback_type = rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;
#endif
// ------------------------------------------

            void add_on_set_parameters_callback(ros2_param_callback_type callback);

        private:
            rclcpp::Node& _node;
            rclcpp::Logger _logger;
            std::shared_ptr<void> _ros_callback;
    };
}
