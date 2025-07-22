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

// cpplint: c system headers
#include "constants.h"
#include "base_realsense_node.h"
#include <builtin_interfaces/msg/time.hpp>
#include <console_bridge/console.h>
#include "rclcpp_components/register_node_macro.hpp"
#include "ros_node_base.h"
#include <algorithm>
#include <csignal>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>

namespace realsense2_camera
{
    class RealSenseNodeFactory : public RosNodeBase
    {
    public:

        explicit RealSenseNodeFactory(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
        RealSenseNodeFactory(
            const std::string & node_name, const std::string & ns,
            const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
        virtual ~RealSenseNodeFactory();
        #ifdef USE_LIFECYCLE_NODE
            using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
            CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
            CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
        #endif

    private:
        void init();
        void closeDevice();
        void startDevice();
        void stopDevice();
        void changeDeviceCallback(rs2::event_information& info);
        void getDevice(rs2::device_list list);
        void tryGetLogSeverity(rs2_log_severity& severity) const;
        static std::string parseUsbPort(std::string line);

        RosNodeBase::SharedPtr _node;
        rs2::device _device;
        std::unique_ptr<BaseRealSenseNode> _realSenseNode;
        rs2::context _ctx;
        std::string _serial_no;
        std::string _usb_port_id;
        std::string _device_type;
        double _wait_for_device_timeout;
        double _reconnect_timeout;
        bool _initial_reset;
        std::thread _query_thread;
        bool _is_alive;
        rclcpp::Logger _logger;
        std::shared_ptr<Parameters> _parameters;
    };
}//end namespace
