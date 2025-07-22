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

#include "../include/realsense_node_factory.h"
#include "../include/base_realsense_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#ifndef _WIN32
#include <sys/time.h>
#endif
#include <regex>

using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

RealSenseNodeFactory::RealSenseNodeFactory(const rclcpp::NodeOptions & node_options) :
    RosNodeBase("camera", "/camera", node_options),
    _logger(this->get_logger())
{
    #ifndef USE_LIFECYCLE_NODE
    init();
    #else
    RCLCPP_INFO(get_logger(), "Transition: Unconfigured...");
    #endif
}

#ifdef USE_LIFECYCLE_NODE
RealSenseNodeFactory::CallbackReturn
RealSenseNodeFactory::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "🚀 Lifecycle Transition | FROM: %s (%d) → TO: CONFIGURING",
                state.label().c_str(), state.id());
    init();
    return CallbackReturn::SUCCESS;
}

RealSenseNodeFactory::CallbackReturn
RealSenseNodeFactory::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "🚀 Lifecycle Transition | FROM: %s (%d) → TO: ACTIVATING",
                state.label().c_str(), state.id());
    startDevice();
    return CallbackReturn::SUCCESS;
}

RealSenseNodeFactory::CallbackReturn
RealSenseNodeFactory::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "🛑 Lifecycle Transition | FROM: %s (%d) → TO: DEACTIVATING",
                state.label().c_str(), state.id());
    stopDevice();
    return CallbackReturn::SUCCESS;
}

RealSenseNodeFactory::CallbackReturn
RealSenseNodeFactory::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "🧹 Lifecycle Transition | FROM: %s (%d) → TO: CLEANING UP",
                state.label().c_str(), state.id());
    closeDevice();
    return CallbackReturn::SUCCESS;
}

RealSenseNodeFactory::CallbackReturn
RealSenseNodeFactory::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "⚡ Lifecycle Transition | FROM: %s (%d) → TO: SHUTTING DOWN",
                state.label().c_str(), state.id());
    closeDevice();
    return CallbackReturn::SUCCESS;
}
#endif

RealSenseNodeFactory::RealSenseNodeFactory(const std::string & node_name, const std::string & ns,
                                           const rclcpp::NodeOptions & node_options) :
    RosNodeBase(node_name, ns, node_options),
    _logger(this->get_logger())
{
  init();
}

RealSenseNodeFactory::~RealSenseNodeFactory()
{
    _is_alive = false;
    if (_query_thread.joinable())
    {
        _query_thread.join();
    }
}

std::string RealSenseNodeFactory::parseUsbPort(std::string line)
{
    std::string port_id;
    std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*", std::regex_constants::ECMAScript);
    std::smatch base_match;
    bool found = std::regex_match(line, base_match, self_regex);
    if (found)
    {
        port_id = base_match[1].str();
        if (base_match[2].str().size() == 0)    //This is libuvc string. Remove counter is exists.
        {
            std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
            bool found_end = std::regex_match(port_id, base_match, end_regex);
            if (found_end)
            {
                port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
            }
        }
    }
    return port_id;
}

void RealSenseNodeFactory::getDevice(rs2::device_list list)
{
    if (!_device)
    {
        if (0 == list.size())
        {
            ROS_WARN("No RealSense devices were found!");
        }
        else
        {
            bool found = false;
            rs2::device dev;
            for (size_t count = 0; count < list.size(); count++)
            {
                try
                {
                    dev = list[count];
                }
                catch(const std::exception& ex)
                {
                    ROS_WARN_STREAM("Device " << count+1 << "/" << list.size() << " failed with exception: " << ex.what());
                    continue;
                }
                auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                ROS_INFO_STREAM("Device with serial number " << sn << " was found."<<std::endl);
                std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
                std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
                ROS_INFO_STREAM("Device with physical ID " << pn << " was found.");
                std::vector<std::string> results;
                ROS_INFO_STREAM("Device with name " << name << " was found.");
                std::string port_id = parseUsbPort(pn);

                std::string pid_str(dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
                if(pid_str != "DDS")
                {
                    if (port_id.empty())
                    {
                        std::stringstream msg;
                        msg << "Error extracting usb port from device with physical ID: " << pn << std::endl
                            << "Please report on github issue at https://github.com/IntelRealSense/realsense-ros";
                        if (_usb_port_id.empty())
                        {
                            ROS_WARN_STREAM(msg.str());
                        }
                        else
                        {
                            ROS_ERROR_STREAM(msg.str());
                            ROS_ERROR_STREAM("Please use serial number instead of usb port.");
                        }
                    }
                    else
                    {
                        ROS_INFO_STREAM("Device with port number " << port_id << " was found.");
                    }
                }

                bool found_device_type(true);
                if (!_device_type.empty())
                {
                    std::smatch match_results;
                    std::regex device_type_regex(_device_type.c_str(), std::regex::icase);
                    found_device_type = std::regex_search(name, match_results, device_type_regex);
                }

                if ((_serial_no.empty() || sn == _serial_no) && (_usb_port_id.empty() || port_id == _usb_port_id) && found_device_type)
                {
                    _device = dev;
                    _serial_no = sn;
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                std::string msg ("The requested device with ");
                bool add_and(false);
                if (!_serial_no.empty())
                {
                    msg += "serial number " + _serial_no;
                    add_and = true;
                }
                if (!_usb_port_id.empty())
                {
                    if (add_and)
                    {
                        msg += " and ";
                    }
                    msg += "usb port id " + _usb_port_id;
                    add_and = true;
                }
                if (!_device_type.empty())
                {
                    if (add_and)
                    {
                        msg += " and ";
                    }
                    msg += "device name containing " + _device_type;
                }
                msg += " is NOT found. Will Try again.";
                ROS_ERROR_STREAM(msg);
            }
            else
            {
                if (_device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
                {
                    std::string usb_type = _device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
                    ROS_INFO_STREAM("Device USB type: " << usb_type);
                    if (usb_type.find("2.") != std::string::npos)
                    {
                        ROS_WARN_STREAM("Device " << _serial_no << " is connected using a " << usb_type << " port. Reduced performance is expected.");
                    }
                }
            }
        }
    }

    if (_device && _initial_reset)
    {
        _initial_reset = false;
        try
        {
            ROS_INFO("Resetting device...");
            _device.hardware_reset();
            _device = rs2::device();

        }
        catch(const std::exception& ex)
        {
            ROS_WARN_STREAM("An exception has been thrown: " << __FILE__ << ":" << __LINE__ << ":" << ex.what());
        }
    }
}

void RealSenseNodeFactory::changeDeviceCallback(rs2::event_information& info)
{
    if (info.was_removed(_device))
    {
        ROS_ERROR("The device has been disconnected!");
        _realSenseNode.reset(nullptr);
        _device = rs2::device();
    }
    if (!_device)
    {
        rs2::device_list new_devices = info.get_new_devices();
        if (new_devices.size() > 0)
        {
            ROS_INFO("Checking new devices...");
            getDevice(new_devices);
            if (_device)
            {
                startDevice();
            }
        }
    }
}

std::string api_version_to_string(int version)
{
    std::ostringstream ss;
    if (version / 10000 == 0)
        ss << version;
    else
        ss << (version / 10000) << "." << (version % 10000) / 100 << "." << (version % 100);
    return ss.str();
}

void RealSenseNodeFactory::init()
{
    try
    {
        _is_alive = true;
        _parameters = std::make_shared<Parameters>(*this);

        rs2_error* e = nullptr;
        std::string running_librealsense_version(api_version_to_string(rs2_get_api_version(&e)));
        ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
        ROS_INFO("Built with LibRealSense v%s", RS2_API_VERSION_STR);
        ROS_INFO_STREAM("Running with LibRealSense v" << running_librealsense_version);
        if (RS2_API_VERSION_STR != running_librealsense_version)
        {
            ROS_WARN("***************************************************");
            ROS_WARN("** running with a different librealsense version **");
            ROS_WARN("** than the one the wrapper was compiled with!   **");
            ROS_WARN("***************************************************");
        }

        auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
        tryGetLogSeverity(severity);
        if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
            console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);

        rs2::log_to_console(severity);

#ifdef BPDEBUG
        std::cout << "Attach to Process: " << getpid() << std::endl;
        std::cout << "Press <ENTER> key to continue." << std::endl;
        std::cin.get();
#endif
        // Using `getOrDeclareParameter()` to avoid re-declaration issues
        _serial_no = _parameters->getOrDeclareParameter<std::string>("serial_no", "");
        _usb_port_id = _parameters->getOrDeclareParameter<std::string>("_usb_port_id", "");
        _device_type = _parameters->getOrDeclareParameter<std::string>("_device_type", "");
        _wait_for_device_timeout = _parameters->getOrDeclareParameter<double>("wait_for_device_timeout", -1.0);
        _reconnect_timeout = _parameters->getOrDeclareParameter<double>("reconnect_timeout", 6.0);

        // A ROS2 hack: until a better way is found to avoid auto convertion of strings containing only digits to integers:
        if (!_serial_no.empty() && _serial_no.front() == '_') _serial_no = _serial_no.substr(1);    // remove '_' prefix

        std::string rosbag_filename(_parameters->getOrDeclareParameter<std::string>("rosbag_filename", ""));
        if (!rosbag_filename.empty())
        {
            {
                ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
                rs2::context ctx;
                _device = ctx.load_device(rosbag_filename.c_str());
                _serial_no = _device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            }
            if (_device)
            {
                bool rosbag_loop(_parameters->getOrDeclareParameter<bool>("rosbag_loop", false));
                startDevice();

                if (rosbag_loop)
                {
                    auto playback = _device.as<rs2::playback>(); // Object to check the playback status periodically.
                    bool is_playing = true; // Flag to indicate if the playback is active

                    while (rclcpp::ok())
                    {
                        // Check the current status only if the playback is not active
                        auto status = playback.current_status();
                        if (!is_playing && status == RS2_PLAYBACK_STATUS_STOPPED)
                        {
                            RCLCPP_INFO(this->get_logger(), "Bag file playback has completed and it is going to be replayed.");
                            startDevice(); // Re-start bag file execution
                            is_playing = true; // Set the flag to true as playback has been restarted
                        }
                        else if (status != RS2_PLAYBACK_STATUS_STOPPED)
                        {
                            // If the playback status is not stopped, it means the playback is active
                            is_playing = true;
                        }
                        else
                        {
                            // If the playback status is stopped, set the flag to false
                            is_playing = false;
                        }

                        // Add a small delay to prevent busy-waiting
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                }
            }
        }
        else
        {
            _initial_reset = _parameters->getOrDeclareParameter<bool>("initial_reset",false);

            _query_thread = std::thread([=]()
            {
                std::chrono::milliseconds timespan(static_cast<int>(_reconnect_timeout*1e3));
                rclcpp::Time first_try_time = this->get_clock()->now();
                while (_is_alive && !_device)
                {
                    try
                    {
                        getDevice(_ctx.query_devices());
                        if (_device)
                        {
                            std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){changeDeviceCallback(info);};
                            _ctx.set_devices_changed_callback(change_device_callback_function);
                            // unconfigure lifecycle state ends here for lifecycled node
                            #ifndef USE_LIFECYCLE_NODE
                            startDevice();
                            #endif
                        }
                        else
                        {
                            std::chrono::milliseconds actual_timespan(timespan);
                            if (_wait_for_device_timeout > 0)
                            {
                                auto time_to_timeout(_wait_for_device_timeout - (this->get_clock()->now() - first_try_time).seconds());
                                if (time_to_timeout < 0)
                                {
                                    ROS_ERROR_STREAM("wait for device timeout of " << _wait_for_device_timeout << " secs expired");
                                    exit(1);
                                }
                                else
                                {
                                    double max_timespan_secs(std::chrono::duration_cast<std::chrono::seconds>(timespan).count());
                                    actual_timespan = std::chrono::milliseconds (static_cast<int>(std::min(max_timespan_secs, time_to_timeout) * 1e3));
                                }
                            }
                            std::this_thread::sleep_for(actual_timespan);
                        }
                    }
                    catch(const std::exception& e)
                    {
                        ROS_ERROR_STREAM("Error starting device: " << e.what());
                    }
                }
            });
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << __FILE__ << ":" << __LINE__ << ":" << ex.what());
        exit(1);
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        exit(1);
    }
}

void RealSenseNodeFactory::startDevice()
{
    if (_realSenseNode) _realSenseNode.reset();
    std::string device_name(_device.get_info(RS2_CAMERA_INFO_NAME));
    std::string pid_str(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
    uint16_t pid;

    if (device_name == "Intel RealSense D555")
    {
        // currently the PID of DDS devices is hardcoded as "DDS"
        // need to be fixed in librealsense
        pid = RS555_PID;
    }
    else
    {
        pid = std::stoi(pid_str, 0, 16);
    }
    try
    {
        switch(pid)
        {
        case RS400_PID:
        case RS405_PID:
        case RS410_PID:
        case RS460_PID:
        case RS415_PID:
        case RS420_PID:
        case RS421_PID:
        case RS420_MM_PID:
        case RS430_PID:
        case RS430i_PID:
        case RS430_MM_PID:
        case RS430_MM_RGB_PID:
        case RS435_RGB_PID:
        case RS435i_RGB_PID:
        case RS455_PID:
        case RS457_PID:
        case RS555_PID:
        case RS_USB2_PID:
            _realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(*this, _device, _parameters, this->get_node_options().use_intra_process_comms()));
            break;
        default:
            ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
            rclcpp::shutdown();
            exit(1);
        }
        _realSenseNode->publishTopics();
    }
    catch(const rs2::backend_error& e)
    {
        std::cerr << "Failed to start device: " << e.what() << '\n';
        _device.hardware_reset();
        _device = rs2::device();
    }
}

void RealSenseNodeFactory::stopDevice()
{
    if (_realSenseNode)
    {
        ROS_INFO("Stopping RealSense device...");
        _realSenseNode.reset(); // Calls the destructor and releases the device node
    }
}

void RealSenseNodeFactory::closeDevice()
{
    _is_alive = false;
    if (_query_thread.joinable())
    {
        _query_thread.join();
    }

    stopDevice();
    if (_device)
    {
        ROS_INFO("Closing RealSense device...");
        _ctx.set_devices_changed_callback([](rs2::event_information&) {});

        // To go into unconfigured lifecycle state for lifecycled node we have to also disconnect the device
        _device = rs2::device();
    }
}


void RealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity& severity) const
{
    static const char* severity_var_name = "LRS_LOG_LEVEL";
    auto content = getenv(severity_var_name);

    if (content)
    {
        std::string content_str(content);
        std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

        for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
        {
            auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
            std::transform(current.begin(), current.end(), current.begin(), ::toupper);
            if (content_str == current)
            {
                severity = (rs2_log_severity)i;
                break;
            }
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(realsense2_camera::RealSenseNodeFactory)
