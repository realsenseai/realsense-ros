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

#include "../include/base_realsense_node.h"
#include <image_publisher.h>
#include <fstream>
#include <rclcpp/qos.hpp>
#include "pointcloud_filter.h"
#include "align_depth_filter.h"

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::setup()
{
#if defined (ACCELERATE_GPU_WITH_GLSL)
    initOpenGLProcessing(_accelerate_gpu_with_glsl);
    _is_accelerate_gpu_with_glsl_changed = false;
#endif
    setDynamicParams();
    startDiagnosticsUpdater();
    setAvailableSensors();
    SetBaseStream();
    setupFilters();
    setCallbackFunctions();
    monitoringProfileChanges();
    updateSensors();
    publishServices();
    publishActions();
}

void BaseRealSenseNode::monitoringProfileChanges()
{
    int time_interval(10000);
    std::function<void()> func = [this, time_interval](){
        std::unique_lock<std::mutex> lock(_profile_changes_mutex);
        while(_is_running) {
            _cv_mpc.wait_for(lock, std::chrono::milliseconds(time_interval),
                                               [&]{return (!_is_running || _is_profile_changed
                                                                        || _is_align_depth_changed
                                                                        #if defined (ACCELERATE_GPU_WITH_GLSL)
                                                                            || _is_accelerate_gpu_with_glsl_changed
                                                                        #endif
                                                           );});

            if (_is_running && (_is_profile_changed
                                        || _is_align_depth_changed
                                        #if defined (ACCELERATE_GPU_WITH_GLSL)
                                            || _is_accelerate_gpu_with_glsl_changed
                                        #endif
                                ))
            {
                ROS_DEBUG("Profile has changed");
                try
                {
                    updateSensors();
                }
                catch(const std::exception& e)
                {
                    ROS_ERROR_STREAM("Error updating the sensors: " << e.what());
                }
                _is_profile_changed = false;
                _is_align_depth_changed = false;

                #if defined (ACCELERATE_GPU_WITH_GLSL)
                    _is_accelerate_gpu_with_glsl_changed = false;
                #endif
            }
        }
    };
    _monitoring_pc = std::make_shared<std::thread>(func);
}

void BaseRealSenseNode::setAvailableSensors()
{
    if (!_json_file_path.empty())
    {
        if (_dev.is<rs400::advanced_mode>())
        {
            std::stringstream ss;
            std::ifstream in(_json_file_path);
            if (in.is_open())
            {
                ss << in.rdbuf();
                std::string json_file_content = ss.str();

                auto adv = _dev.as<rs400::advanced_mode>();
                adv.load_json(json_file_content);
                ROS_INFO_STREAM("JSON file is loaded! (" << _json_file_path << ")");
            }
            else
                ROS_WARN_STREAM("JSON file provided doesn't exist! (" << _json_file_path << ")");
        }
        else
            ROS_WARN("Device does not support advanced settings!");
    }
    else
        ROS_INFO("JSON file is not provided");

    auto device_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
    ROS_INFO_STREAM("Device Name: " << device_name);

    auto serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    ROS_INFO_STREAM("Device Serial No: " << serial_no);

    auto device_port_id = _dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);

    ROS_INFO_STREAM("Device physical port: " << device_port_id);

    auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    ROS_INFO_STREAM("Device FW version: " << fw_ver);

    auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
    ROS_INFO_STREAM("Device Product ID: 0x" << pid);

    ROS_INFO_STREAM("Sync Mode: " << ((_sync_frames)?"On":"Off"));

    std::function<void(rs2::frame)> frame_callback_function = [this](rs2::frame frame){
        bool is_filter(_filters.end() != find_if(_filters.begin(), _filters.end(), [](std::shared_ptr<NamedFilter> f){return (f->is_enabled()); }));
        if (_sync_frames || is_filter)
            this->_asyncer.invoke(frame);
        else
            frame_callback(frame);
    };

    std::function<void(rs2::frame)> imu_callback_function = [this](rs2::frame frame){
        imu_callback(frame);
        if (_imu_sync_method != imu_sync_method::NONE)
            imu_callback_sync(frame);
    };

    std::function<void(rs2::frame)> multiple_message_callback_function = [this](rs2::frame frame){multiple_message_callback(frame, _imu_sync_method);};

    std::function<void()> update_sensor_func = [this](){
        {
            std::lock_guard<std::mutex> lock_guard(_profile_changes_mutex);
            _is_profile_changed = true;
        }
        _cv_mpc.notify_one();
    };

    std::function<void()> hardware_reset_func = [this](){hardwareResetRequest();};

    _dev_sensors = _dev.query_sensors();

    for(auto&& sensor : _dev_sensors)
    {
        const std::string module_name(rs2_to_ros(sensor.get_info(RS2_CAMERA_INFO_NAME)));
        std::unique_ptr<RosSensor> rosSensor;
        if (sensor.is<rs2::depth_sensor>() ||
            sensor.is<rs2::color_sensor>())
        {
            ROS_DEBUG_STREAM("Set " << module_name << " as VideoSensor.");
            rosSensor = std::make_unique<RosSensor>(sensor, _parameters, frame_callback_function, update_sensor_func, hardware_reset_func, _diagnostics_updater, _logger, _use_intra_process, _dev.is<playback>());
        }
        else if (sensor.is<rs2::motion_sensor>())
        {
            ROS_DEBUG_STREAM("Set " << module_name << " as ImuSensor.");
            rosSensor = std::make_unique<RosSensor>(sensor, _parameters, imu_callback_function, update_sensor_func, hardware_reset_func, _diagnostics_updater, _logger, false, _dev.is<playback>());
        }
        else
        {
            ROS_WARN_STREAM("Module Name \"" << module_name << "\" does not define a callback.");
            continue;
        }
        _available_ros_sensors.push_back(std::move(rosSensor));
    }

}

void BaseRealSenseNode::setCallbackFunctions()
{
    _asyncer.start([this](rs2::frame f)
    {
        frame_callback(f);
    });
}

void BaseRealSenseNode::stopPublishers(const std::vector<stream_profile>& profiles)
{
    for (auto& profile : profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (profile.is<rs2::video_stream_profile>())
        {
            _image_publishers.erase(sip);
            _info_publishers.erase(sip);
            _depth_aligned_image_publishers.erase(sip);
            _depth_aligned_info_publisher.erase(sip);
        }
        else if (profile.is<rs2::motion_stream_profile>())
        {
            _is_accel_enabled = false;
            _is_gyro_enabled = false;
            _synced_imu_publisher.reset();
            _imu_publishers.erase(sip);
            _imu_info_publishers.erase(sip);
        }
        _metadata_publishers.erase(sip);
        _extrinsics_publishers.erase(sip);

        if (_publish_tf)
        {
            eraseTransformMsgs(sip, profile);
        }
    }
}

void BaseRealSenseNode::startPublishers(const std::vector<stream_profile>& profiles, const RosSensor& sensor)
{
    const std::string module_name(create_graph_resource_name(rs2_to_ros(sensor.get_info(RS2_CAMERA_INFO_NAME))));
    for (auto& profile : profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        std::string stream_name(STREAM_NAME(sip));

        rmw_qos_profile_t qos = sensor.getQOS(sip);
        rmw_qos_profile_t info_qos = sensor.getInfoQOS(sip);

        if (profile.is<rs2::video_stream_profile>())
        {
            if(profile.stream_type() == RS2_STREAM_COLOR)
                _is_color_enabled = true;
            else if (profile.stream_type() == RS2_STREAM_DEPTH)
                _is_depth_enabled = true;
            std::stringstream image_raw, camera_info;
            bool rectified_image = false;
            if (sensor.rs2::sensor::is<rs2::depth_sensor>())
                rectified_image = true;

            // adding "~/" to the topic name will add node namespace and node name to the topic
            // see "Private Namespace Substitution Character" section on https://design.ros2.org/articles/topic_and_service_names.html
            image_raw << "~/" << stream_name << "/image_" << ((rectified_image)?"rect_":"") << "raw";
            camera_info << "~/" << stream_name << "/camera_info";

            // We can use 2 types of publishers:
            // 1. Native RCL publisher (supports intra-process zero-copy communication)
            // 2. Image-transport package publisher (adds a compressed image topic if installed)

            #ifdef USE_LIFECYCLE_NODE
            // Always use `image_rcl_publisher` when lifecycle nodes are enabled
            _image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, image_raw.str(), qos);
            #else
            // 🚀 Use intra-process if enabled, otherwise use image_transport
            if (_use_intra_process)
            {
                _image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, image_raw.str(), qos);
            }
            else
            {
                _image_publishers[sip] = std::make_shared<image_transport_publisher>(_node, image_raw.str(), qos);
                ROS_DEBUG_STREAM("image transport publisher was created for topic " << image_raw.str());
            }
            #endif


            _info_publishers[sip] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(camera_info.str(),
                                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));

            if (_align_depth_filter->is_enabled() && (sip != DEPTH))
            {
                std::stringstream aligned_image_raw, aligned_camera_info;
                aligned_image_raw << "~/" << "aligned_depth_to_" << stream_name << "/image_raw";
                aligned_camera_info << "~/" << "aligned_depth_to_" << stream_name << "/camera_info";

                std::string aligned_stream_name = "aligned_depth_to_" + stream_name;

                // We can use 2 types of publishers:
                // Native RCL publisher that support intra-process zero-copy comunication
                // image-transport package publisher that add's a commpressed image topic if the package is installed
                #ifdef USE_LIFECYCLE_NODE
                    // Always use `image_rcl_publisher` when lifecycle nodes are enabled
                    _depth_aligned_image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, aligned_image_raw.str(), qos);
                #else
                    // Use intra-process if enabled, otherwise use image_transport
                    if (_use_intra_process)
                    {
                        _depth_aligned_image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, aligned_image_raw.str(), qos);
                    }
                    else
                    {
                        _depth_aligned_image_publishers[sip] = std::make_shared<image_transport_publisher>(_node, aligned_image_raw.str(), qos);
                        ROS_DEBUG_STREAM("image transport publisher was created for topic " << aligned_image_raw.str());
                    }
                #endif

                _depth_aligned_info_publisher[sip] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(aligned_camera_info.str(),
                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));
            }
        }
        else if (profile.is<rs2::motion_stream_profile>())
        {
            if(profile.stream_type() == RS2_STREAM_ACCEL)
                _is_accel_enabled = true;
            else if (profile.stream_type() == RS2_STREAM_GYRO)
                _is_gyro_enabled = true;

            std::stringstream data_topic_name, info_topic_name;
            data_topic_name << "~/" << stream_name << "/sample";
            _imu_publishers[sip] = _node.create_publisher<sensor_msgs::msg::Imu>(data_topic_name.str(),
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
            // Publish Intrinsics:
            info_topic_name << "~/" << stream_name << "/imu_info";

            // IMU Info will have latched QoS, and it will publish its data only once during the ROS Node lifetime.
            // intra-process do not support latched QoS, so we need to disable intra-process for this topic
            rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
            options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
            _imu_info_publishers[sip] = _node.create_publisher<IMUInfo>(info_topic_name.str(),
                                                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_latched), rmw_qos_profile_latched),
                                                                        std::move(options));
            IMUInfo info_msg = getImuInfo(profile);
            _imu_info_publishers[sip]->publish(info_msg);
        }
        std::string topic_metadata("~/" + stream_name + "/metadata");
        _metadata_publishers[sip] = _node.create_publisher<realsense2_camera_msgs::msg::Metadata>(topic_metadata, 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));

        if (!((rs2::stream_profile)profile==(rs2::stream_profile)_base_profile))
        {

            // intra-process do not support latched QoS, so we need to disable intra-process for this topic
            rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
            options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
            rmw_qos_profile_t extrinsics_qos = rmw_qos_profile_latched;

            std::string topic_extrinsics("~/extrinsics/" + create_graph_resource_name(ros_stream_to_string(_base_profile.stream_type()) + "_to_" + stream_name));
            _extrinsics_publishers[sip] = _node.create_publisher<realsense2_camera_msgs::msg::Extrinsics>(topic_extrinsics,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(extrinsics_qos), extrinsics_qos), std::move(options));
        }
    }
    if (_is_accel_enabled && _is_gyro_enabled && (_imu_sync_method > imu_sync_method::NONE))
    {
        rmw_qos_profile_t qos = _use_intra_process ? qos_string_to_qos(DEFAULT_QOS) : qos_string_to_qos(HID_QOS);
        
        _synced_imu_publisher = std::make_shared<SyncedImuPublisher>(_node.create_publisher<sensor_msgs::msg::Imu>("~/imu", 
                                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos)));
    }

}

void BaseRealSenseNode::startRGBDPublisherIfNeeded()
{
    _rgbd_publisher.reset();
    if(_enable_rgbd && !_rgbd_publisher)
    {
        if (_sync_frames && _is_color_enabled && _is_depth_enabled && _align_depth_filter->is_enabled())
        {
            rmw_qos_profile_t qos = _use_intra_process ? qos_string_to_qos(DEFAULT_QOS) : qos_string_to_qos(IMAGE_QOS);

            // adding "~/" to the topic name will add node namespace and node name to the topic
            // see "Private Namespace Substitution Character" section on https://design.ros2.org/articles/topic_and_service_names.html
            _rgbd_publisher = _node.create_publisher<realsense2_camera_msgs::msg::RGBD>("~/rgbd",
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
        }
        else {
            ROS_ERROR("In order to get rgbd topic enabled, "\
             "you should enable: color stream, depth stream, sync_mode and align_depth");
        }
    }
}

void BaseRealSenseNode::updateSensors()
{
    std::lock_guard<std::mutex> lock_guard(_update_sensor_mutex);
    try{
        stopRequiredSensors();

        #if defined (ACCELERATE_GPU_WITH_GLSL)
            if (_is_accelerate_gpu_with_glsl_changed)
            {
                shutdownOpenGLProcessing();

                initOpenGLProcessing(_accelerate_gpu_with_glsl);
            }
        #endif

        startUpdatedSensors();
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::stopRequiredSensors()
{
    try{
        for(auto&& sensor : _available_ros_sensors)
        {
            std::string module_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME)));
            // if active_profiles != wanted_profiles: stop sensor.
            std::vector<stream_profile> wanted_profiles;

            bool is_profile_changed(sensor->getUpdatedProfiles(wanted_profiles));
            bool is_video_sensor = (sensor->is<rs2::depth_sensor>() || sensor->is<rs2::color_sensor>());

            // do all updates if profile has been changed, or if the align depth filter or gpu acceleration status has been changed
            // and we are on a video sensor. TODO: explore better options to monitor and update changes
            // without resetting the whole sensors and topics.
            if (is_profile_changed || (is_video_sensor && (_is_align_depth_changed
                                                                #if defined (ACCELERATE_GPU_WITH_GLSL)
                                                                    || _is_accelerate_gpu_with_glsl_changed
                                                                #endif
                                                            )))
            {
                std::vector<stream_profile> active_profiles = sensor->get_active_streams();
                if (is_profile_changed
                        #if defined (ACCELERATE_GPU_WITH_GLSL)
                            || _is_accelerate_gpu_with_glsl_changed
                        #endif
                    )
                {
                    // Start/stop sensors only if profile or gpu acceleration status was changed
                    // No need to start/stop sensors if align_depth was changed
                    ROS_INFO_STREAM("Stopping Sensor: " << module_name);
                    sensor->stop();
                }
                stopPublishers(active_profiles);
            }
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::startUpdatedSensors()
{
    try{
        for(auto&& sensor : _available_ros_sensors)
        {
            std::string module_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME)));
            // if active_profiles != wanted_profiles: stop sensor.
            std::vector<stream_profile> wanted_profiles;

            bool is_profile_changed(sensor->getUpdatedProfiles(wanted_profiles));
            bool is_video_sensor = (sensor->is<rs2::depth_sensor>() || sensor->is<rs2::color_sensor>());

            if (is_profile_changed || (is_video_sensor && (_is_align_depth_changed
                                                                #if defined (ACCELERATE_GPU_WITH_GLSL)
                                                                    || _is_accelerate_gpu_with_glsl_changed
                                                                #endif
                                                            )))
            {
                if (!wanted_profiles.empty())
                {
                    startPublishers(wanted_profiles, *sensor);
                    updateProfilesStreamCalibData(wanted_profiles);
                    if (_publish_tf)
                    {
                        std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
                        for (auto &profile : wanted_profiles)
                        {
                            calcAndAppendTransformMsgs(profile, _base_profile);
                        }
                    }

                    if (is_profile_changed
                            #if defined (ACCELERATE_GPU_WITH_GLSL)
                                || _is_accelerate_gpu_with_glsl_changed
                            #endif
                        )
                    {
                        // Start/stop sensors only if profile or gpu acceleration was changed
                        // No need to start/stop sensors if align_depth was changed
                        ROS_INFO_STREAM("Starting Sensor: " << module_name);
                        sensor->start(wanted_profiles);
                    }

                    if (sensor->rs2::sensor::is<rs2::depth_sensor>())
                    {
                        _depth_scale_meters = sensor->as<rs2::depth_sensor>().get_depth_scale();
                    }
                }
            }
        }
        if (_publish_tf)
        {
            std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
            publishStaticTransforms();
        }
        startRGBDPublisherIfNeeded();
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::publishServices()
{
    // adding "~/" to the service name will add node namespace and node name to the service
    // see "Private Namespace Substitution Character" section on https://design.ros2.org/articles/topic_and_service_names.html
    _reset_srv = _node.create_service<std_srvs::srv::Empty>(
            "~/hw_reset",
            [&](const std_srvs::srv::Empty::Request::SharedPtr req,
                        std_srvs::srv::Empty::Response::SharedPtr res)
                        {handleHWReset(req, res);});

    _device_info_srv = _node.create_service<realsense2_camera_msgs::srv::DeviceInfo>(
            "~/device_info",
            [&](const realsense2_camera_msgs::srv::DeviceInfo::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::DeviceInfo::Response::SharedPtr res)
                        {getDeviceInfo(req, res);});

    _calib_config_read_srv = _node.create_service<realsense2_camera_msgs::srv::CalibConfigRead>(
            "~/calib_config_read",
            [&](const realsense2_camera_msgs::srv::CalibConfigRead::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::CalibConfigRead::Response::SharedPtr res)
                        {CalibConfigReadService(req, res);});

    _calib_config_write_srv = _node.create_service<realsense2_camera_msgs::srv::CalibConfigWrite>(
            "~/calib_config_write",
            [&](const realsense2_camera_msgs::srv::CalibConfigWrite::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::CalibConfigWrite::Response::SharedPtr res)
                        {CalibConfigWriteService(req, res);});

}

void BaseRealSenseNode::publishActions()
{

    using namespace std::placeholders;
    _triggered_calibration_action_server = rclcpp_action::create_server<TriggeredCalibration>(
      _node.get_node_base_interface(),
      _node.get_node_clock_interface(),
      _node.get_node_logging_interface(),
      _node.get_node_waitables_interface(),
      "~/triggered_calibration",
      std::bind(&BaseRealSenseNode::TriggeredCalibrationHandleGoal, this, _1, _2),
      std::bind(&BaseRealSenseNode::TriggeredCalibrationHandleCancel, this, _1),
      std::bind(&BaseRealSenseNode::TriggeredCalibrationHandleAccepted, this, _1));

}

void BaseRealSenseNode::handleHWReset(const std_srvs::srv::Empty::Request::SharedPtr req,
                                const std_srvs::srv::Empty::Response::SharedPtr res)
{
    (void)req;
    (void)res;
    ROS_INFO_STREAM("Reset requested through service call");
    if (_dev)
    {
        try
        {
            for(auto&& sensor : _available_ros_sensors)
            {
                std::string module_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME)));
                ROS_INFO_STREAM("Stopping Sensor: " << module_name);
                sensor->stop();
            }
            ROS_INFO("Resetting device...");
            _dev.hardware_reset();
        }
        catch(const std::exception& ex)
        {
            ROS_WARN_STREAM("An exception has been thrown: " << __FILE__ << ":" << __LINE__ << ":" << ex.what());
        }
    }
}

void BaseRealSenseNode::getDeviceInfo(const realsense2_camera_msgs::srv::DeviceInfo::Request::SharedPtr,
                                            realsense2_camera_msgs::srv::DeviceInfo::Response::SharedPtr res)
{
    res->device_name = _dev.supports(RS2_CAMERA_INFO_NAME) ? create_graph_resource_name(_dev.get_info(RS2_CAMERA_INFO_NAME)) : "";
    res->serial_number = _dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
    res->firmware_version = _dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION) ? _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) : "";
    res->usb_type_descriptor = _dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) ? _dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) : "";
    res->firmware_update_id = _dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) ? _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) : "";

    std::stringstream sensors_names;

    for(auto&& sensor : _available_ros_sensors)
    {
        sensors_names << create_graph_resource_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME))) << ",";
    }

    res->sensors = sensors_names.str().substr(0, sensors_names.str().size()-1);
    res->physical_port = _dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT) ? _dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) : "";
}

void BaseRealSenseNode::CalibConfigReadService(const realsense2_camera_msgs::srv::CalibConfigRead::Request::SharedPtr req,
    realsense2_camera_msgs::srv::CalibConfigRead::Response::SharedPtr res){
    try
    {
        (void)req; // silence unused parameter warning
        res->calib_config = _dev.as<rs2::auto_calibrated_device>().get_calibration_config();
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::CalibConfigWriteService(const realsense2_camera_msgs::srv::CalibConfigWrite::Request::SharedPtr req,
    realsense2_camera_msgs::srv::CalibConfigWrite::Response::SharedPtr res){
    try
    {
        _dev.as<rs2::auto_calibrated_device>().set_calibration_config(req->calib_config);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}
