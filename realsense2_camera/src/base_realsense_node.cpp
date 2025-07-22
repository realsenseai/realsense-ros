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
#include "assert.h"
#include <algorithm>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <fstream>
#include <image_publisher.h>

// Header files for disabling intra-process comms for static broadcaster.
#include <rclcpp/publisher_options.hpp>
#include <tf2_ros/qos.hpp>
#include "pointcloud_filter.h"
#include "align_depth_filter.h"

using namespace realsense2_camera;

SyncedImuPublisher::SyncedImuPublisher(rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher, 
                                       std::size_t waiting_list_size):
            _publisher(imu_publisher), _pause_mode(false),
            _waiting_list_size(waiting_list_size), _is_enabled(false)
            {}

SyncedImuPublisher::~SyncedImuPublisher()
{
    try
    {
        PublishPendingMessages();
    }
    catch(...){} // Not allowed to throw from Dtor
}

void SyncedImuPublisher::Publish(sensor_msgs::msg::Imu imu_msg)
{
    std::lock_guard<std::mutex> lock_guard(_mutex);
    if (_pause_mode)
    {
        if (_pending_messages.size() >= _waiting_list_size)
        {
            throw std::runtime_error("SyncedImuPublisher inner list reached maximum size of " + std::to_string(_pending_messages.size()));
        }
        _pending_messages.push(imu_msg);
    }
    else
    {
        _publisher->publish(imu_msg);
    }
    return;
}

void SyncedImuPublisher::Pause()
{
    if (!_is_enabled) return;
    std::lock_guard<std::mutex> lock_guard(_mutex);
    _pause_mode = true;
}

void SyncedImuPublisher::Resume()
{
    std::lock_guard<std::mutex> lock_guard(_mutex);
    PublishPendingMessages();
    _pause_mode = false;
}

void SyncedImuPublisher::PublishPendingMessages()
{
    while (!_pending_messages.empty())
    {
        const sensor_msgs::msg::Imu &imu_msg = _pending_messages.front();
        _publisher->publish(imu_msg);
        _pending_messages.pop();
    }
}
size_t SyncedImuPublisher::getNumSubscribers()
{ 
    if (!_publisher) return 0;
    return _publisher->get_subscription_count();
}

BaseRealSenseNode::BaseRealSenseNode(RosNodeBase& node,
                                     rs2::device dev,
                                     std::shared_ptr<Parameters> parameters,
                                     bool use_intra_process) :
    _is_running(true),
    _node(node),
    _logger(node.get_logger()),
    _parameters(parameters),
    _dev(dev),
    _json_file_path(""),
    _depth_scale_meters(0),
    _clipping_distance(0),
    _linear_accel_cov(0),
    _angular_velocity_cov(0),
    _hold_back_imu_for_frames(false),
    _publish_tf(false),
    _tf_publish_rate(TF_PUBLISH_RATE),
    _diagnostics_period(0),
    _use_intra_process(use_intra_process),
    _is_initialized_time_base(false),
    _camera_time_base(0),
    _sync_frames(SYNC_FRAMES),
    _enable_rgbd(ENABLE_RGBD),
    _is_color_enabled(false),
    _is_depth_enabled(false),
    _is_accel_enabled(false),
    _is_gyro_enabled(false),
    _pointcloud(false),
    _imu_sync_method(imu_sync_method::NONE),
    _is_profile_changed(false),
    _is_align_depth_changed(false)
#if defined (ACCELERATE_GPU_WITH_GLSL)
    ,_app(1280, 720, "RS_GLFW_Window"),
    _accelerate_gpu_with_glsl(false),
    _is_accelerate_gpu_with_glsl_changed(false)
#endif
{
    if ( use_intra_process )
    {
        ROS_INFO("Intra-Process communication enabled");
    }

    initializeFormatsMaps();
    _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_PROJECTOR_TEMPERATURE};
}

BaseRealSenseNode::~BaseRealSenseNode()
{
    // Kill dynamic transform thread
    _is_running = false;
    _cv_tf.notify_one();
    if (_tf_t && _tf_t->joinable())
        _tf_t->join();

    _cv_temp.notify_one();
    _cv_mpc.notify_one();
    if (_monitoring_t && _monitoring_t->joinable())
    {
        _monitoring_t->join();
    }
    if (_monitoring_pc && _monitoring_pc->joinable())
    {
        _monitoring_pc->join();
    }
    clearParameters();
    try
    {
        for(auto&& sensor : _available_ros_sensors)
        {
            sensor->stop();
        }
    }
    catch(...){} // Not allowed to throw from Dtor
}

void BaseRealSenseNode::hardwareResetRequest()
{
    ROS_ERROR_STREAM("Performing Hardware Reset.");
    _dev.hardware_reset();
}

void BaseRealSenseNode::publishTopics()
{
    getParameters();
    setup();
    ROS_INFO_STREAM("RealSense Node Is Up!");
}

void BaseRealSenseNode::initializeFormatsMaps()
{
    // from rs2_format to OpenCV format
    // https://docs.opencv.org/3.4/d1/d1b/group__core__hal__interface.html
    // https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html
    // CV_<bit-depth>{U|S|F}C(<number_of_channels>)
    // where U is unsigned integer type, S is signed integer type, and F is float type.
    // For example, CV_8UC1 means a 8-bit single-channel array,
    // CV_32FC2 means a 2-channel (complex) floating-point array, and so on.
    _rs_format_to_cv_format[RS2_FORMAT_Y8] = CV_8UC1;
    _rs_format_to_cv_format[RS2_FORMAT_Y16] = CV_16UC1;
    _rs_format_to_cv_format[RS2_FORMAT_Z16] = CV_16UC1;
    _rs_format_to_cv_format[RS2_FORMAT_RGB8] = CV_8UC3;
    _rs_format_to_cv_format[RS2_FORMAT_BGR8] = CV_8UC3;
    _rs_format_to_cv_format[RS2_FORMAT_RGBA8] = CV_8UC4;
    _rs_format_to_cv_format[RS2_FORMAT_BGRA8] = CV_8UC4;
    _rs_format_to_cv_format[RS2_FORMAT_YUYV] = CV_8UC2;
    _rs_format_to_cv_format[RS2_FORMAT_UYVY] = CV_8UC2;
    // _rs_format_to_cv_format[RS2_FORMAT_M420] = not supported yet in ROS2
    _rs_format_to_cv_format[RS2_FORMAT_RAW8] = CV_8UC1;
    _rs_format_to_cv_format[RS2_FORMAT_RAW10] = CV_16UC1;
    _rs_format_to_cv_format[RS2_FORMAT_RAW16] = CV_16UC1;

    // from rs2_format to ROS2 image msg encoding (format)
    // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
    // http://docs.ros.org/en/jade/api/sensor_msgs/html/image__encodings_8h_source.html
    _rs_format_to_ros_format[RS2_FORMAT_Y8] = sensor_msgs::image_encodings::MONO8;
    _rs_format_to_ros_format[RS2_FORMAT_Y16] = sensor_msgs::image_encodings::MONO16;
    _rs_format_to_ros_format[RS2_FORMAT_Z16] = sensor_msgs::image_encodings::TYPE_16UC1;
    _rs_format_to_ros_format[RS2_FORMAT_RGB8] = sensor_msgs::image_encodings::RGB8;
    _rs_format_to_ros_format[RS2_FORMAT_BGR8] = sensor_msgs::image_encodings::BGR8;
    _rs_format_to_ros_format[RS2_FORMAT_RGBA8] = sensor_msgs::image_encodings::RGBA8;
    _rs_format_to_ros_format[RS2_FORMAT_BGRA8] = sensor_msgs::image_encodings::BGRA8;
    _rs_format_to_ros_format[RS2_FORMAT_YUYV] = sensor_msgs::image_encodings::YUV422_YUY2;
    _rs_format_to_ros_format[RS2_FORMAT_UYVY] = sensor_msgs::image_encodings::YUV422;
    // _rs_format_to_ros_format[RS2_FORMAT_M420] =  not supported yet in ROS2
    _rs_format_to_ros_format[RS2_FORMAT_RAW8] = sensor_msgs::image_encodings::TYPE_8UC1;
    _rs_format_to_ros_format[RS2_FORMAT_RAW10] = sensor_msgs::image_encodings::TYPE_16UC1;
    _rs_format_to_ros_format[RS2_FORMAT_RAW16] = sensor_msgs::image_encodings::TYPE_16UC1;
}

void BaseRealSenseNode::setupFilters()
{
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::decimation_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::hdr_merge>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::sequence_id_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::disparity_transform>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::spatial_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::temporal_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::hole_filling_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::disparity_transform>(false), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::rotation_filter>(std::vector< rs2_stream >{ RS2_STREAM_DEPTH, RS2_STREAM_COLOR, RS2_STREAM_INFRARED }), _parameters, _logger));

    /* 
    update_align_depth_func is being used in the align depth filter for triggiring the thread that monitors profile
    changes (_monitoring_pc) on every disable/enable of the align depth filter. This filter enablement/disablement affects
    several topics creation/destruction, therefore, refreshing the topics is required similarly to what is done when turning on/off a sensor.
    See BaseRealSenseNode::monitoringProfileChanges() as reference.
    */ 
    std::function<void(const rclcpp::Parameter&)> update_align_depth_func = [this](const rclcpp::Parameter&){
        {
            std::lock_guard<std::mutex> lock_guard(_profile_changes_mutex);
            _is_align_depth_changed = true;
        }
        _cv_mpc.notify_one();
    };

#if defined (ACCELERATE_GPU_WITH_GLSL)
    _colorizer_filter = std::make_shared<NamedFilter>(std::make_shared<rs2::gl::colorizer>(), _parameters, _logger); 
    _pc_filter = std::make_shared<PointcloudFilter>(std::make_shared<rs2::gl::pointcloud>(), _node, _parameters, _logger);
#else
    _colorizer_filter = std::make_shared<NamedFilter>(std::make_shared<rs2::colorizer>(), _parameters, _logger);
    _pc_filter = std::make_shared<PointcloudFilter>(std::make_shared<rs2::pointcloud>(), _node, _parameters, _logger);
#endif

    // Apply PointCloud filter before applying Align-depth as it requires original depth image not aligned-depth image.
    _filters.push_back(_pc_filter);

    _align_depth_filter = std::make_shared<AlignDepthFilter>(std::make_shared<rs2::align>(RS2_STREAM_COLOR), update_align_depth_func, _parameters, _logger);
    _filters.push_back(_align_depth_filter);

    // Apply Colorizer filter after applying Align-Depth to get colorized aligned depth image.
    _filters.push_back(_colorizer_filter);
}

cv::Mat& BaseRealSenseNode::fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image)
{
    static const float meter_to_mm = 0.001f;
    if (fabs(_depth_scale_meters - meter_to_mm) < 1e-6)
    {
        to_image = from_image;
        return to_image;
    }

    if (to_image.size() != from_image.size())
    {
        to_image.create(from_image.rows, from_image.cols, from_image.type());
    }

    CV_Assert(CV_MAKETYPE(from_image.depth(),from_image.channels()) == _rs_format_to_cv_format[RS2_FORMAT_Z16]);

    int nRows = from_image.rows;
    int nCols = from_image.cols;

    if (from_image.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    int i,j;
    const uint16_t* p_from;
    uint16_t* p_to;
    for( i = 0; i < nRows; ++i)
    {
        p_from = from_image.ptr<uint16_t>(i);
        p_to = to_image.ptr<uint16_t>(i);
        for ( j = 0; j < nCols; ++j)
        {
            p_to[j] = p_from[j] * _depth_scale_meters / meter_to_mm;
        }
    }
    return to_image;
}

void BaseRealSenseNode::clip_depth(rs2::depth_frame depth_frame, float clipping_dist)
{
    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));
    uint16_t clipping_value = static_cast<uint16_t>(clipping_dist / _depth_scale_meters);

    int width = depth_frame.get_width();
    int height = depth_frame.get_height();

    #ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    #endif
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Check if the depth value is greater than the threashold
            if (p_depth_frame[depth_pixel_index] > clipping_value)
            {
                p_depth_frame[depth_pixel_index] = 0; //Set to invalid (<=0) value.
            }
        }
    }
}

sensor_msgs::msg::Imu BaseRealSenseNode::CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data)
{
    sensor_msgs::msg::Imu imu_msg;
    rclcpp::Time t(gyro_data.m_time_ns);  //rclcpp::Time(uint64_t nanoseconds)
    imu_msg.header.stamp = t;

    imu_msg.angular_velocity.x = gyro_data.m_data.x();
    imu_msg.angular_velocity.y = gyro_data.m_data.y();
    imu_msg.angular_velocity.z = gyro_data.m_data.z();

    imu_msg.linear_acceleration.x = accel_data.m_data.x();
    imu_msg.linear_acceleration.y = accel_data.m_data.y();
    imu_msg.linear_acceleration.z = accel_data.m_data.z();
    return imu_msg;
}

template <typename T> T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

void BaseRealSenseNode::FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
    static std::deque<CimuData> _imu_history;
    _imu_history.push_back(imu_data);
    stream_index_pair type(imu_data.m_type);
    imu_msgs.clear();

    if ((type != ACCEL) || _imu_history.size() < 3)
        return;
    
    std::deque<CimuData> gyros_data;
    CimuData accel0, accel1, crnt_imu;

    while (_imu_history.size()) 
    {
        crnt_imu = _imu_history.front();
        _imu_history.pop_front();
        if (!accel0.is_set() && crnt_imu.m_type == ACCEL) 
        {
            accel0 = crnt_imu;
        } 
        else if (accel0.is_set() && crnt_imu.m_type == ACCEL) 
        {
            accel1 = crnt_imu;
            const double dt = accel1.m_time_ns - accel0.m_time_ns;

            while (gyros_data.size())
            {
                CimuData crnt_gyro = gyros_data.front();
                gyros_data.pop_front();
                const double alpha = (crnt_gyro.m_time_ns - accel0.m_time_ns) / dt;
                CimuData crnt_accel(ACCEL, lerp(accel0.m_data, accel1.m_data, alpha), crnt_gyro.m_time_ns);
                imu_msgs.push_back(CreateUnitedMessage(crnt_accel, crnt_gyro));
            }
            accel0 = accel1;
        } 
        else if (accel0.is_set() && crnt_imu.m_time_ns >= accel0.m_time_ns && crnt_imu.m_type == GYRO)
        {
            gyros_data.push_back(crnt_imu);
        }
    }
    _imu_history.push_back(crnt_imu);
    return;
}

void BaseRealSenseNode::FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
    stream_index_pair type(imu_data.m_type);

    static CimuData _accel_data(ACCEL, {0,0,0}, -1.0);
    if (ACCEL == type)
    {
        _accel_data = imu_data;
        return;
    }
    if (!_accel_data.is_set())
        return;

    imu_msgs.push_back(CreateUnitedMessage(_accel_data, imu_data));
}

void BaseRealSenseNode::ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu& imu_msg)
{
    imu_msg.header.frame_id = IMU_OPTICAL_FRAME_ID;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;

    imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imu_msg.linear_acceleration_covariance = { _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov};
    imu_msg.angular_velocity_covariance = { _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov};
}

void BaseRealSenseNode::imu_callback_sync(rs2::frame frame, imu_sync_method sync_method)
{
    static std::mutex m_mutex;

    m_mutex.lock();

    auto stream = frame.get_profile().stream_type();
    auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
    double frame_time = frame.get_timestamp();

    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    if (_synced_imu_publisher && (0 != _synced_imu_publisher->getNumSubscribers()))
    {
        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
        CimuData imu_data(stream_index, v, frameSystemTimeSec(frame).nanoseconds());
        std::deque<sensor_msgs::msg::Imu> imu_msgs;
        switch (sync_method)
        {
            case imu_sync_method::COPY:
                FillImuData_Copy(imu_data, imu_msgs);
                break;
            case imu_sync_method::LINEAR_INTERPOLATION:
                FillImuData_LinearInterpolation(imu_data, imu_msgs);
                break;
            case imu_sync_method::NONE: //Cannot really be NONE. Just to avoid compilation warning.
                throw std::runtime_error("sync_method in this section can be either COPY or LINEAR_INTERPOLATION");
                break;
        }
        while (imu_msgs.size())
        {
            sensor_msgs::msg::Imu imu_msg = imu_msgs.front();
            ImuMessage_AddDefaultValues(imu_msg);
            _synced_imu_publisher->Publish(imu_msg);
            ROS_DEBUG("Publish united %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
            imu_msgs.pop_front();
         }
    }
    m_mutex.unlock();
}

void BaseRealSenseNode::imu_callback(rs2::frame frame)
{
    auto stream = frame.get_profile().stream_type();
    double frame_time = frame.get_timestamp();
    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                ros_stream_to_string(frame.get_profile().stream_type()).c_str(),
                frame.get_profile().stream_index(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));

    stream_index_pair stream_index;
    
    if(stream == GYRO.first)
    {
        stream_index = GYRO;
    }
    else if(stream == ACCEL.first)
    {
        stream_index = ACCEL;
    }
    else if(stream == MOTION.first)
    {
        stream_index = MOTION;
    }
    else 
    {
        ROS_ERROR("Unknown IMU stream type.");
        return;
    }

    rclcpp::Time t(frameSystemTimeSec(frame));

    if(_imu_publishers.find(stream_index) == _imu_publishers.end())
    {
        ROS_DEBUG("Received IMU callback while topic does not exist");
        return;
    }

    if (0 != _imu_publishers[stream_index]->get_subscription_count())
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        ImuMessage_AddDefaultValues(imu_msg);
        imu_msg.header.frame_id = OPTICAL_FRAME_ID(stream_index);

        if (MOTION == stream_index)
        {
            auto combined_motion_data = frame.as<rs2::motion_frame>().get_combined_motion_data();

            imu_msg.linear_acceleration.x = combined_motion_data.linear_acceleration.x;
            imu_msg.linear_acceleration.y = combined_motion_data.linear_acceleration.y;
            imu_msg.linear_acceleration.z = combined_motion_data.linear_acceleration.z;

            imu_msg.angular_velocity.x = combined_motion_data.angular_velocity.x;
            imu_msg.angular_velocity.y = combined_motion_data.angular_velocity.y;
            imu_msg.angular_velocity.z = combined_motion_data.angular_velocity.z;

            imu_msg.orientation.x = combined_motion_data.orientation.x;
            imu_msg.orientation.y = combined_motion_data.orientation.y;
            imu_msg.orientation.z = combined_motion_data.orientation.z;
            imu_msg.orientation.w = combined_motion_data.orientation.w;

        }
        else
        {
            auto motion_data = frame.as<rs2::motion_frame>().get_motion_data();
            if (GYRO == stream_index)
            {
                imu_msg.angular_velocity.x = motion_data.x;
                imu_msg.angular_velocity.y = motion_data.y;
                imu_msg.angular_velocity.z = motion_data.z;
            }
            else // ACCEL == stream_index
            {
                imu_msg.linear_acceleration.x = motion_data.x;
                imu_msg.linear_acceleration.y = motion_data.y;
                imu_msg.linear_acceleration.z = motion_data.z;
            }
        }

        imu_msg.header.stamp = t;
        _imu_publishers[stream_index]->publish(imu_msg);
        ROS_DEBUG("Publish %s stream", ros_stream_to_string(frame.get_profile().stream_type()).c_str());
    }
    publishMetadata(frame, t, OPTICAL_FRAME_ID(stream_index));
}


void BaseRealSenseNode::frame_callback(rs2::frame frame)
{
    if (_synced_imu_publisher)
        _synced_imu_publisher->Pause();
    double frame_time = frame.get_timestamp();

    // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
    // and the incremental timestamp from the camera.
    // In sync mode the timestamp is based on ROS time
    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    rclcpp::Time t(frameSystemTimeSec(frame));
    if (frame.is<rs2::frameset>())
    {
        ROS_DEBUG("Frameset arrived.");
        auto frameset = frame.as<rs2::frameset>();
        ROS_DEBUG("List of frameset before applying filters: size: %d", static_cast<int>(frameset.size()));
        for (auto it = frameset.begin(); it != frameset.end(); ++it)
        {
            auto f = (*it);
            auto stream_type = f.get_profile().stream_type();
            auto stream_index = f.get_profile().stream_index();
            auto stream_format = f.get_profile().format();
            auto stream_unique_id = f.get_profile().unique_id();

            ROS_DEBUG("Frameset contain (%s, %d, %s %d) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                        rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), stream_unique_id, frame.get_frame_number(), frame_time, t.nanoseconds());
        }
        // Clip depth_frame for max range:
        rs2::depth_frame original_depth_frame = frameset.get_depth_frame();
        if (original_depth_frame && _clipping_distance > 0)
        {
            clip_depth(original_depth_frame, _clipping_distance);
        }

        rs2::video_frame original_color_frame = frameset.get_color_frame();
        rs2::video_frame original_infra2_frame = frameset.get_infrared_frame(2);

        ROS_DEBUG("num_filters: %d", static_cast<int>(_filters.size()));
        for (auto filter_it : _filters)
        {
            frameset = filter_it->Process(frameset);
        }

        ROS_DEBUG("List of frameset after applying filters: size: %d", static_cast<int>(frameset.size()));
        bool sent_depth_frame(false);
        for (auto it = frameset.begin(); it != frameset.end(); ++it)
        {
            auto f = (*it);
            auto stream_type = f.get_profile().stream_type();
            auto stream_index = f.get_profile().stream_index();
            auto stream_format = f.get_profile().format();
            stream_index_pair sip{stream_type,stream_index};

            ROS_DEBUG("Frameset contain (%s, %d, %s) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu", 
                rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), f.get_frame_number(), frame_time, t.nanoseconds());
            if (f.is<rs2::video_frame>())
                ROS_DEBUG_STREAM("frame: " << f.as<rs2::video_frame>().get_width() << " x " << f.as<rs2::video_frame>().get_height());

            if (f.is<rs2::points>())
            {
                publishPointCloud(f.as<rs2::points>(), t, frameset);
            }
            else
            {
                if (stream_type == RS2_STREAM_DEPTH)
                {
                    if (sent_depth_frame) continue;
                    sent_depth_frame = true;
                    if (original_color_frame && _align_depth_filter->is_enabled())
                    {
                        publishFrame(f, t, COLOR, _depth_aligned_image, _depth_aligned_info_publisher, _depth_aligned_image_publishers, false);
                        continue;
                    }
                    if (original_infra2_frame && _align_depth_filter->is_enabled())
                    {
                        publishFrame(f, t, INFRA2, _depth_aligned_image, _depth_aligned_info_publisher, _depth_aligned_image_publishers, false);
                        continue;
                    }
                }
                publishFrame(f, t, sip, _images, _info_publishers, _image_publishers);
            }
        }
        if (original_depth_frame && _align_depth_filter->is_enabled())
        {
            rs2::frame frame_to_send;
            if (_colorizer_filter->is_enabled())
                frame_to_send = _colorizer_filter->Process(original_depth_frame);
            else
                frame_to_send = original_depth_frame;
            publishFrame(frame_to_send, t, DEPTH, _images, _info_publishers, _image_publishers);

            // Publish RGBD only if rgbd enabled and both depth and color frames exist.
            // On this line we already know original_depth_frame is valid.
            if(_enable_rgbd && original_color_frame)
            {
                auto color_format = original_color_frame.get_profile().format();
                auto depth_format = original_depth_frame.get_profile().format();
                publishRGBD(_images[COLOR], color_format, _depth_aligned_image[COLOR], depth_format, t);
            }  
        }
    }
    else if (frame.is<rs2::video_frame>())
    {
        auto stream_type = frame.get_profile().stream_type();
        auto stream_index = frame.get_profile().stream_index();
        ROS_DEBUG("Single video frame arrived (%s, %d). frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                    rs2_stream_to_string(stream_type), stream_index, frame.get_frame_number(), frame_time, t.nanoseconds());
            
        stream_index_pair sip{stream_type,stream_index};
        if (frame.is<rs2::depth_frame>())
        {
            if (_clipping_distance > 0)
            {
                clip_depth(frame, _clipping_distance);
            }
        }
        publishFrame(frame, t, sip, _images, _info_publishers, _image_publishers);
     }
     if (_synced_imu_publisher)
        _synced_imu_publisher->Resume();
} // frame_callback

void BaseRealSenseNode::multiple_message_callback(rs2::frame frame, imu_sync_method sync_method)
{
    auto stream = frame.get_profile().stream_type();
    switch (stream)
    {
        case RS2_STREAM_GYRO:
        case RS2_STREAM_ACCEL:
            if (sync_method > imu_sync_method::NONE) imu_callback_sync(frame, sync_method);
            else imu_callback(frame);
            break;
        default:
            frame_callback(frame);
    }
}

bool BaseRealSenseNode::setBaseTime(double frame_time, rs2_timestamp_domain time_domain)
{
    if (time_domain == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)
    {
        ROS_WARN_ONCE("Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)");
    }

    if (time_domain == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        ROS_WARN("frame's time domain is HARDWARE_CLOCK. Timestamps may reset periodically.");
        _ros_time_base = _node.now();
        _camera_time_base = frame_time;
        return true;
    }
    return false;
}

uint64_t BaseRealSenseNode::millisecondsToNanoseconds(double timestamp_ms)
{
        // modf breaks input into an integral and fractional part
        double int_part_ms, fract_part_ms;
        fract_part_ms = modf(timestamp_ms, &int_part_ms);

        //convert both parts to ns
        static constexpr uint64_t milli_to_nano = 1000000;
        uint64_t int_part_ns = static_cast<uint64_t>(int_part_ms) * milli_to_nano;
        uint64_t fract_part_ns = static_cast<uint64_t>(std::round(fract_part_ms * milli_to_nano));

        return int_part_ns + fract_part_ns;
}

rclcpp::Time BaseRealSenseNode::frameSystemTimeSec(rs2::frame frame)
{
    double timestamp_ms = frame.get_timestamp();
    if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        double elapsed_camera_ns = millisecondsToNanoseconds(timestamp_ms - _camera_time_base);

        /*
        Fixing deprecated-declarations compilation error for EOL distro (foxy)
        */
#if defined(FOXY)
        auto duration = rclcpp::Duration(elapsed_camera_ns);
#else
        auto duration = rclcpp::Duration::from_nanoseconds(elapsed_camera_ns);
#endif

        return rclcpp::Time(_ros_time_base + duration);
    }
    else
    {
        return rclcpp::Time(millisecondsToNanoseconds(timestamp_ms));
    }
}

void BaseRealSenseNode::updateProfilesStreamCalibData(const std::vector<rs2::stream_profile>& profiles)
{
    std::shared_ptr<rs2::stream_profile> left_profile;
    std::shared_ptr<rs2::stream_profile> right_profile;
    for (auto& profile : profiles)
    {
        if (profile.is<rs2::video_stream_profile>())
        {
            updateStreamCalibData(profile.as<rs2::video_stream_profile>());

            // stream index: 1=left, 2=right
            if (profile.stream_index() == 1) { left_profile = std::make_shared<rs2::stream_profile>(profile); }
            if (profile.stream_index() == 2) { right_profile = std::make_shared<rs2::stream_profile>(profile);  }
        }
    }
    if (left_profile && right_profile) {
        updateExtrinsicsCalibData(left_profile->as<rs2::video_stream_profile>(), right_profile->as<rs2::video_stream_profile>());
    }
}

void BaseRealSenseNode::updateStreamCalibData(const rs2::video_stream_profile& video_profile)
{
    stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};

    rs2_intrinsics intrinsic;
    try
    {
        intrinsic = video_profile.get_intrinsics();
    }
    catch(const std::exception& ex)
    {
        // e.g. infra1/infra2 in Y16i format (calibration mode) doesn't have intrinsics.
        ROS_WARN_STREAM("No intrinsics available for this stream profile. Using zeroed intrinsics as default.");
        intrinsic = { 0, 0, 0, 0, 0, 0, RS2_DISTORTION_NONE ,{ 0,0,0,0,0 } };
    }

    _camera_info[stream_index].width = intrinsic.width;
    _camera_info[stream_index].height = intrinsic.height;
    _camera_info[stream_index].header.frame_id = OPTICAL_FRAME_ID(stream_index);

    _camera_info[stream_index].k.at(0) = intrinsic.fx;
    _camera_info[stream_index].k.at(2) = intrinsic.ppx;
    _camera_info[stream_index].k.at(4) = intrinsic.fy;
    _camera_info[stream_index].k.at(5) = intrinsic.ppy;
    _camera_info[stream_index].k.at(8) = 1;

    _camera_info[stream_index].p.at(0) = _camera_info[stream_index].k.at(0);
    _camera_info[stream_index].p.at(1) = 0;
    _camera_info[stream_index].p.at(2) = _camera_info[stream_index].k.at(2);
    _camera_info[stream_index].p.at(3) = 0;
    _camera_info[stream_index].p.at(4) = 0;
    _camera_info[stream_index].p.at(5) = _camera_info[stream_index].k.at(4);
    _camera_info[stream_index].p.at(6) = _camera_info[stream_index].k.at(5);
    _camera_info[stream_index].p.at(7) = 0;
    _camera_info[stream_index].p.at(8) = 0;
    _camera_info[stream_index].p.at(9) = 0;
    _camera_info[stream_index].p.at(10) = 1;
    _camera_info[stream_index].p.at(11) = 0;

    // set R (rotation matrix) values to identity matrix
    _camera_info[stream_index].r.at(0) = 1.0;
    _camera_info[stream_index].r.at(1) = 0.0;
    _camera_info[stream_index].r.at(2) = 0.0;
    _camera_info[stream_index].r.at(3) = 0.0;
    _camera_info[stream_index].r.at(4) = 1.0;
    _camera_info[stream_index].r.at(5) = 0.0;
    _camera_info[stream_index].r.at(6) = 0.0;
    _camera_info[stream_index].r.at(7) = 0.0;
    _camera_info[stream_index].r.at(8) = 1.0;

    int coeff_size(5);
    if (intrinsic.model == RS2_DISTORTION_KANNALA_BRANDT4)
    {
        _camera_info[stream_index].distortion_model = "equidistant";
        coeff_size = 4;
    } else {
        _camera_info[stream_index].distortion_model = "plumb_bob";
    }

    _camera_info[stream_index].d.resize(coeff_size);
    for (int i = 0; i < coeff_size; i++)
    {
        _camera_info[stream_index].d.at(i) = intrinsic.coeffs[i];
    }

    if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR])
    {
        _camera_info[stream_index].p.at(3) = 0;     // Tx
        _camera_info[stream_index].p.at(7) = 0;     // Ty
    }
}

void BaseRealSenseNode::updateExtrinsicsCalibData(const rs2::video_stream_profile& left_video_profile, const rs2::video_stream_profile& right_video_profile)
{
    stream_index_pair left{left_video_profile.stream_type(), left_video_profile.stream_index()};
    stream_index_pair right{right_video_profile.stream_type(), right_video_profile.stream_index()};

    float fx = _camera_info[right].k.at(0);
    float fy = _camera_info[right].k.at(4);
    const auto& ex = right_video_profile.get_extrinsics_to(left_video_profile);
    _camera_info[right].header.frame_id = OPTICAL_FRAME_ID(left);
    _camera_info[right].p.at(3) = -fx * ex.translation[0] + 0.0; // Tx - avoid -0.0 values.
    _camera_info[right].p.at(7) = -fy * ex.translation[1] + 0.0; // Ty - avoid -0.0 values.
}

void BaseRealSenseNode::SetBaseStream()
{
    const std::vector<stream_index_pair> base_stream_priority = {DEPTH};
    std::set<stream_index_pair> checked_sips;
    std::map<stream_index_pair, rs2::stream_profile> available_profiles;
    for(auto&& sensor : _available_ros_sensors)
    {
        for (auto& profile : sensor->get_stream_profiles())
        {
            stream_index_pair sip(profile.stream_type(), profile.stream_index());
            if (available_profiles.find(sip) != available_profiles.end())
                continue;
            available_profiles[sip] = profile;
        }
    }
    
    std::vector<stream_index_pair>::const_iterator base_stream(base_stream_priority.begin());
    while((base_stream != base_stream_priority.end()) && (available_profiles.find(*base_stream) == available_profiles.end()))
    {
        base_stream++;
    }
    if (base_stream == base_stream_priority.end())
    {
        throw std::runtime_error("No known base_stream found for transformations.");
    }
    ROS_DEBUG_STREAM("SELECTED BASE:" << base_stream->first << ", " << base_stream->second);

    _base_profile = available_profiles[*base_stream];
}

void BaseRealSenseNode::publishPointCloud(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset)
{
    std::string frame_id = OPTICAL_FRAME_ID(DEPTH);
    _pc_filter->Publish(pc, t, frameset, frame_id);
}


Extrinsics BaseRealSenseNode::rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics) const
{
    Extrinsics extrinsicsMsg;
    for (int i = 0; i < 9; ++i)
    {
        extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
        if (i < 3)
            extrinsicsMsg.translation[i] = extrinsics.translation[i];
    }
    return extrinsicsMsg;
}

IMUInfo BaseRealSenseNode::getImuInfo(const rs2::stream_profile& profile)
{
    IMUInfo info{};
    auto sp = profile.as<rs2::motion_stream_profile>();
    rs2_motion_device_intrinsic imuIntrinsics;
    try
    {
        imuIntrinsics = sp.get_motion_intrinsics();
    }
    catch(const std::runtime_error &ex)
    {
        ROS_DEBUG_STREAM("No Motion Intrinsics available.");
        imuIntrinsics = {{{1,0,0,0},{0,1,0,0},{0,0,1,0}}, {0,0,0}, {0,0,0}};
    }

    auto index = 0;
    stream_index_pair sip(profile.stream_type(), profile.stream_index());
    info.header.frame_id = OPTICAL_FRAME_ID(sip);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            info.data[index] = imuIntrinsics.data[i][j];
            ++index;
        }
        info.noise_variances[i] =  imuIntrinsics.noise_variances[i];
        info.bias_variances[i] = imuIntrinsics.bias_variances[i];
    }
    return info;
}

bool BaseRealSenseNode::fillROSImageMsgAndReturnStatus(
    const cv::Mat& cv_matrix_image,
    const stream_index_pair& stream,
    unsigned int width,
    unsigned int height,
    const rs2_format& stream_format,
    const rclcpp::Time& t,
    sensor_msgs::msg::Image* img_msg_ptr)
{
    if (cv_matrix_image.empty())
    {
        ROS_ERROR_STREAM("cv::Mat is empty. Ignoring this frame.");
        return false;
    }
    else if (_rs_format_to_ros_format.find(stream_format) == _rs_format_to_ros_format.end())
    {
        ROS_ERROR_STREAM("Format " << rs2_format_to_string(stream_format) << " is not supported in ROS2 image messages"
                                   << "Please try different format of this stream.");
        return false;
    }
    // Convert the CV::Mat into a ROS image message (1 copy is done here)
    cv_bridge::CvImage(std_msgs::msg::Header(), _rs_format_to_ros_format[stream_format], cv_matrix_image).toImageMsg(*img_msg_ptr);

    // Convert OpenCV Mat to ROS Image
    img_msg_ptr->header.frame_id = OPTICAL_FRAME_ID(stream);
    img_msg_ptr->header.stamp = t;
    img_msg_ptr->height = height;
    img_msg_ptr->width = width;
    img_msg_ptr->is_bigendian = false;
    img_msg_ptr->step = width * cv_matrix_image.elemSize();
    return true;
}

bool BaseRealSenseNode::fillCVMatImageAndReturnStatus(
    rs2::frame& frame,
    std::map<stream_index_pair, cv::Mat>& images,
    unsigned int width,
    unsigned int height,
    const stream_index_pair& stream)
{
    auto& image = images[stream];
    auto stream_format = frame.get_profile().format();

    if (_rs_format_to_cv_format.find(stream_format) == _rs_format_to_cv_format.end())
    {
        ROS_ERROR_STREAM("Format " << rs2_format_to_string(stream_format) << " is not supported in realsense2_camera node."
                                   << "\nPlease try different format of this stream.");
        return false;
    }
    // we try to reduce image creation as much we can, so we check if the same image structure
    // was already created before, and we fill this image next with the frame data
    // image.create() should be called once per <stream>_<profile>_<format>
    if (image.size() != cv::Size(width, height) || CV_MAKETYPE(image.depth(), image.channels()) != _rs_format_to_cv_format[stream_format])
    {
        image.create(height, width, _rs_format_to_cv_format[stream_format]);
    }

    image.data = (uint8_t*)frame.get_data();

    if (frame.is<rs2::depth_frame>())
    {
        image = fix_depth_scale(image, _depth_scaled_image[stream]);
    }

    return true;
}

void BaseRealSenseNode::publishFrame(
    rs2::frame f,
    const rclcpp::Time& t,
    const stream_index_pair& stream,
    std::map<stream_index_pair, cv::Mat>& images,
    const std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>& info_publishers,
    const std::map<stream_index_pair, std::shared_ptr<image_publisher>>& image_publishers,
    const bool is_publishMetadata)
{
    ROS_DEBUG("publishFrame(...)");
    unsigned int width = 0;
    unsigned int height = 0;
    auto stream_format = RS2_FORMAT_ANY;
    if (f.is<rs2::video_frame>())
    {
        auto timage = f.as<rs2::video_frame>();
        width = timage.get_width();
        height = timage.get_height();
        stream_format = timage.get_profile().format();
    }
    else
    {
        ROS_ERROR("f.is<rs2::video_frame>() check failed. Frame was dropped.");
        return;
    }

    // Publish stream image
    if (image_publishers.find(stream) != image_publishers.end())
    {
        auto &image_publisher = image_publishers.at(stream);
        cv::Mat image_cv_matrix;

        // if rgbd has subscribers we fetch the CV image here
        if (_rgbd_publisher && 0 != _rgbd_publisher->get_subscription_count())
        {
            if (fillCVMatImageAndReturnStatus(f, images, width, height, stream))
            {
                image_cv_matrix = images[stream];
            }
        }

        // if depth/color has subscribers, ask first if rgbd already fetched
        // the images from the frame. if not, fetch the relevant color/depth image.
        if (0 != image_publisher->get_subscription_count())
        {
            if (image_cv_matrix.empty() && fillCVMatImageAndReturnStatus(f, images, width, height, stream))
            {
                image_cv_matrix = images[stream];
            }

            // Prepare image topic to be published
            // We use UniquePtr for allow intra-process publish when subscribers of that type are available
            sensor_msgs::msg::Image::UniquePtr img_msg_ptr(new sensor_msgs::msg::Image());
            if (!img_msg_ptr)
            {
                ROS_ERROR("Sensor image message allocation failed. Frame was dropped.");
                return;
            }

            if (fillROSImageMsgAndReturnStatus(image_cv_matrix, stream, width, height, stream_format, t, img_msg_ptr.get()))
            {

                // Transfer the unique pointer ownership to the RMW
                sensor_msgs::msg::Image *msg_address = img_msg_ptr.get();
                image_publisher->publish(std::move(img_msg_ptr));

                ROS_DEBUG_STREAM(rs2_stream_to_string(f.get_profile().stream_type()) << " stream published, message address: " << std::hex << msg_address);
            }
            else
            {
                ROS_ERROR("Could not fill ROS message. Frame was dropped.");
            }
        }
    }

    // Publish stream camera info
    if(info_publishers.find(stream) != info_publishers.end())
    {
        auto& info_publisher = info_publishers.at(stream);

        // If rgbd has subscribers, get the camera info of color/detph sensors from _camera_info map.
        // We need this camera info to fill the rgbd msg, regardless if there subscribers to depth/color camera info.
        // We are not publishing this cam_info here, but will be published by rgbd publisher.
        if (_rgbd_publisher && 0 != _rgbd_publisher->get_subscription_count())
        {
            auto& cam_info = _camera_info.at(stream);

            // Fix the camera info if needed, usually only in the first time
            // when we init this object in the _camera_info map
            if (cam_info.width != width)
            {
                updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
            }
            cam_info.header.stamp = t;
        }

        // If depth/color camera info has subscribers get camera info from _camera_info map,
        // and publish this msg.
        if(0 != info_publisher->get_subscription_count())
        {
            auto& cam_info = _camera_info.at(stream);

            // Fix the camera info if needed, usually only in the first time
            // when we init this object in the _camera_info map
            if (cam_info.width != width)
            {
                updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
            }
            cam_info.header.stamp = t;
            info_publisher->publish(cam_info);
        }
    }

    // Publish stream metadata
    if (is_publishMetadata)
    {
        publishMetadata(f, t, OPTICAL_FRAME_ID(stream));
    }
}


void BaseRealSenseNode::publishRGBD(
    const cv::Mat& rgb_cv_matrix,
    const rs2_format& color_format,
    const cv::Mat& depth_cv_matrix,
    const rs2_format& depth_format,
    const rclcpp::Time& t)
{
    if (_rgbd_publisher && 0 != _rgbd_publisher->get_subscription_count())
    {
        ROS_DEBUG_STREAM("Publishing RGBD message");
        unsigned int rgb_width = rgb_cv_matrix.size().width;
        unsigned int rgb_height = rgb_cv_matrix.size().height;
        unsigned int depth_width = depth_cv_matrix.size().width;
        unsigned int depth_height = depth_cv_matrix.size().height;

        realsense2_camera_msgs::msg::RGBD::UniquePtr msg(new realsense2_camera_msgs::msg::RGBD());

        bool rgb_message_filled = fillROSImageMsgAndReturnStatus(rgb_cv_matrix, COLOR, rgb_width, rgb_height, color_format, t, &msg->rgb);
        if(!rgb_message_filled)
        {
            ROS_ERROR_STREAM("Failed to fill rgb message inside RGBD message");
            return;
        }

        bool depth_messages_filled = fillROSImageMsgAndReturnStatus(depth_cv_matrix, DEPTH, depth_width, depth_height, depth_format, t, &msg->depth);
        if(!depth_messages_filled)
        {
            ROS_ERROR_STREAM("Failed to fill depth message inside RGBD message");
            return;
        }

        msg->header.frame_id = "camera_rgbd_optical_frame";
        msg->header.stamp = t;

        auto rgb_camera_info = _camera_info.at(COLOR);
        msg->rgb_camera_info = rgb_camera_info;

        auto depth_camera_info = _camera_info.at(DEPTH);
        msg->depth_camera_info = depth_camera_info;

        realsense2_camera_msgs::msg::RGBD *msg_address = msg.get();
        _rgbd_publisher->publish(std::move(msg));
        ROS_DEBUG_STREAM("rgbd stream published, message address: " << std::hex << msg_address);
    }
}

void BaseRealSenseNode::publishMetadata(rs2::frame f, const rclcpp::Time& header_time, const std::string& frame_id)
{
    stream_index_pair stream = {f.get_profile().stream_type(), f.get_profile().stream_index()};    
    if (_metadata_publishers.find(stream) != _metadata_publishers.end())
    {
        auto& md_publisher = _metadata_publishers.at(stream);
        if (0 != md_publisher->get_subscription_count())
        {
            realsense2_camera_msgs::msg::Metadata msg;
            msg.header.frame_id = frame_id;
            msg.header.stamp = header_time;
            std::stringstream json_data;
            const char* separator = ",";
            json_data << "{";
            // Add additional fields:
            json_data << "\"" << "frame_number" << "\":" << f.get_frame_number();
            json_data << separator << "\"" << "clock_domain" << "\":" << "\"" << create_graph_resource_name(rs2_timestamp_domain_to_string(f.get_frame_timestamp_domain())) << "\"";
            json_data << separator << "\"" << "frame_timestamp" << "\":" << std::fixed << f.get_timestamp();

            for (auto i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
            {
                if (f.supports_frame_metadata((rs2_frame_metadata_value)i))
                {
                    rs2_frame_metadata_value mparam = (rs2_frame_metadata_value)i;
                    std::string name = create_graph_resource_name(rs2_frame_metadata_to_string(mparam));
                    if (RS2_FRAME_METADATA_FRAME_TIMESTAMP == i)
                    {
                        name = "hw_timestamp";
                    }
                    rs2_metadata_type val = f.get_frame_metadata(mparam);
                    json_data << separator << "\"" << name << "\":" << val;
                }
            }
            json_data << "}";
            msg.json_data = json_data.str();
            md_publisher->publish(msg);
        }
    }
}

void BaseRealSenseNode::startDiagnosticsUpdater()
{
    std::string serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    if (_diagnostics_period > 0)
    {
        ROS_INFO_STREAM("Publish diagnostics every " << _diagnostics_period << " seconds.");
        _diagnostics_updater = std::make_shared<diagnostic_updater::Updater>(&_node, _diagnostics_period);

        _diagnostics_updater->setHardwareID(serial_no);

        _diagnostics_updater->add("Temperatures", [this](diagnostic_updater::DiagnosticStatusWrapper& status)
        {
            bool got_temperature(false);
            for(auto&& sensor : _available_ros_sensors)
            {
                for (rs2_option option : _monitor_options)
                {
                    try
                    {
                        if (sensor->supports(option))
                        {
                            status.add(rs2_option_to_string(option), sensor->get_option(option));
                            got_temperature = true;
                        }
                    }
                    catch(const std::exception& ex)
                    {
                        got_temperature = false;
                        ROS_WARN_STREAM("An error has occurred during monitoring: " << ex.what());
                    }
                }
                if (got_temperature) break;
            }
            status.summary(0, "OK");
        });
    }
}
