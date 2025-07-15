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

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "constants.h"

// cv_bridge.h last supported version is humble
#if defined(CV_BRDIGE_HAS_HPP)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <std_srvs/srv/empty.hpp>
#include "realsense2_camera_msgs/msg/imu_info.hpp"
#include "realsense2_camera_msgs/msg/extrinsics.hpp"
#include "realsense2_camera_msgs/msg/metadata.hpp"
#include "realsense2_camera_msgs/msg/rgbd.hpp"
#include "realsense2_camera_msgs/srv/device_info.hpp"
#include "realsense2_camera_msgs/srv/calib_config_read.hpp"
#include "realsense2_camera_msgs/srv/calib_config_write.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realsense2_camera_msgs/action/triggered_calibration.hpp"
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#if defined(HUMBLE) || defined(IRON) || defined(JAZZY) 
#include <tf2/LinearMath/Quaternion.h>
#else
#include <tf2/LinearMath/Quaternion.hpp>
#endif

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <eigen3/Eigen/Geometry>
#include <condition_variable>

#include <ros_sensor.h>
#include <named_filter.h>

#if defined (ACCELERATE_GPU_WITH_GLSL)
#include <gl_window.h>
#endif

#include <queue>
#include <mutex>
#include <atomic>
#include <thread>

using realsense2_camera_msgs::msg::Extrinsics;
using realsense2_camera_msgs::msg::IMUInfo;
using realsense2_camera_msgs::msg::RGBD;

#define FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << STREAM_NAME(sip) << "_frame")).str()
#define IMU_FRAME_ID (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_imu_frame")).str()
#define IMU_OPTICAL_FRAME_ID (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_imu_optical_frame")).str()
#define OPTICAL_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << STREAM_NAME(sip) << "_optical_frame")).str()
#define ALIGNED_DEPTH_TO_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << "aligned_depth_to_" << STREAM_NAME(sip) << "_frame")).str()

namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    class image_publisher; // forward declaration

    class PipelineSyncer : public rs2::asynchronous_syncer
    {
    public: 
        void operator()(rs2::frame f) const
        {
            invoke(std::move(f));
        }
    };

    class SyncedImuPublisher
    {
        public:
            SyncedImuPublisher(rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher, 
                               std::size_t waiting_list_size=1000);
            ~SyncedImuPublisher();
            void Pause();   // Pause sending messages. All messages from now on are saved in queue.
            void Resume();  // Send all pending messages and allow sending future messages.
            void Publish(sensor_msgs::msg::Imu msg);     //either send or hold message.
            size_t getNumSubscribers();
            void Enable(bool is_enabled) {_is_enabled=is_enabled;};
        
        private:
            void PublishPendingMessages();

        private:
            std::mutex                                          _mutex;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisher;
            bool                                                _pause_mode;
            std::queue<sensor_msgs::msg::Imu>                   _pending_messages;
            std::size_t                                         _waiting_list_size;
            bool                                                _is_enabled;
    };

    class AlignDepthFilter;
    class PointcloudFilter;
    class BaseRealSenseNode
    {
    public:
        BaseRealSenseNode(RosNodeBase& node,
                          rs2::device dev,
                          std::shared_ptr<Parameters> parameters,
                          bool use_intra_process = false);
        ~BaseRealSenseNode();
        void publishTopics();

    public:
        using TriggeredCalibration = realsense2_camera_msgs::action::TriggeredCalibration;
        using GoalHandleTriggeredCalibration = rclcpp_action::ServerGoalHandle<TriggeredCalibration>;
        enum class imu_sync_method{NONE, COPY, LINEAR_INTERPOLATION};

    protected:
        class float3
        {
            public:
                float x, y, z;

            public:
                float3& operator*=(const float& factor)
                {
                    x*=factor;
                    y*=factor;
                    z*=factor;
                    return (*this);
                }
                float3& operator+=(const float3& other)
                {
                    x+=other.x;
                    y+=other.y;
                    z+=other.z;
                    return (*this);
                }
        };

        std::string _base_frame_id;
        bool _is_running;
        RosNodeBase& _node;
        std::string _camera_name;
        std::vector<rs2_option> _monitor_options;
        rclcpp::Logger _logger;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reset_srv;
        rclcpp::Service<realsense2_camera_msgs::srv::DeviceInfo>::SharedPtr _device_info_srv;
        rclcpp::Service<realsense2_camera_msgs::srv::CalibConfigRead>::SharedPtr _calib_config_read_srv;
        rclcpp::Service<realsense2_camera_msgs::srv::CalibConfigWrite>::SharedPtr _calib_config_write_srv;
        rclcpp_action::Server<TriggeredCalibration>::SharedPtr _triggered_calibration_action_server;
        
        std::shared_ptr<Parameters> _parameters;
        std::list<std::string> _parameters_names;

        void restartStaticTransformBroadcaster();
        void publishExtrinsicsTopic(const stream_index_pair& sip, const rs2_extrinsics& ex);
        void calcAndAppendTransformMsgs(const rs2::stream_profile& profile, const rs2::stream_profile& base_profile);

        void handleHWReset(const std_srvs::srv::Empty::Request::SharedPtr req,
                                const std_srvs::srv::Empty::Response::SharedPtr res);
        void getDeviceInfo(const realsense2_camera_msgs::srv::DeviceInfo::Request::SharedPtr req,
                                 realsense2_camera_msgs::srv::DeviceInfo::Response::SharedPtr res);
        void CalibConfigReadService(const realsense2_camera_msgs::srv::CalibConfigRead::Request::SharedPtr req,
                                 realsense2_camera_msgs::srv::CalibConfigRead::Response::SharedPtr res);
        void CalibConfigWriteService(const realsense2_camera_msgs::srv::CalibConfigWrite::Request::SharedPtr req,
                                 realsense2_camera_msgs::srv::CalibConfigWrite::Response::SharedPtr res);

        rclcpp_action::GoalResponse TriggeredCalibrationHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TriggeredCalibration::Goal> goal);
        rclcpp_action::CancelResponse TriggeredCalibrationHandleCancel(const std::shared_ptr<GoalHandleTriggeredCalibration> goal_handle);
        void TriggeredCalibrationHandleAccepted(const std::shared_ptr<GoalHandleTriggeredCalibration> goal_handle);
        void TriggeredCalibrationExecute(const std::shared_ptr<GoalHandleTriggeredCalibration> goal_handle);
  
        tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
        void append_static_tf_msg(const rclcpp::Time& t,
                               const float3& trans,
                               const tf2::Quaternion& q,
                               const std::string& from,
                               const std::string& to);
        void erase_static_tf_msg(const std::string& frame_id,
                                 const std::string& child_frame_id);
        void eraseTransformMsgs(const stream_index_pair& sip, const rs2::stream_profile& profile);
        void setup();

    private:
        class CimuData
        {
            public:
                CimuData() : m_data({0,0,0}), m_time_ns(-1) {};
                CimuData(const stream_index_pair type, Eigen::Vector3d data, double time):
                    m_type(type),
                    m_data(data),
                    m_time_ns(time){};
                bool is_set() {return m_time_ns > 0;};
            public:
                stream_index_pair m_type;
                Eigen::Vector3d m_data;
                double          m_time_ns;
        };

        void getParameters();
        void setDynamicParams();
        void clearParameters();
        void setupDevice();
        void hardwareResetRequest();
        void setupPublishers();
        void enable_devices();
        void setupFilters();
        bool setBaseTime(double frame_time, rs2_timestamp_domain time_domain);
        uint64_t millisecondsToNanoseconds(double timestamp_ms);
        rclcpp::Time frameSystemTimeSec(rs2::frame frame);
        cv::Mat& fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image);
        void clip_depth(rs2::depth_frame depth_frame, float clipping_dist);
        void updateProfilesStreamCalibData(const std::vector<rs2::stream_profile>& profiles);
        void updateExtrinsicsCalibData(const rs2::video_stream_profile& left_video_profile, const rs2::video_stream_profile& right_video_profile);
        void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
        void SetBaseStream();
        void publishStaticTransforms();
        void startDynamicTf();
        void publishDynamicTransforms();
        void publishPointCloud(rs2::points f, const rclcpp::Time& t, const rs2::frameset& frameset);
        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics) const;
        IMUInfo getImuInfo(const rs2::stream_profile& profile);
        void initializeFormatsMaps();

        bool fillROSImageMsgAndReturnStatus(
            const cv::Mat& cv_matrix_image,
            const stream_index_pair& stream,
            unsigned int width,
            unsigned int height,
            const rs2_format& stream_format,
            const rclcpp::Time& t,
            sensor_msgs::msg::Image* img_msg_ptr);

        bool fillCVMatImageAndReturnStatus(
            rs2::frame& frame,
            std::map<stream_index_pair, cv::Mat>& images,
            unsigned int width,
            unsigned int height,
            const stream_index_pair& stream);

        void publishFrame(
            rs2::frame f,
            const rclcpp::Time& t,
            const stream_index_pair& stream,
            std::map<stream_index_pair, cv::Mat>& images,
            const std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>& info_publishers,
            const std::map<stream_index_pair, std::shared_ptr<image_publisher>>& image_publishers,
            const bool is_publishMetadata = true);

        void publishRGBD(
            const cv::Mat& rgb_cv_matrix,
            const rs2_format& color_format,
            const cv::Mat& depth_cv_matrix,
            const rs2_format& depth_format,
            const rclcpp::Time& t);

        void publishMetadata(rs2::frame f, const rclcpp::Time& header_time, const std::string& frame_id);

        sensor_msgs::msg::Imu CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data);

        void FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);
        void ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu& imu_msg);
        void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);
        void imu_callback(rs2::frame frame);
        void imu_callback_sync(rs2::frame frame, imu_sync_method sync_method=imu_sync_method::COPY);
        void multiple_message_callback(rs2::frame frame, imu_sync_method sync_method);
        void frame_callback(rs2::frame frame);
        
        void startDiagnosticsUpdater();
        void monitoringProfileChanges();
        void publish_temperature();
        void setAvailableSensors();
        void setCallbackFunctions();
        void updateSensors();
        void startUpdatedSensors();
        void stopRequiredSensors();
        void publishServices();
        void publishActions();
        void startPublishers(const std::vector<rs2::stream_profile>& profiles, const RosSensor& sensor);
        void startRGBDPublisherIfNeeded();
        void stopPublishers(const std::vector<rs2::stream_profile>& profiles);

#if defined (ACCELERATE_GPU_WITH_GLSL)
        void initOpenGLProcessing(bool use_gpu_processing);
        void shutdownOpenGLProcessing();
        void glfwPollEventCallback();
#endif

        rs2::device _dev;
        std::map<stream_index_pair, rs2::sensor> _sensors;
        std::map<std::string, std::function<void(rs2::frame)>> _sensors_callback;

        std::string _json_file_path;
        float _depth_scale_meters;
        float _clipping_distance;

        double _linear_accel_cov;
        double _angular_velocity_cov;
        bool  _hold_back_imu_for_frames;

        std::map<stream_index_pair, bool> _enable;
        bool _publish_tf;
        double _tf_publish_rate, _diagnostics_period;
        std::mutex _publish_tf_mutex;
        std::mutex _update_sensor_mutex;
        std::mutex _profile_changes_mutex;
        std::mutex _publish_dynamic_tf_mutex;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;
        std::shared_ptr<tf2_ros::TransformBroadcaster> _dynamic_tf_broadcaster;
        std::vector<geometry_msgs::msg::TransformStamped> _static_tf_msgs;
        std::shared_ptr<std::thread> _tf_t;

        bool _use_intra_process;      
        std::map<stream_index_pair, std::shared_ptr<image_publisher>> _image_publishers;
        
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
        std::shared_ptr<SyncedImuPublisher> _synced_imu_publisher;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _info_publishers;
        std::map<stream_index_pair, rclcpp::Publisher<realsense2_camera_msgs::msg::Metadata>::SharedPtr> _metadata_publishers;
        std::map<stream_index_pair, rclcpp::Publisher<IMUInfo>::SharedPtr> _imu_info_publishers;
        std::map<stream_index_pair, rclcpp::Publisher<Extrinsics>::SharedPtr> _extrinsics_publishers;
        rclcpp::Publisher<realsense2_camera_msgs::msg::RGBD>::SharedPtr _rgbd_publisher;
        std::map<stream_index_pair, cv::Mat> _images;
        std::map<rs2_format, std::string> _rs_format_to_ros_format;
        std::map<rs2_format, int> _rs_format_to_cv_format;

        std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> _camera_info;
        std::atomic_bool _is_initialized_time_base;
        double _camera_time_base;

        rclcpp::Time _ros_time_base;
        bool _sync_frames;
        bool _enable_rgbd;
        bool _is_color_enabled;
        bool _is_depth_enabled;
        bool _is_accel_enabled;
        bool _is_gyro_enabled;
        bool _pointcloud;
        imu_sync_method _imu_sync_method;
        stream_index_pair _pointcloud_texture;
        PipelineSyncer _syncer;
        rs2::asynchronous_syncer _asyncer;
        std::shared_ptr<NamedFilter> _colorizer_filter;
        std::shared_ptr<AlignDepthFilter> _align_depth_filter;
        std::shared_ptr<PointcloudFilter> _pc_filter;
        std::vector<std::shared_ptr<NamedFilter>> _filters;
        std::vector<rs2::sensor> _dev_sensors;
        std::vector<std::unique_ptr<RosSensor>> _available_ros_sensors;

        std::map<rs2_stream, std::shared_ptr<rs2::align>> _align;

        std::map<stream_index_pair, cv::Mat> _depth_aligned_image;
        std::map<stream_index_pair, cv::Mat> _depth_scaled_image;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _depth_aligned_info_publisher;
        std::map<stream_index_pair, std::shared_ptr<image_publisher>> _depth_aligned_image_publishers;
        std::map<std::string, rs2::region_of_interest> _auto_exposure_roi;
        std::map<rs2_stream, bool> _is_first_frame;

        std::shared_ptr<std::thread> _monitoring_t;
        std::shared_ptr<std::thread> _monitoring_pc;   //pc = profile changes
        mutable std::condition_variable _cv_temp, _cv_mpc, _cv_tf;
        bool _is_profile_changed;
        bool _is_align_depth_changed;

        std::shared_ptr<diagnostic_updater::Updater> _diagnostics_updater;
        rs2::stream_profile _base_profile;

#if defined (ACCELERATE_GPU_WITH_GLSL)
        GLwindow _app;
        rclcpp::TimerBase::SharedPtr _timer;
        bool _accelerate_gpu_with_glsl;
        bool _is_accelerate_gpu_with_glsl_changed;
#endif

    };//end class
}

