// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "constants.h"
#include <cv_bridge/cv_bridge.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include "realsense2_camera_msgs/msg/imu_info.hpp"
#include "realsense2_camera_msgs/msg/extrinsics.hpp"
#include "realsense2_camera_msgs/msg/metadata.hpp"
#include "realsense2_camera_msgs/srv/device_info.hpp"
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <eigen3/Eigen/Geometry>
#include <condition_variable>

#include <ros_sensor.h>
#include <named_filter.h>

#include <queue>
#include <mutex>
#include <atomic>
#include <thread>

using realsense2_camera_msgs::msg::Extrinsics;
using realsense2_camera_msgs::msg::IMUInfo;

#define FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << STREAM_NAME(sip) << "_frame")).str()
#define OPTICAL_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << STREAM_NAME(sip) << "_optical_frame")).str()
#define ALIGNED_DEPTH_TO_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << "aligned_depth_to_" << STREAM_NAME(sip) << "_frame")).str()

namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA0, INFRA1, INFRA2,
                                                          COLOR,
                                                          FISHEYE,
                                                          FISHEYE1, FISHEYE2};

    const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL, POSE};
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
            SyncedImuPublisher() {_is_enabled=false;};
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

    class BaseRealSenseNode
    {
    public:
        BaseRealSenseNode(rclcpp::Node& node,
                          rs2::device dev,
                          std::shared_ptr<Parameters> parameters,
                          bool use_intra_process = false);
        ~BaseRealSenseNode();
        void publishTopics();

    public:
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
        rclcpp::Node& _node;
        std::string _camera_name;
        std::vector<rs2_option> _monitor_options;
        rclcpp::Logger _logger;
        rclcpp::Service<realsense2_camera_msgs::srv::DeviceInfo>::SharedPtr _device_info_srv;
        std::shared_ptr<Parameters> _parameters;
        std::list<std::string> _parameters_names;

        void publishExtrinsicsTopic(const stream_index_pair& sip, const rs2_extrinsics& ex);
        virtual void calcAndPublishStaticTransform(const rs2::stream_profile& profile, const rs2::stream_profile& base_profile);
        void getDeviceInfo(const realsense2_camera_msgs::srv::DeviceInfo::Request::SharedPtr req,
                                 realsense2_camera_msgs::srv::DeviceInfo::Response::SharedPtr res);
        tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
        void publish_static_tf(const rclcpp::Time& t,
                               const float3& trans,
                               const tf2::Quaternion& q,
                               const std::string& from,
                               const std::string& to);
        void setup();

    private:
        class CimuData
        {
            public:
                CimuData() : m_time_ns(-1) {};
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
        rclcpp::Time frameSystemTimeSec(rs2::frame frame);
        cv::Mat& fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image);
        void clip_depth(rs2::depth_frame depth_frame, float clipping_dist);
        void updateProfilesStreamCalibData(const std::vector<rs2::stream_profile>& profiles);
        void updateExtrinsicsCalibData(const rs2::video_stream_profile& left_video_profile, const rs2::video_stream_profile& right_video_profile);
        void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
        void SetBaseStream();
        void publishStaticTransforms(std::vector<rs2::stream_profile> profiles);
        void startDynamicTf();
        void publishDynamicTransforms();
        void publishPointCloud(rs2::points f, const rclcpp::Time& t, const rs2::frameset& frameset);
        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics) const;

        IMUInfo getImuInfo(const rs2::stream_profile& profile);
        
        void publishFrame(rs2::frame f, const rclcpp::Time& t,
                          const stream_index_pair& stream,
                          std::map<stream_index_pair, cv::Mat>& images,
                          const std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>& info_publishers,
                          const std::map<stream_index_pair, std::shared_ptr<image_publisher>>& image_publishers,
                          const bool is_publishMetadata = true);
        void publishMetadata(rs2::frame f, const rclcpp::Time& header_time, const std::string& frame_id);

        sensor_msgs::msg::Imu CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data);

        void FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);
        void ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu& imu_msg);
        void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs);
        void imu_callback(rs2::frame frame);
        void imu_callback_sync(rs2::frame frame, imu_sync_method sync_method=imu_sync_method::COPY);
        void pose_callback(rs2::frame frame);
        void multiple_message_callback(rs2::frame frame, imu_sync_method sync_method);
        void frame_callback(rs2::frame frame);
        
        void startDiagnosticsUpdater();
        void monitoringProfileChanges();
        void publish_temperature();
        void setupFiltersPublishers();
        void setAvailableSensors();
        void setCallbackFunctions();
        void updateSensors();
        void publishServices();
        void startPublishers(const std::vector<rs2::stream_profile>& profiles, const RosSensor& sensor);
        void stopPublishers(const std::vector<rs2::stream_profile>& profiles);

        rs2::device _dev;
        std::map<stream_index_pair, rs2::sensor> _sensors;
        std::map<std::string, std::function<void(rs2::frame)>> _sensors_callback;

        std::string _json_file_path;
        float _depth_scale_meters;
        float _clipping_distance;

        double _linear_accel_cov;
        double _angular_velocity_cov;
        bool  _hold_back_imu_for_frames;

        std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
        std::map<rs2_stream, rs2_format>  _format;
        std::map<stream_index_pair, bool> _enable;
        bool _publish_tf;
        double _tf_publish_rate, _diagnostics_period;
        std::mutex _publish_tf_mutex;
        std::mutex _update_sensor_mutex;
        std::mutex _profile_changes_mutex;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;
        std::shared_ptr<tf2_ros::TransformBroadcaster> _dynamic_tf_broadcaster;
        std::vector<geometry_msgs::msg::TransformStamped> _static_tf_msgs;
        std::shared_ptr<std::thread> _tf_t;

        bool _use_intra_process;      
        std::map<stream_index_pair, std::shared_ptr<image_publisher>> _image_publishers;
        
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> _odom_publisher;
        std::shared_ptr<SyncedImuPublisher> _synced_imu_publisher;
        std::map<unsigned int, int> _image_format;
        std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _info_publisher;
        std::map<stream_index_pair, rclcpp::Publisher<realsense2_camera_msgs::msg::Metadata>::SharedPtr> _metadata_publishers;
        std::map<stream_index_pair, rclcpp::Publisher<IMUInfo>::SharedPtr> _imu_info_publisher;
        std::map<stream_index_pair, rclcpp::Publisher<Extrinsics>::SharedPtr> _extrinsics_publishers;
        std::map<stream_index_pair, Extrinsics> _extrinsics_msgs;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<unsigned int, std::string> _encoding;

        std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> _camera_info;
        std::atomic_bool _is_initialized_time_base;
        double _camera_time_base;

        rclcpp::Time _ros_time_base;
        bool _sync_frames;
        bool _pointcloud;
        bool _publish_odom_tf;
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


    };//end class
}

