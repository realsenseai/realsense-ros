// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include "realsense2_camera/realsense_node_factory.h"
#include <realsense2_camera/DeviceInfo.h>
#include "realsense2_camera/Metadata.h"
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <condition_variable>

#include <queue>
#include <mutex>
#include <atomic>
#include <thread>

namespace realsense2_camera
{
    struct FrequencyDiagnostics
    {
      FrequencyDiagnostics(double expected_frequency, std::string name, std::string hardware_id) :
        expected_frequency_(expected_frequency),
        frequency_status_(diagnostic_updater::FrequencyStatusParam(&expected_frequency_, &expected_frequency_)),
        diagnostic_updater_(ros::NodeHandle(), ros::NodeHandle("~"), ros::this_node::getName() + "_" + name)
      {
        ROS_INFO("Expected frequency for %s = %.5f", name.c_str(), expected_frequency_);
        diagnostic_updater_.setHardwareID(hardware_id);
        diagnostic_updater_.add(frequency_status_);
      }

      void tick()
      {
        frequency_status_.tick();
      }

      void update()
      {
        diagnostic_updater_.update();
      }

      double expected_frequency_;
      diagnostic_updater::FrequencyStatus frequency_status_;
      diagnostic_updater::Updater diagnostic_updater_;
    };
    typedef std::pair<image_transport::Publisher, std::shared_ptr<FrequencyDiagnostics>> ImagePublisherWithFrequencyDiagnostics;

    class TemperatureDiagnostics
    {
        public:
            TemperatureDiagnostics(std::string name, std::string serial_no);
            void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

            void update(double crnt_temperaure)
            {
                _crnt_temp = crnt_temperaure;
                _updater.update();
            }

        private:
            double _crnt_temp;
            diagnostic_updater::Updater _updater;

    };

    class NamedFilter
    {
        public:
            std::string _name;
            std::shared_ptr<rs2::filter> _filter;

        public:
            NamedFilter(std::string name, std::shared_ptr<rs2::filter> filter):
            _name(name), _filter(filter)
            {}
    };

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
            SyncedImuPublisher(ros::Publisher imu_publisher, std::size_t waiting_list_size=1000);
            ~SyncedImuPublisher();
            void Pause();   // Pause sending messages. All messages from now on are saved in queue.
            void Resume();  // Send all pending messages and allow sending future messages.
            void Publish(sensor_msgs::Imu msg);     //either send or hold message.
            uint32_t getNumSubscribers() { return _publisher.getNumSubscribers();};
            void Enable(bool is_enabled) {_is_enabled=is_enabled;};
        
        private:
            void PublishPendingMessages();

        private:
            std::mutex                    _mutex;
            ros::Publisher                _publisher;
            bool                          _pause_mode;
            std::queue<sensor_msgs::Imu>  _pending_messages;
            std::size_t                     _waiting_list_size;
            bool                          _is_enabled;
    };

    class BaseRealSenseNode : public InterfaceRealSenseNode
    {
    public:
        BaseRealSenseNode(ros::NodeHandle& nodeHandle,
                          ros::NodeHandle& privateNodeHandle,
                          rs2::device dev,
                          const std::string& serial_no);

        virtual void toggleSensors(bool enabled) override;
        virtual void publishTopics() override;
        virtual void registerDynamicReconfigCb(ros::NodeHandle& nh) override;
        virtual ~BaseRealSenseNode();

    public:
        enum imu_sync_method{NONE, COPY, LINEAR_INTERPOLATION};

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

        bool _is_running;
        std::string _base_frame_id;
        std::string _odom_frame_id;
        std::map<stream_index_pair, std::string> _frame_id;
        std::map<stream_index_pair, std::string> _optical_frame_id;
        std::map<stream_index_pair, std::string> _depth_aligned_frame_id;
        ros::NodeHandle& _node_handle, _pnh;
        bool _align_depth;
        std::vector<rs2_option> _monitor_options;
        std::shared_ptr<ros::ServiceServer> _device_info_srv;

        virtual void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile);
        bool getDeviceInfo(realsense2_camera::DeviceInfo::Request& req,
                           realsense2_camera::DeviceInfo::Response& res);
        rs2::stream_profile getAProfile(const stream_index_pair& stream);
        tf::Quaternion rotationMatrixToQuaternion(const float rotation[9]) const;
        void publish_static_tf(const ros::Time& t,
                               const float3& trans,
                               const tf::Quaternion& q,
                               const std::string& from,
                               const std::string& to);


    private:
        class CimuData
        {
            public:
                CimuData() : m_time(-1) {};
                CimuData(const stream_index_pair type, Eigen::Vector3d data, double time):
                    m_type(type),
                    m_data(data),
                    m_time(time){};
                bool is_set() {return m_time > 0;};
            public:
                stream_index_pair m_type;
                Eigen::Vector3d m_data;
                double          m_time;
        };

        static std::string getNamespaceStr();
        void getParameters();
        void setupDevice();
        void setupErrorCallback();
        void setupPublishers();
        void enable_devices();
        void setupFilters();
        void setupStreams();
        bool setBaseTime(double frame_time, rs2_timestamp_domain time_domain);
        double frameSystemTimeSec(rs2::frame frame);
        cv::Mat& fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image);
        void clip_depth(rs2::depth_frame depth_frame, float clipping_dist);
        void updateStreamCalibData(const rs2::video_stream_profile& video_profile);
        void SetBaseStream();
        void publishStaticTransforms();
        void publishDynamicTransforms();
        void publishIntrinsics();
        void runFirstFrameInitialization(rs2_stream stream_type);
        void publishPointCloud(rs2::points f, const ros::Time& t, const rs2::frameset& frameset);
        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const;

        IMUInfo getImuInfo(const stream_index_pair& stream_index);
        void publishFrame(rs2::frame f, const ros::Time& t,
                          const stream_index_pair& stream,
                          std::map<stream_index_pair, cv::Mat>& images,
                          const std::map<stream_index_pair, ros::Publisher>& info_publishers,
                          const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers,
                          const bool is_publishMetadata,
                          std::map<stream_index_pair, int>& seq,
                          std::map<stream_index_pair, sensor_msgs::CameraInfo>& camera_info,
                          const std::map<rs2_stream, std::string>& encoding,
                          bool copy_data_from_frame = true);
        void publishMetadata(rs2::frame f, const ros::Time& header_time, const std::string& frame_id);
        bool getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile);

        void publishAlignedDepthToOthers(rs2::frameset frames, const ros::Time& t);
        sensor_msgs::Imu CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data);

        void FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs);
        void ImuMessage_AddDefaultValues(sensor_msgs::Imu& imu_msg);
        void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs);
        void imu_callback(rs2::frame frame);
        void imu_callback_sync(rs2::frame frame, imu_sync_method sync_method=imu_sync_method::COPY);
        void pose_callback(rs2::frame frame);
        void multiple_message_callback(rs2::frame frame, imu_sync_method sync_method);
        void frame_callback(rs2::frame frame);
        void registerDynamicOption(ros::NodeHandle& nh, rs2::options sensor, std::string& module_name);
        void registerHDRoptions();
        void set_sensor_parameter_to_ros(const std::string& module_name, rs2::options sensor, rs2_option option);
        void monitor_update_functions();
        void readAndSetDynamicParam(ros::NodeHandle& nh1, std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec, const std::string option_name, const int min_val, const int max_val, rs2::sensor sensor, int* option_value);
        void registerAutoExposureROIOptions(ros::NodeHandle& nh);
        void set_auto_exposure_roi(const std::string option_name, rs2::sensor sensor, int new_value);
        void set_sensor_auto_exposure_roi(rs2::sensor sensor);
        rs2_stream rs2_string_to_stream(std::string str);
        void startMonitoring();
        void publish_temperature();
        void publish_frequency_update();
        void publishServices();

        rs2::device _dev;
        std::map<stream_index_pair, rs2::sensor> _sensors;
        std::map<std::string, std::function<void(rs2::frame)>> _sensors_callback;
        std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>> _ddynrec;

        std::string _json_file_path;
        std::string _serial_no;
        float _depth_scale_meters;
        float _clipping_distance;
        bool _allow_no_texture_points;
        bool _ordered_pc;


        double _linear_accel_cov;
        double _angular_velocity_cov;
        bool  _hold_back_imu_for_frames;

        std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
        std::map<stream_index_pair, int> _width;
        std::map<stream_index_pair, int> _height;
        std::map<stream_index_pair, int> _fps;
        std::map<rs2_stream, int>        _format;
        std::map<stream_index_pair, bool> _enable;
        std::map<rs2_stream, std::string> _stream_name;
        bool _publish_tf;
        double _tf_publish_rate;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;
        tf2_ros::TransformBroadcaster _dynamic_tf_broadcaster;
        std::vector<geometry_msgs::TransformStamped> _static_tf_msgs;
        std::shared_ptr<std::thread> _tf_t, _update_functions_t;

        std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _image_publishers;
        std::map<stream_index_pair, ros::Publisher> _imu_publishers;
        std::shared_ptr<SyncedImuPublisher> _synced_imu_publisher;
        std::map<rs2_stream, int> _image_format;
        std::map<stream_index_pair, ros::Publisher> _info_publisher;
        std::map<stream_index_pair, std::shared_ptr<ros::Publisher>> _metadata_publishers;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<rs2_stream, std::string> _encoding;

        std::map<stream_index_pair, int> _seq;
        std::map<rs2_stream, int> _unit_step_size;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _camera_info;
        std::atomic_bool _is_initialized_time_base;
        double _camera_time_base;
        double _previous_frame_time;
        std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

        ros::Publisher _pointcloud_publisher;
        ros::Time _ros_time_base;
        bool _sync_frames;
        bool _pointcloud;
        bool _publish_odom_tf;
        imu_sync_method _imu_sync_method;
        std::string _filters_str;
        stream_index_pair _pointcloud_texture;
        PipelineSyncer _syncer;
        std::vector<NamedFilter> _filters;
        std::shared_ptr<rs2::filter> _colorizer, _pointcloud_filter;
        std::vector<rs2::sensor> _dev_sensors;

        std::map<stream_index_pair, cv::Mat> _depth_aligned_image;
        std::map<stream_index_pair, cv::Mat> _depth_scaled_image;
        std::map<rs2_stream, std::string> _depth_aligned_encoding;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _depth_aligned_camera_info;
        std::map<stream_index_pair, int> _depth_aligned_seq;
        std::map<stream_index_pair, ros::Publisher> _depth_aligned_info_publisher;
        std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics> _depth_aligned_image_publishers;
        std::map<stream_index_pair, ros::Publisher> _depth_to_other_extrinsics_publishers;
        std::map<stream_index_pair, rs2_extrinsics> _depth_to_other_extrinsics;
        std::map<std::string, rs2::region_of_interest> _auto_exposure_roi;
        std::map<rs2_stream, bool> _is_first_frame;
        std::map<rs2_stream, std::vector<std::function<void()> > > _video_functions_stack;

        typedef std::pair<rs2_option, std::shared_ptr<TemperatureDiagnostics>> OptionTemperatureDiag;
        std::vector< OptionTemperatureDiag > _temperature_nodes;
        std::shared_ptr<std::thread> _monitoring_t;
        std::vector<std::function<void()> > _update_functions_v;
        mutable std::condition_variable _cv_monitoring, _cv_tf, _update_functions_cv;

        stream_index_pair _base_stream;
        const std::string _namespace;

        sensor_msgs::PointCloud2 _msg_pointcloud;
        std::vector< unsigned int > _valid_pc_indices;
    };//end class

}

