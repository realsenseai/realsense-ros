#include "realsense2_camera/base_realsense_node.h"
#include "assert.h"
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <cctype>
#include <mutex>

#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

using namespace realsense2_camera;
using namespace ddynamic_reconfigure;

// stream_index_pair sip{stream_type, stream_index};
#define STREAM_NAME(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << _stream_name[sip.first] << ((sip.second>0) ? std::to_string(sip.second) : ""))).str()
#define FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_" << STREAM_NAME(sip) << "_frame")).str()
#define OPTICAL_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_" << STREAM_NAME(sip) << "_optical_frame")).str()
#define ALIGNED_DEPTH_TO_FRAME_ID(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << "camera_aligned_depth_to_" << STREAM_NAME(sip) << "_frame")).str()

SyncedImuPublisher::SyncedImuPublisher(ros::Publisher imu_publisher, std::size_t waiting_list_size):
            _publisher(imu_publisher), _pause_mode(false),
            _waiting_list_size(waiting_list_size)
            {}

SyncedImuPublisher::~SyncedImuPublisher()
{
    PublishPendingMessages();
}

void SyncedImuPublisher::Publish(sensor_msgs::Imu imu_msg)
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
        _publisher.publish(imu_msg);
        // ROS_INFO_STREAM("iid1:" << imu_msg.header.seq << ", time: " << std::setprecision (20) << imu_msg.header.stamp.toSec());
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
    // ROS_INFO_STREAM("publish imu: " << _pending_messages.size());
    while (!_pending_messages.empty())
    {
        const sensor_msgs::Imu &imu_msg = _pending_messages.front();
        _publisher.publish(imu_msg);
        // ROS_INFO_STREAM("iid2:" << imu_msg.header.seq << ", time: " << std::setprecision (20) << imu_msg.header.stamp.toSec());
        _pending_messages.pop();
    }
}

std::string BaseRealSenseNode::getNamespaceStr()
{
    auto ns = ros::this_node::getNamespace();
    ns.erase(std::remove(ns.begin(), ns.end(), '/'), ns.end());
    return ns;
}

BaseRealSenseNode::BaseRealSenseNode(ros::NodeHandle& nodeHandle,
                                     ros::NodeHandle& privateNodeHandle,
                                     rs2::device dev,
                                     const std::string& serial_no) :
    _is_running(true), _base_frame_id(""),  _node_handle(nodeHandle),
    _pnh(privateNodeHandle), _dev(dev), _json_file_path(""),
    _serial_no(serial_no),
    _is_initialized_time_base(false),
    _namespace(getNamespaceStr())
{
    // Types for depth stream
    _format[RS2_STREAM_DEPTH] = RS2_FORMAT_Z16;
    _image_format[RS2_STREAM_DEPTH] = CV_16UC1;    // CVBridge type
    _encoding[RS2_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    _unit_step_size[RS2_STREAM_DEPTH] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
    _stream_name[RS2_STREAM_DEPTH] = "depth";

    // Types for confidence stream
    _image_format[RS2_STREAM_CONFIDENCE] = CV_8UC1;    // CVBridge type
    _encoding[RS2_STREAM_CONFIDENCE] = sensor_msgs::image_encodings::MONO8; // ROS message type
    _unit_step_size[RS2_STREAM_CONFIDENCE] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[RS2_STREAM_CONFIDENCE] = "confidence";

    // Infrared stream
    _format[RS2_STREAM_INFRARED] = RS2_FORMAT_Y8;

    _image_format[RS2_STREAM_INFRARED] = CV_8UC1;    // CVBridge type
    _encoding[RS2_STREAM_INFRARED] = sensor_msgs::image_encodings::MONO8; // ROS message type
    _unit_step_size[RS2_STREAM_INFRARED] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[RS2_STREAM_INFRARED] = "infra";

    // Types for color stream
    _image_format[RS2_STREAM_COLOR] = CV_8UC3;    // CVBridge type
    _encoding[RS2_STREAM_COLOR] = sensor_msgs::image_encodings::RGB8; // ROS message type
    _unit_step_size[RS2_STREAM_COLOR] = 3; // sensor_msgs::ImagePtr row step size
    _stream_name[RS2_STREAM_COLOR] = "color";
    _depth_aligned_encoding[RS2_STREAM_COLOR] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for fisheye stream
    _image_format[RS2_STREAM_FISHEYE] = CV_8UC1;    // CVBridge type
    _encoding[RS2_STREAM_FISHEYE] = sensor_msgs::image_encodings::MONO8; // ROS message type
    _unit_step_size[RS2_STREAM_FISHEYE] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[RS2_STREAM_FISHEYE] = "fisheye";

    // Types for Motion-Module streams
    _stream_name[RS2_STREAM_GYRO] = "gyro";

    _stream_name[RS2_STREAM_ACCEL] = "accel";

    _stream_name[RS2_STREAM_POSE] = "pose";

    _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_PROJECTOR_TEMPERATURE};
}

BaseRealSenseNode::~BaseRealSenseNode()
{
    // Kill dynamic transform thread
    _is_running = false;
    _cv_tf.notify_one();
    if (_tf_t && _tf_t->joinable())
        _tf_t->join();
    if (_update_functions_t && _update_functions_t->joinable())
        _update_functions_t->join();

    _cv_monitoring.notify_one();
    if (_monitoring_t && _monitoring_t->joinable())
    {
        _monitoring_t->join();
    }

    std::set<std::string> module_names;
    for (const std::pair<stream_index_pair, std::vector<rs2::stream_profile>>& profile : _enabled_profiles)
    {
        try
        {
            std::string module_name = _sensors[profile.first].get_info(RS2_CAMERA_INFO_NAME);
            std::pair< std::set<std::string>::iterator, bool> res = module_names.insert(module_name);
            if (res.second)
            {
                _sensors[profile.first].stop();
                _sensors[profile.first].close();
            }
        }
        catch (const rs2::error& e)
        {
            ROS_ERROR_STREAM("Exception: " << e.what());
        }
    }
}

void BaseRealSenseNode::toggleSensors(bool enabled)
{
  if(enabled)
  {
    std::map<std::string, std::vector<rs2::stream_profile> > profiles;
    std::map<std::string, rs2::sensor> active_sensors;
    for (const std::pair<stream_index_pair, std::vector<rs2::stream_profile>>& profile : _enabled_profiles)
    {
        std::string module_name = _sensors[profile.first].get_info(RS2_CAMERA_INFO_NAME);
        ROS_INFO_STREAM("insert " << rs2_stream_to_string(profile.second.begin()->stream_type())
          << " to " << module_name);
        profiles[module_name].insert(profiles[module_name].begin(),
                                        profile.second.begin(),
                                        profile.second.end());
        active_sensors[module_name] = _sensors[profile.first];
    }

    for (const std::pair<std::string, std::vector<rs2::stream_profile> >& sensor_profile : profiles)
    {
        std::string module_name = sensor_profile.first;
        rs2::sensor sensor = active_sensors[module_name];
        sensor.open(sensor_profile.second);
        sensor.start(_sensors_callback[module_name]);
        if (sensor.is<rs2::depth_sensor>())
        {
            _depth_scale_meters = sensor.as<rs2::depth_sensor>().get_depth_scale();
        }
    }
  }
  else
  {
    std::set<std::string> module_names;
    for (const std::pair<stream_index_pair, std::vector<rs2::stream_profile>>& profile : _enabled_profiles)
    {
        std::string module_name = _sensors[profile.first].get_info(RS2_CAMERA_INFO_NAME);
        std::pair< std::set<std::string>::iterator, bool> res = module_names.insert(module_name);
        if (res.second)
        {
            _sensors[profile.first].stop();
            _sensors[profile.first].close();
        }
    }
  }
}

void BaseRealSenseNode::setupErrorCallback()
{
    for (auto&& s : _dev.query_sensors())
    {
        s.set_notifications_callback([&](const rs2::notification& n)
        {
            std::vector<std::string> error_strings({"RT IC2 Config error",
                                                    "Left IC2 Config error"});
            if (n.get_severity() >= RS2_LOG_SEVERITY_ERROR)
            {
                ROS_WARN_STREAM("Hardware Notification:" << n.get_description() << "," << n.get_timestamp() << "," << n.get_severity() << "," << n.get_category());
            }
            if (error_strings.end() != std::find_if(error_strings.begin(), error_strings.end(), [&n] (std::string err) 
                                        {return (n.get_description().find(err) != std::string::npos); }))
            {
                ROS_ERROR_STREAM("Performing Hardware Reset.");
                _dev.hardware_reset();
            }
        });
    }
}

void BaseRealSenseNode::publishTopics()
{
    getParameters();
    setupDevice();
    setupFilters();
    registerHDRoptions();
    registerDynamicReconfigCb(_node_handle);
    setupErrorCallback();
    enable_devices();
    setupPublishers();
    setupStreams();
    SetBaseStream();
    registerAutoExposureROIOptions(_node_handle);
    publishStaticTransforms();
    publishIntrinsics();
    startMonitoring();
    publishServices();
    ROS_INFO_STREAM("RealSense Node Is Up!");
}

void BaseRealSenseNode::runFirstFrameInitialization(rs2_stream stream_type)
{
    if (_is_first_frame[stream_type])
    {
        ROS_DEBUG_STREAM("runFirstFrameInitialization: " << _video_functions_stack.size() << ", " << rs2_stream_to_string(stream_type));
        _is_first_frame[stream_type] = false;
        if (!_video_functions_stack[stream_type].empty())
        {
            std::thread t = std::thread([=]()
            {
                while (!_video_functions_stack[stream_type].empty())
                {
                    _video_functions_stack[stream_type].back()();
                    _video_functions_stack[stream_type].pop_back();
                }
            });
            t.detach();
        }
    }
}

bool is_checkbox(rs2::options sensor, rs2_option option)
{
    rs2::option_range op_range = sensor.get_option_range(option);
    return op_range.max == 1.0f &&
        op_range.min == 0.0f &&
        op_range.step == 1.0f;
}

bool is_enum_option(rs2::options sensor, rs2_option option)
{
    static const int MAX_ENUM_OPTION_VALUES(100);
    static const float EPSILON(0.05);
    
    rs2::option_range op_range = sensor.get_option_range(option);
    if (abs((op_range.step - 1)) > EPSILON || (op_range.max > MAX_ENUM_OPTION_VALUES)) return false;
    for (auto i = op_range.min; i <= op_range.max; i += op_range.step)
    {
        if (sensor.get_option_value_description(option, i) == nullptr)
            continue;
        return true;
    }
    return false;
}

bool is_int_option(rs2::options sensor, rs2_option option)
{
    rs2::option_range op_range = sensor.get_option_range(option);
    return (op_range.step == 1.0);
}

std::map<std::string, int> get_enum_method(rs2::options sensor, rs2_option option)
{
    std::map<std::string, int> dict; // An enum to set size
    if (is_enum_option(sensor, option))
    {
        rs2::option_range op_range = sensor.get_option_range(option);
        const auto op_range_min = int(op_range.min);
        const auto op_range_max = int(op_range.max);
        const auto op_range_step = int(op_range.step);
        for (auto val = op_range_min; val <= op_range_max; val += op_range_step)
        {
            if (sensor.get_option_value_description(option, val) == nullptr)
                continue;
            dict[sensor.get_option_value_description(option, val)] = val;
        }
    }
    return dict;
}

namespace realsense2_camera
{

template <typename K, typename V>
std::ostream& operator<<(std::ostream& os, const std::map<K, V>& m)
{
    os << '{';
    for (const auto& kv : m)
    {
        os << " {" << kv.first << ": " << kv.second << '}';
    }
    os << " }";
    return os;
}

}

/**
 * Same as ros::names::isValidCharInName, but re-implemented here because it's not exposed.
 */
bool isValidCharInName(char c)
{
    return std::isalnum(c) || c == '/' || c == '_';
}

/**
 * ROS Graph Resource names don't allow spaces and hyphens (see http://wiki.ros.org/Names),
 * so we replace them here with underscores.
 */
std::string create_graph_resource_name(const std::string &original_name)
{
  std::string fixed_name = original_name;
  std::transform(fixed_name.begin(), fixed_name.end(), fixed_name.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  std::replace_if(fixed_name.begin(), fixed_name.end(), [](const char c) { return !isValidCharInName(c); },
                  '_');
  return fixed_name;
}

void BaseRealSenseNode::set_auto_exposure_roi(const std::string option_name, rs2::sensor sensor, int new_value)
{
    rs2::region_of_interest& auto_exposure_roi(_auto_exposure_roi[sensor.get_info(RS2_CAMERA_INFO_NAME)]);
    if (option_name == "left")
        auto_exposure_roi.min_x = new_value;
    else if (option_name == "right")
        auto_exposure_roi.max_x = new_value;
    else if (option_name == "top")
        auto_exposure_roi.min_y = new_value;
    else if (option_name == "bottom")
        auto_exposure_roi.max_y = new_value;
    else
    {
        ROS_WARN_STREAM("Invalid option_name: " << option_name << " while setting auto exposure ROI.");
        return;
    }
    set_sensor_auto_exposure_roi(sensor);
}

void BaseRealSenseNode::set_sensor_auto_exposure_roi(rs2::sensor sensor)
{
    const rs2::region_of_interest& auto_exposure_roi(_auto_exposure_roi[sensor.get_info(RS2_CAMERA_INFO_NAME)]);
    try
    {
        sensor.as<rs2::roi_sensor>().set_region_of_interest(auto_exposure_roi);
    }
    catch(const std::runtime_error& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}

void BaseRealSenseNode::readAndSetDynamicParam(ros::NodeHandle& nh1, std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec, 
                                               const std::string option_name, const int min_val, const int max_val, rs2::sensor sensor, 
                                               int* option_value)
{
    nh1.param(option_name, *option_value, *option_value); //param (const std::string &param_name, T &param_val, const T &default_val) const
    if (*option_value < min_val) *option_value = min_val;
    if (*option_value > max_val) *option_value = max_val;
    
    ddynrec->registerVariable<int>(
        option_name, *option_value, [this, sensor, option_name](int new_value){set_auto_exposure_roi(option_name, sensor, new_value);},
        "auto-exposure " + option_name + " coordinate", min_val, max_val);
}

void BaseRealSenseNode::registerAutoExposureROIOptions(ros::NodeHandle& nh)
{
    for (const std::pair<stream_index_pair, std::vector<rs2::stream_profile>>& profile : _enabled_profiles)
    {
        rs2::sensor sensor = _sensors[profile.first];
        std::string module_base_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
        if (sensor.is<rs2::roi_sensor>() && _auto_exposure_roi.find(module_base_name) == _auto_exposure_roi.end())
        {
            int max_x(_width[profile.first]-1);
            int max_y(_height[profile.first]-1);

            std::string module_name = create_graph_resource_name(module_base_name) +"/auto_exposure_roi";
            ros::NodeHandle nh1(nh, module_name);
            std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh1);

            _auto_exposure_roi[module_base_name] = {0, 0, max_x, max_y};
            rs2::region_of_interest& auto_exposure_roi(_auto_exposure_roi[module_base_name]);
            readAndSetDynamicParam(nh1, ddynrec, "left", 0, max_x, sensor, &(auto_exposure_roi.min_x));
            readAndSetDynamicParam(nh1, ddynrec, "right", 0, max_x, sensor, &(auto_exposure_roi.max_x));
            readAndSetDynamicParam(nh1, ddynrec, "top", 0, max_y, sensor, &(auto_exposure_roi.min_y));
            readAndSetDynamicParam(nh1, ddynrec, "bottom", 0, max_y, sensor, &(auto_exposure_roi.max_y));

            ddynrec->publishServicesTopics();
            _ddynrec.push_back(ddynrec);

            // Initiate the call to set_sensor_auto_exposure_roi, after the first frame arrive.
            rs2_stream stream_type = profile.first.first;
            _video_functions_stack[stream_type].push_back([this, sensor](){set_sensor_auto_exposure_roi(sensor);});
            _is_first_frame[stream_type] = true;
        }
    }
}

void BaseRealSenseNode::registerDynamicOption(ros::NodeHandle& nh, rs2::options sensor, std::string& module_name)
{
    ros::NodeHandle nh1(nh, module_name);
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh1);
    for (auto i = 0; i < RS2_OPTION_COUNT; i++)
    {
        rs2_option option = static_cast<rs2_option>(i);
        const std::string option_name(create_graph_resource_name(rs2_option_to_string(option)));
        try
        {
            if (!sensor.supports(option) || sensor.is_option_read_only(option))
            {
                continue;
            }
            if (is_checkbox(sensor, option))
            {
                auto option_value = bool(sensor.get_option(option));
                if (nh1.param(option_name, option_value, option_value))
                {
                    sensor.set_option(option, option_value);
                }
                ddynrec->registerVariable<bool>(
                option_name, option_value,
                [option, sensor](bool new_value) { sensor.set_option(option, new_value); },
                sensor.get_option_description(option));
                continue;
            }
            const auto enum_dict = get_enum_method(sensor, option);
            if (enum_dict.empty())
            {
                rs2::option_range op_range = sensor.get_option_range(option);
                const auto sensor_option_value = sensor.get_option(option);
                auto option_value = sensor_option_value;
                if (nh1.param(option_name, option_value, option_value))
                {
                    if (option_value < op_range.min || op_range.max < option_value)
                    {
                        ROS_WARN_STREAM("Param '" << nh1.resolveName(option_name) << "' has value " << option_value
                                << " outside the range [" << op_range.min << ", " << op_range.max
                                << "]. Using current sensor value " << sensor_option_value << " instead.");
                        option_value = sensor_option_value;
                    }
                    else
                    {
                        sensor.set_option(option, option_value);
                    }
                }
                if (option_value < op_range.min || op_range.max < option_value)
                {
                    ROS_WARN_STREAM("Param '" << nh1.resolveName(option_name) << "' has value " << option_value
                                            << " that is not in range [" << op_range.min << ", " << op_range.max << "]"
                                            << ". Removing this parameter from dynamic reconfigure options.");
                    continue;
                }
                if (is_int_option(sensor, option))
                {
                ddynrec->registerVariable<int>(
                    option_name, int(option_value),
                    [option, sensor](int new_value) { sensor.set_option(option, new_value); },
                    sensor.get_option_description(option), int(op_range.min), int(op_range.max));
                }
                else
                {
                    if (i == RS2_OPTION_DEPTH_UNITS)
                    {
                        if (ROS_DEPTH_SCALE >= op_range.min && ROS_DEPTH_SCALE <= op_range.max)
                        {
                            sensor.set_option(option, ROS_DEPTH_SCALE);
                            op_range.min = ROS_DEPTH_SCALE;
                            op_range.max = ROS_DEPTH_SCALE;

                            _depth_scale_meters = ROS_DEPTH_SCALE;
                        }
                    }
                    else
                    {
                    ddynrec->registerVariable<double>(
                        option_name, option_value,
                        [option, sensor](double new_value) { sensor.set_option(option, new_value); },
                        sensor.get_option_description(option), double(op_range.min), double(op_range.max));
                    }
                }
            }
            else
            {
                const auto sensor_option_value = sensor.get_option(option);
                auto option_value = int(sensor_option_value);
                if (nh1.param(option_name, option_value, option_value))
                {
                    if (std::find_if(enum_dict.cbegin(), enum_dict.cend(),
                                    [&option_value](const std::pair<std::string, int>& kv) {
                                        return kv.second == option_value;
                                    }) == enum_dict.cend())
                    {
                        ROS_WARN_STREAM("Param '" << nh1.resolveName(option_name) << "' has value " << option_value
                                                << " that is not in the enum " << enum_dict
                                                << ". Using current sensor value " << sensor_option_value << " instead.");
                        option_value = sensor_option_value;
                    }
                    else
                    {
                        sensor.set_option(option, option_value);
                    }
                }
                if (std::find_if(enum_dict.cbegin(), enum_dict.cend(),
                                    [&option_value](const std::pair<std::string, int>& kv) {
                                        return kv.second == option_value;
                                    }) == enum_dict.cend())
                {
                    ROS_WARN_STREAM("Param '" << nh1.resolveName(option_name) << "' has value " << option_value
                                            << " that is not in the enum " << enum_dict
                                            << ". Removing this parameter from dynamic reconfigure options.");
                    continue;
                }
                if (module_name == "stereo_module" && i == RS2_OPTION_SEQUENCE_ID)
                {
                    ddynrec->registerEnumVariable<int>(
                        option_name, option_value,
                        [this, option, sensor, module_name](int new_value) 
                            { 
                                sensor.set_option(option, new_value); 
                                _update_functions_v.push_back([this, module_name, sensor]()
                                    {set_sensor_parameter_to_ros(module_name, sensor, RS2_OPTION_EXPOSURE);});
                                _update_functions_v.push_back([this, module_name, sensor]()
                                    {set_sensor_parameter_to_ros(module_name, sensor, RS2_OPTION_GAIN);});
                                _update_functions_cv.notify_one();
                            },
                        sensor.get_option_description(option), enum_dict);
                }
                else
                {
                    ddynrec->registerEnumVariable<int>(
                        option_name, option_value,
                        [option, sensor](int new_value) { sensor.set_option(option, new_value); },
                        sensor.get_option_description(option), enum_dict);
                }
            }
        }
        catch(const rs2::backend_error& e)
        {
            ROS_WARN_STREAM("Failed to set option: " << option_name << ": " << e.what());
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
    }
    ddynrec->publishServicesTopics();
    _ddynrec.push_back(ddynrec);
}

void BaseRealSenseNode::registerDynamicReconfigCb(ros::NodeHandle& nh)
{
    ROS_INFO("Setting Dynamic reconfig parameters.");

    for(rs2::sensor sensor : _dev_sensors)
    {
        std::string module_name = create_graph_resource_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
        ROS_DEBUG_STREAM("module_name:" << module_name);
        registerDynamicOption(nh, sensor, module_name);
    }

    for (NamedFilter nfilter : _filters)
    {
        std::string module_name = nfilter._name;
        auto sensor = *(nfilter._filter);
        ROS_DEBUG_STREAM("module_name:" << module_name);
        registerDynamicOption(nh, sensor, module_name);
    }
    ROS_INFO("Done Setting Dynamic reconfig parameters.");
}

void BaseRealSenseNode::registerHDRoptions()
{
    if (std::find_if(std::begin(_filters), std::end(_filters), [](NamedFilter f){return f._name == "hdr_merge";}) == std::end(_filters))
        return;

    std::string module_name;
    std::vector<rs2_option> options{RS2_OPTION_EXPOSURE, RS2_OPTION_GAIN};
    for(rs2::sensor sensor : _dev_sensors)
    {
        if (!sensor.is<rs2::depth_sensor>()) continue;
        std::string module_name = create_graph_resource_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
        // Read initialization parameters and initialize hdr_merge filter:
        unsigned int seq_size = sensor.get_option(RS2_OPTION_SEQUENCE_SIZE);
        for (unsigned int seq_id = 1; seq_id <= seq_size; seq_id++ )
        {
            sensor.set_option(RS2_OPTION_SEQUENCE_ID, seq_id);
            for (rs2_option& option : options)
            {
                std::stringstream param_name_str;
                param_name_str << module_name << "/" << create_graph_resource_name(rs2_option_to_string(option)) << "/" << seq_id;
                std::string param_name(param_name_str.str());
                ROS_INFO_STREAM("Reading option: " << param_name);
                int option_value = sensor.get_option(option);

                int user_set_option_value;
                _pnh.param(param_name, user_set_option_value, option_value);
                _pnh.deleteParam(param_name);
                if (option_value != user_set_option_value)
                {
                    ROS_INFO_STREAM("Set " << rs2_option_to_string(option) << " to " << user_set_option_value);
                    sensor.set_option(option, user_set_option_value);
                }
            }
        }
        sensor.set_option(RS2_OPTION_SEQUENCE_ID, 0);   // Set back to default.
        sensor.set_option(RS2_OPTION_HDR_ENABLED, true);

        monitor_update_functions(); // Start parameters update thread

        break;
    }
}

void BaseRealSenseNode::set_sensor_parameter_to_ros(const std::string& module_name, rs2::options sensor, rs2_option option)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;

    int_param.name = create_graph_resource_name(rs2_option_to_string(option)); //"sequence_id"
    int_param.value = sensor.get_option(option);
    conf.ints.push_back(int_param);

    srv_req.config = conf;

    std::string service_name = module_name + "/set_parameters";
    if (ros::service::call(service_name, srv_req, srv_resp)) {
        ROS_INFO_STREAM( "call to set " << service_name << "/" << int_param.name << " to " << int_param.value << " succeeded");
    } else {
        ROS_ERROR_STREAM( "call to set " << service_name << "/" << int_param.name << " to " << int_param.value << " failed");
    }
}

void BaseRealSenseNode::monitor_update_functions()
{
    int time_interval(1000);
    std::function<void()> func = [this, time_interval](){
        std::mutex mu;
        std::unique_lock<std::mutex> lock(mu);
        while(_is_running) {
            _update_functions_cv.wait_for(lock, std::chrono::milliseconds(time_interval), [&]{return !_is_running || !_update_functions_v.empty();});
            while (!_update_functions_v.empty())
            {
                _update_functions_v.back()();
                _update_functions_v.pop_back();                
            }
        }
    };
    _update_functions_t = std::make_shared<std::thread>(func);
}

rs2_stream BaseRealSenseNode::rs2_string_to_stream(std::string str)
{
    if (str == "RS2_STREAM_ANY")
        return RS2_STREAM_ANY;
    if (str == "RS2_STREAM_COLOR")
        return RS2_STREAM_COLOR;
    if (str == "RS2_STREAM_INFRARED")
        return RS2_STREAM_INFRARED;
    if (str == "RS2_STREAM_FISHEYE")
        return RS2_STREAM_FISHEYE;
    throw std::runtime_error("Unknown stream string " + str);
}

void BaseRealSenseNode::getParameters()
{
    ROS_INFO("getParameters...");

    // Setup system to use RGB image from the infra stream if configured by user
    bool infra_rgb;
    _pnh.param("infra_rgb", infra_rgb, false);
    if (infra_rgb)
    {
      _format[RS2_STREAM_INFRARED] = RS2_FORMAT_RGB8;
      _image_format[RS2_STREAM_INFRARED] = CV_8UC3;    // CVBridge type
      _encoding[RS2_STREAM_INFRARED] = sensor_msgs::image_encodings::RGB8; // ROS message type
      _unit_step_size[RS2_STREAM_INFRARED] = 3 * sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
      ROS_INFO_STREAM("Infrared RGB stream enabled");
    }

    _pnh.param("align_depth", _align_depth, ALIGN_DEPTH);
    _pnh.param("enable_pointcloud", _pointcloud, POINTCLOUD);
    std::string pc_texture_stream("");
    int pc_texture_idx;
    _pnh.param("pointcloud_texture_stream", pc_texture_stream, std::string("RS2_STREAM_COLOR"));
    _pnh.param("pointcloud_texture_index", pc_texture_idx, 0);
    _pointcloud_texture = stream_index_pair{rs2_string_to_stream(pc_texture_stream), pc_texture_idx};

    _pnh.param("filters", _filters_str, DEFAULT_FILTERS);
    _pointcloud |= (_filters_str.find("pointcloud") != std::string::npos);

    _pnh.param("publish_tf", _publish_tf, PUBLISH_TF);
    _pnh.param("tf_publish_rate", _tf_publish_rate, TF_PUBLISH_RATE);

    _pnh.param("enable_sync", _sync_frames, SYNC_FRAMES);
    if (_pointcloud || _align_depth || _filters_str.size() > 0)
        _sync_frames = true;

    _pnh.param("json_file_path", _json_file_path, std::string(""));

    for (auto& stream : IMAGE_STREAMS)
    {
        std::string param_name(_stream_name[stream.first] + "_width");
        _pnh.param(param_name, _width[stream], IMAGE_WIDTH);
        ROS_DEBUG_STREAM("parameter:" << param_name << " = " << _width[stream]);
        param_name = _stream_name[stream.first] + "_height";
        _pnh.param(param_name, _height[stream], IMAGE_HEIGHT);
        ROS_DEBUG_STREAM("parameter:" << param_name << " = " << _height[stream]);
        param_name = _stream_name[stream.first] + "_fps";
        _pnh.param(param_name, _fps[stream], IMAGE_FPS);
        ROS_DEBUG_STREAM("parameter:" << param_name << " = " << _fps[stream]);
        param_name = "enable_" + STREAM_NAME(stream);
        _pnh.param(param_name, _enable[stream], true);
        ROS_DEBUG_STREAM("parameter:" << param_name << " = " << _enable[stream]);
    }

    for (auto& stream : HID_STREAMS)
    {
        std::string param_name(_stream_name[stream.first] + "_fps");
        ROS_DEBUG_STREAM("reading parameter:" << param_name);
        _pnh.param(param_name, _fps[stream], IMU_FPS);
        param_name = "enable_" + STREAM_NAME(stream);
        _pnh.param(param_name, _enable[stream], ENABLE_IMU);
        ROS_DEBUG_STREAM("_enable[" << _stream_name[stream.first] << "]:" << _enable[stream]);
    }
    _pnh.param("base_frame_id", _base_frame_id, DEFAULT_BASE_FRAME_ID);
    _pnh.param("odom_frame_id", _odom_frame_id, DEFAULT_ODOM_FRAME_ID);

    std::vector<stream_index_pair> streams(IMAGE_STREAMS);
    streams.insert(streams.end(), HID_STREAMS.begin(), HID_STREAMS.end());
    for (auto& stream : streams)
    {
        std::string param_name(static_cast<std::ostringstream&&>(std::ostringstream() << STREAM_NAME(stream) << "_frame_id").str());
        _pnh.param(param_name, _frame_id[stream], FRAME_ID(stream));
        ROS_DEBUG_STREAM("frame_id: reading parameter:" << param_name << " : " << _frame_id[stream]);
        param_name = static_cast<std::ostringstream&&>(std::ostringstream() << STREAM_NAME(stream) << "_optical_frame_id").str();
        _pnh.param(param_name, _optical_frame_id[stream], OPTICAL_FRAME_ID(stream));
        ROS_DEBUG_STREAM("optical: reading parameter:" << param_name << " : " << _optical_frame_id[stream]);
    }

    std::string unite_imu_method_str("");
    _pnh.param("unite_imu_method", unite_imu_method_str, DEFAULT_UNITE_IMU_METHOD);
    if (unite_imu_method_str == "linear_interpolation")
        _imu_sync_method = imu_sync_method::LINEAR_INTERPOLATION;
    else if (unite_imu_method_str == "copy")
        _imu_sync_method = imu_sync_method::COPY;
    else
        _imu_sync_method = imu_sync_method::NONE;

    if (_imu_sync_method > imu_sync_method::NONE)
    {
        _pnh.param("imu_optical_frame_id", _optical_frame_id[GYRO], DEFAULT_IMU_OPTICAL_FRAME_ID);
    }

    {
        stream_index_pair stream(COLOR);
        std::string param_name(static_cast<std::ostringstream&&>(std::ostringstream() << "aligned_depth_to_" << STREAM_NAME(stream) << "_frame_id").str());
        _pnh.param(param_name, _depth_aligned_frame_id[stream], ALIGNED_DEPTH_TO_FRAME_ID(stream));
    }

    _pnh.param("allow_no_texture_points", _allow_no_texture_points, ALLOW_NO_TEXTURE_POINTS);
    _pnh.param("ordered_pc", _ordered_pc, ORDERED_POINTCLOUD);
    _pnh.param("clip_distance", _clipping_distance, static_cast<float>(-1.0));
    _pnh.param("linear_accel_cov", _linear_accel_cov, static_cast<double>(0.01));
    _pnh.param("angular_velocity_cov", _angular_velocity_cov, static_cast<double>(0.01));
    _pnh.param("hold_back_imu_for_frames", _hold_back_imu_for_frames, HOLD_BACK_IMU_FOR_FRAMES);
    _pnh.param("publish_odom_tf", _publish_odom_tf, PUBLISH_ODOM_TF);
}

void BaseRealSenseNode::setupDevice()
{
    ROS_INFO("setupDevice...");
    try{
        if (!_json_file_path.empty())
        {
            if (_dev.is<rs2::serializable_device>())
            {
                std::stringstream ss;
                std::ifstream in(_json_file_path);
                if (in.is_open())
                {
                    ss << in.rdbuf();
                    std::string json_file_content = ss.str();

                    auto adv = _dev.as<rs2::serializable_device>();
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

        ROS_INFO_STREAM("ROS Node Namespace: " << _namespace);

        auto camera_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
        ROS_INFO_STREAM("Device Name: " << camera_name);

        ROS_INFO_STREAM("Device Serial No: " << _serial_no);

        auto camera_id = _dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);

        ROS_INFO_STREAM("Device physical port: " << camera_id);

        auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        ROS_INFO_STREAM("Device FW version: " << fw_ver);

        auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
        ROS_INFO_STREAM("Device Product ID: 0x" << pid);

        ROS_INFO_STREAM("Enable PointCloud: " << ((_pointcloud)?"On":"Off"));
        ROS_INFO_STREAM("Align Depth: " << ((_align_depth)?"On":"Off"));
        ROS_INFO_STREAM("Sync Mode: " << ((_sync_frames)?"On":"Off"));

        _dev_sensors = _dev.query_sensors();


        std::function<void(rs2::frame)> frame_callback_function, imu_callback_function;
        if (_sync_frames)
        {
            frame_callback_function = _syncer;

            auto frame_callback_inner = [this](rs2::frame frame){
                frame_callback(frame);
            };
            _syncer.start(frame_callback_inner);
        }
        else
        {
            frame_callback_function = [this](rs2::frame frame){frame_callback(frame);};
        }

        if (_imu_sync_method == imu_sync_method::NONE)
        {
            imu_callback_function = [this](rs2::frame frame){imu_callback(frame);};
        }
        else
        {
            imu_callback_function = [this](rs2::frame frame){imu_callback_sync(frame, _imu_sync_method);};
        }
        std::function<void(rs2::frame)> multiple_message_callback_function = [this](rs2::frame frame){multiple_message_callback(frame, _imu_sync_method);};

        ROS_INFO_STREAM("Device Sensors: ");
        for(auto&& sensor : _dev_sensors)
        {
            for (auto& profile : sensor.get_stream_profiles())
            {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                stream_index_pair sip(video_profile.stream_type(), video_profile.stream_index());
                if (_sensors.find( sip ) != _sensors.end())
                    continue;
                _sensors[sip] = sensor;
            }

            std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
            if (sensor.is<rs2::depth_sensor>())
            {
                _sensors_callback[module_name] = frame_callback_function;
            }
            else if (sensor.is<rs2::color_sensor>())
            {
                _sensors_callback[module_name] = frame_callback_function;
            }
            else if (sensor.is<rs2::fisheye_sensor>())
            {
                _sensors_callback[module_name] = frame_callback_function;
            }
            else if (sensor.is<rs2::motion_sensor>())
            {
                _sensors_callback[module_name] = imu_callback_function;
            }
            else if (sensor.is<rs2::pose_sensor>())
            {
                _sensors_callback[module_name] = multiple_message_callback_function;
            }
            else
            {
                ROS_ERROR_STREAM("Module Name \"" << module_name << "\" isn't supported by LibRealSense! Terminating RealSense Node...");
                ros::shutdown();
                exit(1);
            }
            ROS_INFO_STREAM(module_name << " was found.");
        }

        // Update "enable" map
        for (std::pair<stream_index_pair, bool> const& enable : _enable )
        {
            const stream_index_pair& stream_index(enable.first);
            if (enable.second && _sensors.find(stream_index) == _sensors.end())
            {
                ROS_INFO_STREAM("(" << rs2_stream_to_string(stream_index.first) << ", " << stream_index.second << ") sensor isn't supported by current device! -- Skipping...");
                _enable[enable.first] = false;
            }
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::setupPublishers()
{
    ROS_INFO("setupPublishers...");
    image_transport::ImageTransport image_transport(_node_handle);

    for (auto& stream : IMAGE_STREAMS)
    {
        if (_enable[stream])
        {
            std::stringstream image_raw, camera_info, topic_metadata;
            bool rectified_image = false;
            if (stream == DEPTH || stream == CONFIDENCE || stream == INFRA1 || stream == INFRA2)
                rectified_image = true;

            std::string stream_name(STREAM_NAME(stream));
            image_raw << stream_name << "/image_" << ((rectified_image)?"rect_":"") << "raw";
            camera_info << stream_name << "/camera_info";
            topic_metadata << stream_name << "/metadata";

            std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(_fps[stream], stream_name, _serial_no));
            _image_publishers[stream] = {image_transport.advertise(image_raw.str(), 1), frequency_diagnostics};
            _info_publisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(camera_info.str(), 1);
            _metadata_publishers[stream] = std::make_shared<ros::Publisher>(_node_handle.advertise<realsense2_camera::Metadata>(topic_metadata.str(), 1));

            if (_align_depth && stream == COLOR)
            {
                std::stringstream aligned_image_raw, aligned_camera_info;
                aligned_image_raw << "aligned_depth_to_" << stream_name << "/image_raw";
                aligned_camera_info << "aligned_depth_to_" << stream_name << "/camera_info";

                std::string aligned_stream_name = "aligned_depth_to_" + stream_name;
                std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(_fps[stream], aligned_stream_name, _serial_no));
                _depth_aligned_image_publishers[stream] = {image_transport.advertise(aligned_image_raw.str(), 1), frequency_diagnostics};
                _depth_aligned_info_publisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(aligned_camera_info.str(), 1);
            }

            if (stream == DEPTH && _pointcloud)
            {
                _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);
            }
        }
    }

    _synced_imu_publisher = std::make_shared<SyncedImuPublisher>();
    if (_imu_sync_method > imu_sync_method::NONE && _enable[GYRO] && _enable[ACCEL])
    {
        ROS_INFO("Start publisher IMU");
        _synced_imu_publisher = std::make_shared<SyncedImuPublisher>(_node_handle.advertise<sensor_msgs::Imu>("imu", 5));
        _synced_imu_publisher->Enable(_hold_back_imu_for_frames);
    }
    else
    {
        if (_enable[GYRO])
        {
            _imu_publishers[GYRO] = _node_handle.advertise<sensor_msgs::Imu>("gyro/sample", 100);
            _metadata_publishers[GYRO] = std::make_shared<ros::Publisher>(_node_handle.advertise<realsense2_camera::Metadata>("gyro/metadata", 1));
        }

        if (_enable[ACCEL])
        {
            _imu_publishers[ACCEL] = _node_handle.advertise<sensor_msgs::Imu>("accel/sample", 100);
            _metadata_publishers[ACCEL] = std::make_shared<ros::Publisher>(_node_handle.advertise<realsense2_camera::Metadata>("accel/metadata", 1));
        }
    }
    if (_enable[POSE])
    {
        _imu_publishers[POSE] = _node_handle.advertise<nav_msgs::Odometry>("odom/sample", 100);
        _metadata_publishers[POSE] = std::make_shared<ros::Publisher>(_node_handle.advertise<realsense2_camera::Metadata>("odom/metadata", 1));
    }


    if (_enable[FISHEYE] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[FISHEYE] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_fisheye", 1, true);
    }

    if (_enable[COLOR] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[COLOR] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_color", 1, true);
    }

    if (_enable[INFRA1] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[INFRA1] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_infra1", 1, true);
    }

    if (_enable[INFRA2] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[INFRA2] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_infra2", 1, true);
    }
}

void BaseRealSenseNode::enable_devices()
{
    for (auto& elem : IMAGE_STREAMS)
    {
        if (_enable[elem])
        {
            auto& sens = _sensors[elem];
            auto profiles = sens.get_stream_profiles();
            rs2::stream_profile default_profile, selected_profile;
            for (auto& profile : profiles)
            {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                ROS_DEBUG_STREAM("Sensor profile: " <<
                                    "stream_type: " << rs2_stream_to_string(video_profile.stream_type()) << "(" << video_profile.stream_index() << ")" <<
                                    "Format: " << video_profile.format() <<
                                    ", Width: " << video_profile.width() <<
                                    ", Height: " << video_profile.height() <<
                                    ", FPS: " << video_profile.fps());

                if (profile.stream_type() == elem.first && profile.stream_index() == elem.second)
                {
                    if (profile.is_default())
                    {
                        default_profile = profile;
                    }
                    if ((_width[elem] == 0 || video_profile.width() == _width[elem]) &&
                        (_height[elem] == 0 || video_profile.height() == _height[elem]) &&
                        (_fps[elem] == 0 || video_profile.fps() == _fps[elem]) &&
                        (_format.find(elem.first) == _format.end() || video_profile.format() == _format[elem.first] ))
                    {
                        selected_profile = profile;
                        break;
                    }
                }

            }
            if (!selected_profile)
            {
                ROS_WARN_STREAM_COND((_width[elem]!=-1 || _height[elem]!=-1 || _fps[elem]!=-1), "Given stream configuration is not supported by the device! " <<
                    " Stream: " << rs2_stream_to_string(elem.first) <<
                    ", Stream Index: " << elem.second <<
                    ", Width: " << _width[elem] <<
                    ", Height: " << _height[elem] <<
                    ", FPS: " << _fps[elem] << 
                    ", Format: " << ((_format.find(elem.first) == _format.end())? "None":rs2_format_to_string(rs2_format(_format[elem.first]))));
                if (default_profile)
                {
                    ROS_WARN_STREAM_COND((_width[elem]!=-1 || _height[elem]!=-1 || _fps[elem]!=-1), "Using default profile instead.");
                    selected_profile = default_profile;
                }
            }
            if (selected_profile)
            {
                auto video_profile = selected_profile.as<rs2::video_stream_profile>();
                _width[elem] = video_profile.width();
                _height[elem] = video_profile.height();
                _fps[elem] = video_profile.fps();
                _enabled_profiles[elem].push_back(selected_profile);
                _image[elem] = cv::Mat(_height[elem], _width[elem], _image_format[elem.first], cv::Scalar(0, 0, 0));
                ROS_INFO_STREAM(STREAM_NAME(elem) << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem] << ", fps: " << _fps[elem] << ", " << "Format: " << video_profile.format());
            }
            else
            {
                _enable[elem] = false;
            }
        }
    }
	if (_align_depth)
	{
		for (auto& profiles : _enabled_profiles)
		{
			_depth_aligned_image[profiles.first] = cv::Mat(_height[DEPTH], _width[DEPTH], _image_format[DEPTH.first], cv::Scalar(0, 0, 0));
			_depth_scaled_image[profiles.first] = cv::Mat(_height[DEPTH], _width[DEPTH], _image_format[DEPTH.first], cv::Scalar(0, 0, 0));
		}
	}

    // Streaming HID
    for (auto& elem : HID_STREAMS)
    {
        if (_enable[elem])
        {
            auto& sens = _sensors[elem];
            auto profiles = sens.get_stream_profiles();
            rs2::stream_profile default_profile, selected_profile;
            ROS_DEBUG_STREAM("Available profiles:");
            for (rs2::stream_profile& profile : profiles)
            {
                ROS_DEBUG_STREAM("type:" << rs2_stream_to_string(profile.stream_type()) <<
                                " fps: " << profile.fps() << ". format: " << profile.format());
            }
            for (rs2::stream_profile& profile : profiles)
            {
                if (profile.stream_type() == elem.first)
                {
                    if (profile.is_default())
                        default_profile = profile;
                    if (_fps[elem] == 0 || profile.fps() == _fps[elem])
                    {
                        selected_profile = profile;
                        break;
                    }
                }
            }
            if (!selected_profile)
            {
                std::string stream_name(STREAM_NAME(elem));
                ROS_WARN_STREAM_COND((_fps[elem]!=-1), "No mathcing profile found for " << stream_name << " with fps=" << _fps[elem]);
                if (default_profile)
                {
                    ROS_WARN_STREAM_COND((_fps[elem]!=-1), "Using default profile instead.");
                    selected_profile = default_profile;
                }
            }
            if(selected_profile)
            {
                _fps[elem] = selected_profile.fps();
                _enabled_profiles[elem].push_back(selected_profile);
                ROS_INFO_STREAM(STREAM_NAME(elem) << " stream is enabled - fps: " << _fps[elem]);
            }
            else
            {
                _enable[elem] = false;
            }
        }
    }
}

void BaseRealSenseNode::setupFilters()
{
    std::vector<std::string> filters_str;
    boost::split(filters_str, _filters_str, [](char c){return c == ',';});
    bool use_disparity_filter(false);
    bool use_colorizer_filter(false);
    bool use_decimation_filter(false);
    bool use_hdr_filter(false);
    for (std::vector<std::string>::iterator s_iter=filters_str.begin(); s_iter!=filters_str.end(); s_iter++)
    {
        (*s_iter).erase(std::remove_if((*s_iter).begin(), (*s_iter).end(), isspace), (*s_iter).end()); // Remove spaces

        if ((*s_iter) == "colorizer")
        {
            use_colorizer_filter = true;
        }
        else if ((*s_iter) == "disparity")
        {
            use_disparity_filter = true;
        }
        else if ((*s_iter) == "spatial")
        {
            ROS_INFO("Add Filter: spatial");
            _filters.push_back(NamedFilter("spatial", std::make_shared<rs2::spatial_filter>()));
        }
        else if ((*s_iter) == "temporal")
        {
            ROS_INFO("Add Filter: temporal");
            _filters.push_back(NamedFilter("temporal", std::make_shared<rs2::temporal_filter>()));
        }
        else if ((*s_iter) == "hole_filling")
        {
            ROS_INFO("Add Filter: hole_filling");
            _filters.push_back(NamedFilter("hole_filling", std::make_shared<rs2::hole_filling_filter>()));
        }
        else if ((*s_iter) == "decimation")
        {
            use_decimation_filter = true;
        }
        else if ((*s_iter) == "pointcloud")
        {
            assert(_pointcloud); // For now, it is set in getParameters()..
        }
        else if ((*s_iter) == "hdr_merge")
        {
            use_hdr_filter = true;
        }
        else if ((*s_iter).size() > 0)
        {
            ROS_ERROR_STREAM("Unknown Filter: " << (*s_iter));
            throw;
        }
    }
    if (use_disparity_filter)
    {
        ROS_INFO("Add Filter: disparity");
        _filters.insert(_filters.begin(), NamedFilter("disparity_start", std::make_shared<rs2::disparity_transform>()));
        _filters.push_back(NamedFilter("disparity_end", std::make_shared<rs2::disparity_transform>(false)));
        ROS_INFO("Done Add Filter: disparity");
    }
    if (use_hdr_filter)
    {
      ROS_INFO("Add Filter: hdr_merge");
      _filters.insert(_filters.begin(),NamedFilter("hdr_merge", std::make_shared<rs2::hdr_merge>()));
      ROS_INFO("Add Filter: sequence_id_filter");
      _filters.insert(_filters.begin(),NamedFilter("sequence_id_filter", std::make_shared<rs2::sequence_id_filter>()));
    }
    if (use_decimation_filter)
    {
      ROS_INFO("Add Filter: decimation");
      _filters.insert(_filters.begin(),NamedFilter("decimation", std::make_shared<rs2::decimation_filter>()));
    }
    if (_align_depth)
    {
        _filters.push_back(NamedFilter("align_to_color", std::make_shared<rs2::align>(RS2_STREAM_COLOR)));
    }
    if (use_colorizer_filter)
    {
        ROS_INFO("Add Filter: colorizer");
        _colorizer = std::make_shared<rs2::colorizer>();
        _filters.push_back(NamedFilter("colorizer", _colorizer));
        // Types for depth stream
        _image_format[DEPTH.first] = _image_format[COLOR.first];    // CVBridge type
        _encoding[DEPTH.first] = _encoding[COLOR.first]; // ROS message type
        _unit_step_size[DEPTH.first] = _unit_step_size[COLOR.first]; // sensor_msgs::ImagePtr row step size
        _depth_aligned_encoding[RS2_STREAM_COLOR] = _encoding[COLOR.first]; // ROS message type

        _width[DEPTH] = _width[COLOR];
        _height[DEPTH] = _height[COLOR];
        _image[DEPTH] = cv::Mat(std::max(0, _height[DEPTH]), std::max(0, _width[DEPTH]), _image_format[DEPTH.first], cv::Scalar(0, 0, 0));
    }
    if (_pointcloud)
    {
    	ROS_INFO("Add Filter: pointcloud");
        _pointcloud_filter = std::make_shared<rs2::pointcloud>(_pointcloud_texture.first, _pointcloud_texture.second);
        _filters.push_back(NamedFilter("pointcloud", _pointcloud_filter));
    }
    ROS_INFO("num_filters: %d", static_cast<int>(_filters.size()));
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

    CV_Assert(from_image.depth() == _image_format[RS2_STREAM_DEPTH]);

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

sensor_msgs::Imu BaseRealSenseNode::CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data)
{
    sensor_msgs::Imu imu_msg;
    ros::Time t(gyro_data.m_time);
    imu_msg.header.seq = 0;
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

void BaseRealSenseNode::FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs)
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
            const double dt = accel1.m_time - accel0.m_time;

            while (gyros_data.size())
            {
                CimuData crnt_gyro = gyros_data.front();
                gyros_data.pop_front();
                const double alpha = (crnt_gyro.m_time - accel0.m_time) / dt;
                CimuData crnt_accel(ACCEL, lerp(accel0.m_data, accel1.m_data, alpha), crnt_gyro.m_time);
                imu_msgs.push_back(CreateUnitedMessage(crnt_accel, crnt_gyro));
            }
            accel0 = accel1;
        } 
        else if (accel0.is_set() && crnt_imu.m_time >= accel0.m_time && crnt_imu.m_type == GYRO)
        {
            gyros_data.push_back(crnt_imu);
        }
    }
    _imu_history.push_back(crnt_imu);
    return;
}

void BaseRealSenseNode::FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::Imu>& imu_msgs)
{
    stream_index_pair type(imu_data.m_type);

    static CimuData _accel_data(ACCEL, {0,0,0}, -1.0);
    if (ACCEL == type)
    {
        _accel_data = imu_data;
        return;
    }
    if (_accel_data.m_time < 0)
        return;

    imu_msgs.push_back(CreateUnitedMessage(_accel_data, imu_data));
}

void BaseRealSenseNode::ImuMessage_AddDefaultValues(sensor_msgs::Imu& imu_msg)
{
    imu_msg.header.frame_id = _optical_frame_id[GYRO];
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
    static int seq = 0;

    m_mutex.lock();

    auto stream = frame.get_profile().stream_type();
    auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
    double frame_time = frame.get_timestamp();

    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    seq += 1;

    if (0 != _synced_imu_publisher->getNumSubscribers())
    {
        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
        CimuData imu_data(stream_index, v, frameSystemTimeSec(frame));
        std::deque<sensor_msgs::Imu> imu_msgs;
        switch (sync_method)
        {
            case NONE: //Cannot really be NONE. Just to avoid compilation warning.
            case COPY:
                FillImuData_Copy(imu_data, imu_msgs);
                break;
            case LINEAR_INTERPOLATION:
                FillImuData_LinearInterpolation(imu_data, imu_msgs);
                break;
        }
        while (imu_msgs.size())
        {
            sensor_msgs::Imu imu_msg = imu_msgs.front();
            imu_msg.header.seq = seq;
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
                rs2_stream_to_string(frame.get_profile().stream_type()),
                frame.get_profile().stream_index(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));

    auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
    ros::Time t(frameSystemTimeSec(frame));
    if (0 != _imu_publishers[stream_index].getNumSubscribers())
    {
        auto imu_msg = sensor_msgs::Imu();
        ImuMessage_AddDefaultValues(imu_msg);
        imu_msg.header.frame_id = _optical_frame_id[stream_index];

        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        if (GYRO == stream_index)
        {
            imu_msg.angular_velocity.x = crnt_reading.x;
            imu_msg.angular_velocity.y = crnt_reading.y;
            imu_msg.angular_velocity.z = crnt_reading.z;
        }
        else if (ACCEL == stream_index)
        {
            imu_msg.linear_acceleration.x = crnt_reading.x;
            imu_msg.linear_acceleration.y = crnt_reading.y;
            imu_msg.linear_acceleration.z = crnt_reading.z;
        }
        _seq[stream_index] += 1;
        imu_msg.header.seq = _seq[stream_index];
        imu_msg.header.stamp = t;
        _imu_publishers[stream_index].publish(imu_msg);
        ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
    }
    publishMetadata(frame, t, _optical_frame_id[stream_index]);
}

void BaseRealSenseNode::pose_callback(rs2::frame frame)
{
    double frame_time = frame.get_timestamp();
    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                rs2_stream_to_string(frame.get_profile().stream_type()),
                frame.get_profile().stream_index(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));
    const auto& stream_index(POSE);
    rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
    ros::Time t(frameSystemTimeSec(frame));

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = -pose.translation.z;
    pose_msg.pose.position.y = -pose.translation.x;
    pose_msg.pose.position.z = pose.translation.y;
    pose_msg.pose.orientation.x = -pose.rotation.z;
    pose_msg.pose.orientation.y = -pose.rotation.x;
    pose_msg.pose.orientation.z = pose.rotation.y;
    pose_msg.pose.orientation.w = pose.rotation.w;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = _odom_frame_id;
    msg.child_frame_id = _frame_id[POSE];
    msg.transform.translation.x = pose_msg.pose.position.x;
    msg.transform.translation.y = pose_msg.pose.position.y;
    msg.transform.translation.z = pose_msg.pose.position.z;
    msg.transform.rotation.x = pose_msg.pose.orientation.x;
    msg.transform.rotation.y = pose_msg.pose.orientation.y;
    msg.transform.rotation.z = pose_msg.pose.orientation.z;
    msg.transform.rotation.w = pose_msg.pose.orientation.w;

    if (_publish_odom_tf) br.sendTransform(msg);

    if (0 != _imu_publishers[stream_index].getNumSubscribers())
    {
        double cov_pose(_linear_accel_cov * pow(10, 3-(int)pose.tracker_confidence));
        double cov_twist(_angular_velocity_cov * pow(10, 1-(int)pose.tracker_confidence));

        geometry_msgs::Vector3Stamped v_msg;
        v_msg.vector.x = -pose.velocity.z;
        v_msg.vector.y = -pose.velocity.x;
        v_msg.vector.z = pose.velocity.y;
        tf::Vector3 tfv;
        tf::vector3MsgToTF(v_msg.vector,tfv);
        tf::Quaternion q(-msg.transform.rotation.x,-msg.transform.rotation.y,-msg.transform.rotation.z,msg.transform.rotation.w);
        tfv=tf::quatRotate(q,tfv);
        tf::vector3TFToMsg(tfv,v_msg.vector);
	
        geometry_msgs::Vector3Stamped om_msg;
        om_msg.vector.x = -pose.angular_velocity.z;
        om_msg.vector.y = -pose.angular_velocity.x;
        om_msg.vector.z = pose.angular_velocity.y;
        tf::vector3MsgToTF(om_msg.vector,tfv);
        tfv=tf::quatRotate(q,tfv);
        tf::vector3TFToMsg(tfv,om_msg.vector);
	

        nav_msgs::Odometry odom_msg;
        _seq[stream_index] += 1;

        odom_msg.header.frame_id = _odom_frame_id;
        odom_msg.child_frame_id = _frame_id[POSE];
        odom_msg.header.stamp = t;
        odom_msg.header.seq = _seq[stream_index];
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.pose.covariance = {cov_pose, 0, 0, 0, 0, 0,
                                    0, cov_pose, 0, 0, 0, 0,
                                    0, 0, cov_pose, 0, 0, 0,
                                    0, 0, 0, cov_twist, 0, 0,
                                    0, 0, 0, 0, cov_twist, 0,
                                    0, 0, 0, 0, 0, cov_twist};
        odom_msg.twist.twist.linear = v_msg.vector;
        odom_msg.twist.twist.angular = om_msg.vector;
        odom_msg.twist.covariance ={cov_pose, 0, 0, 0, 0, 0,
                                    0, cov_pose, 0, 0, 0, 0,
                                    0, 0, cov_pose, 0, 0, 0,
                                    0, 0, 0, cov_twist, 0, 0,
                                    0, 0, 0, 0, cov_twist, 0,
                                    0, 0, 0, 0, 0, cov_twist};
        _imu_publishers[stream_index].publish(odom_msg);
        ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
    }
    publishMetadata(frame, t, _frame_id[POSE]);
}

void BaseRealSenseNode::frame_callback(rs2::frame frame)
{
    _synced_imu_publisher->Pause();
    
    try{
        double frame_time = frame.get_timestamp();

        // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
        // and the incremental timestamp from the camera.
        // In sync mode the timestamp is based on ROS time
        bool placeholder_false(false);
        if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
        {
            _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
        }

        ros::Time t(frameSystemTimeSec(frame));
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
                            rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), stream_unique_id, frame.get_frame_number(), frame_time, t.toNSec());
                runFirstFrameInitialization(stream_type);
            }
            // Clip depth_frame for max range:
            rs2::depth_frame original_depth_frame = frameset.get_depth_frame();
            bool is_color_frame(frameset.get_color_frame());
            if (original_depth_frame && _clipping_distance > 0)
            {
                clip_depth(original_depth_frame, _clipping_distance);
            }

            ROS_DEBUG("num_filters: %d", static_cast<int>(_filters.size()));
            for (std::vector<NamedFilter>::const_iterator filter_it = _filters.begin(); filter_it != _filters.end(); filter_it++)
            {
                ROS_DEBUG("Applying filter: %s", filter_it->_name.c_str());
                if ((filter_it->_name == "pointcloud") && (!original_depth_frame))
                    continue;
                if ((filter_it->_name == "align_to_color") && (!is_color_frame))
                    continue;
                frameset = filter_it->_filter->process(frameset);
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
                            rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), f.get_frame_number(), frame_time, t.toNSec());
                if (f.is<rs2::video_frame>())
                    ROS_DEBUG_STREAM("frame: " << f.as<rs2::video_frame>().get_width() << " x " << f.as<rs2::video_frame>().get_height());

                if (f.is<rs2::points>())
                {
                    publishPointCloud(f.as<rs2::points>(), t, frameset);
                    continue;
                }
                if (stream_type == RS2_STREAM_DEPTH)
                {
                    if (sent_depth_frame) continue;
                    sent_depth_frame = true;
                    if (_align_depth && is_color_frame)
                    {
                        publishFrame(f, t, COLOR,
                                    _depth_aligned_image,
                                    _depth_aligned_info_publisher,
                                    _depth_aligned_image_publishers,
                                    false,
                                    _depth_aligned_seq,
                                    _depth_aligned_camera_info,
                                    _depth_aligned_encoding);
                        continue;
                    }
                }
                publishFrame(f, t,
                                sip,
                                _image,
                                _info_publisher,
                                _image_publishers,
                                true,
                                _seq,
                                _camera_info,
                                _encoding);
            }
            if (original_depth_frame && _align_depth)
            {
                rs2::frame frame_to_send;
                if (_colorizer)
                    frame_to_send = _colorizer->process(original_depth_frame);
                else
                    frame_to_send = original_depth_frame;

                publishFrame(frame_to_send, t,
                                DEPTH,
                                _image,
                                _info_publisher,
                                _image_publishers,
                                true,
                                _seq,
                                _camera_info,
                                _encoding);
            }
        }
        else if (frame.is<rs2::video_frame>())
        {
            auto stream_type = frame.get_profile().stream_type();
            auto stream_index = frame.get_profile().stream_index();
            ROS_DEBUG("Single video frame arrived (%s, %d). frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                        rs2_stream_to_string(stream_type), stream_index, frame.get_frame_number(), frame_time, t.toNSec());
            runFirstFrameInitialization(stream_type);

            stream_index_pair sip{stream_type,stream_index};
            if (frame.is<rs2::depth_frame>())
            {
                if (_clipping_distance > 0)
                {
                    clip_depth(frame, _clipping_distance);
                }
            }
            publishFrame(frame, t,
                            sip,
                            _image,
                            _info_publisher,
                            _image_publishers,
                            true,
                            _seq,
                            _camera_info,
                            _encoding);
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An error has occurred during frame callback: " << ex.what());
    }
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
        case RS2_STREAM_POSE:
            pose_callback(frame);
            break;
        default:
            frame_callback(frame);
    }
}

bool BaseRealSenseNode::setBaseTime(double frame_time, rs2_timestamp_domain time_domain)
{
    ROS_WARN_ONCE(time_domain == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME ? "Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)" : "");
    if (time_domain == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        ROS_WARN("frame's time domain is HARDWARE_CLOCK. Timestamps may reset periodically.");
        _ros_time_base = ros::Time::now();
        _camera_time_base = frame_time;
        _previous_frame_time = frame_time;
        return true;
    }
    return false;
}

double BaseRealSenseNode::frameSystemTimeSec(rs2::frame frame)
{
    if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        if (_previous_frame_time > frame.get_timestamp())
        {
            ROS_WARN("Hardware clock reset detected. Resetting ROS time base.");
            _ros_time_base = ros::Time::now();
            _camera_time_base = frame.get_timestamp();
        }
        _previous_frame_time = frame.get_timestamp();

        double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / 1000.0;
        return (_ros_time_base.toSec() + elapsed_camera_ms);
    }
    else
    {
        return (frame.get_timestamp() / 1000.0);
    }
}

void BaseRealSenseNode::setupStreams()
{
	ROS_INFO("setupStreams...");
    try{
		// Publish image stream info
        for (auto& profiles : _enabled_profiles)
        {
            for (auto& profile : profiles.second)
            {
                if (profile.is<rs2::video_stream_profile>())
                {
                    auto video_profile = profile.as<rs2::video_stream_profile>();
                    updateStreamCalibData(video_profile);
                }
            }
        }

        // Streaming IMAGES
        std::map<std::string, std::vector<rs2::stream_profile> > profiles;
        std::map<std::string, rs2::sensor> active_sensors;
        for (const std::pair<stream_index_pair, std::vector<rs2::stream_profile>>& profile : _enabled_profiles)
        {
            std::string module_name = _sensors[profile.first].get_info(RS2_CAMERA_INFO_NAME);
            ROS_DEBUG_STREAM("insert " << rs2_stream_to_string(profile.second.begin()->stream_type())
              << " to " << module_name);
            profiles[module_name].insert(profiles[module_name].begin(),
                                            profile.second.begin(),
                                            profile.second.end());
            active_sensors[module_name] = _sensors[profile.first];
        }

        for (const std::pair<std::string, std::vector<rs2::stream_profile> >& sensor_profile : profiles)
        {
            std::string module_name = sensor_profile.first;
            rs2::sensor sensor = active_sensors[module_name];
            sensor.open(sensor_profile.second);
            sensor.start(_sensors_callback[module_name]);
            if (sensor.is<rs2::depth_sensor>())
            {
                _depth_scale_meters = sensor.as<rs2::depth_sensor>().get_depth_scale();
            }
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::updateStreamCalibData(const rs2::video_stream_profile& video_profile)
{
    stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
    auto intrinsic = video_profile.get_intrinsics();
    _stream_intrinsics[stream_index] = intrinsic;
    _camera_info[stream_index].width = intrinsic.width;
    _camera_info[stream_index].height = intrinsic.height;
    _camera_info[stream_index].header.frame_id = _optical_frame_id[stream_index];

    _camera_info[stream_index].K.at(0) = intrinsic.fx;
    _camera_info[stream_index].K.at(2) = intrinsic.ppx;
    _camera_info[stream_index].K.at(4) = intrinsic.fy;
    _camera_info[stream_index].K.at(5) = intrinsic.ppy;
    _camera_info[stream_index].K.at(8) = 1;

    _camera_info[stream_index].P.at(0) = _camera_info[stream_index].K.at(0);
    _camera_info[stream_index].P.at(1) = 0;
    _camera_info[stream_index].P.at(2) = _camera_info[stream_index].K.at(2);
    _camera_info[stream_index].P.at(3) = 0;
    _camera_info[stream_index].P.at(4) = 0;
    _camera_info[stream_index].P.at(5) = _camera_info[stream_index].K.at(4);
    _camera_info[stream_index].P.at(6) = _camera_info[stream_index].K.at(5);
    _camera_info[stream_index].P.at(7) = 0;
    _camera_info[stream_index].P.at(8) = 0;
    _camera_info[stream_index].P.at(9) = 0;
    _camera_info[stream_index].P.at(10) = 1;
    _camera_info[stream_index].P.at(11) = 0;

    // Set Tx, Ty for right camera
    if (stream_index.second == 2)
    {
        stream_index_pair sip1{stream_index.first, 1};
        if (_enable[sip1])
        {
            const auto& ex = getAProfile(stream_index).get_extrinsics_to(getAProfile(sip1));
            _camera_info[stream_index].header.frame_id = _optical_frame_id[sip1];
            _camera_info[stream_index].P.at(3) = -intrinsic.fx * ex.translation[0] + 0.0; // Tx - avoid -0.0 values.
            _camera_info[stream_index].P.at(7) = -intrinsic.fy * ex.translation[1] + 0.0; // Ty - avoid -0.0 values.
        }
    }

    // set R (rotation matrix) values to identity matrix
    _camera_info[stream_index].R.at(0) = 1.0;
    _camera_info[stream_index].R.at(1) = 0.0;
    _camera_info[stream_index].R.at(2) = 0.0;
    _camera_info[stream_index].R.at(3) = 0.0;
    _camera_info[stream_index].R.at(4) = 1.0;
    _camera_info[stream_index].R.at(5) = 0.0;
    _camera_info[stream_index].R.at(6) = 0.0;
    _camera_info[stream_index].R.at(7) = 0.0;
    _camera_info[stream_index].R.at(8) = 1.0;

    int coeff_size(5);
    if (intrinsic.model == RS2_DISTORTION_KANNALA_BRANDT4)
    {
        _camera_info[stream_index].distortion_model = "equidistant";
        coeff_size = 4;
    } else {
        _camera_info[stream_index].distortion_model = "plumb_bob";
    }
    _camera_info[stream_index].D.resize(coeff_size);
    for (int i = 0; i < coeff_size; i++)
    {
        _camera_info[stream_index].D.at(i) = intrinsic.coeffs[i];
    }

    if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR])
    {
        _camera_info[stream_index].P.at(3) = 0;     // Tx
        _camera_info[stream_index].P.at(7) = 0;     // Ty
    }

    if (_align_depth)
    {
        for (auto& profiles : _enabled_profiles)
        {
            for (auto& profile : profiles.second)
            {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
                _depth_aligned_camera_info[stream_index] = _camera_info[stream_index];
            }
        }
    }
}

tf::Quaternion BaseRealSenseNode::rotationMatrixToQuaternion(const float rotation[9]) const
{
    Eigen::Matrix3f m;
    // We need to be careful about the order, as RS2 rotation matrix is
    // column-major, while Eigen::Matrix3f expects row-major.
    m << rotation[0], rotation[3], rotation[6],
         rotation[1], rotation[4], rotation[7],
         rotation[2], rotation[5], rotation[8];
    Eigen::Quaternionf q(m);
    return tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}

void BaseRealSenseNode::publish_static_tf(const ros::Time& t,
                                          const float3& trans,
                                          const tf::Quaternion& q,
                                          const std::string& from,
                                          const std::string& to)
{
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = from;
    msg.child_frame_id = to;
    msg.transform.translation.x = trans.z;
    msg.transform.translation.y = -trans.x;
    msg.transform.translation.z = -trans.y;
    msg.transform.rotation.x = q.getX();
    msg.transform.rotation.y = q.getY();
    msg.transform.rotation.z = q.getZ();
    msg.transform.rotation.w = q.getW();
    _static_tf_msgs.push_back(msg);
}

void BaseRealSenseNode::calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    tf::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    float3 zero_trans{0, 0, 0};

    ros::Time transform_ts_ = ros::Time::now();

    rs2_extrinsics ex;
    try
    {
        ex = getAProfile(stream).get_extrinsics_to(base_profile);
    }
    catch (std::exception& e)
    {
        if (!strcmp(e.what(), "Requested extrinsics are not available!"))
        {
            ROS_WARN_STREAM("(" << rs2_stream_to_string(stream.first) << ", " << stream.second << ") -> (" << rs2_stream_to_string(base_profile.stream_type()) << ", " << base_profile.stream_index() << "): " << e.what() << " : using unity as default.");
            ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
        }
        else
        {
            throw e;
        }
    }

    auto Q = rotationMatrixToQuaternion(ex.rotation);
    Q = quaternion_optical * Q * quaternion_optical.inverse();

    float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
    publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _frame_id[stream]);

    // Transform stream frame to stream optical frame
    publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _frame_id[stream], _optical_frame_id[stream]);

    if (_align_depth && _depth_aligned_frame_id.find(stream) != _depth_aligned_frame_id.end())
    {
        publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _depth_aligned_frame_id[stream]);
        publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _depth_aligned_frame_id[stream], _optical_frame_id[stream]);
    }
}

void BaseRealSenseNode::SetBaseStream()
{
    const std::vector<stream_index_pair> base_stream_priority = {DEPTH, POSE};

    std::vector<stream_index_pair>::const_iterator base_stream(base_stream_priority.begin());
    while( (_sensors.find(*base_stream) == _sensors.end()) && (base_stream != base_stream_priority.end()))
    {
        base_stream++;
    }
    if (base_stream == base_stream_priority.end())
    {
        throw std::runtime_error("No known base_stream found for transformations.");
    }
    ROS_INFO_STREAM("SELECTED BASE:" << base_stream->first << ", " << base_stream->second);

    _base_stream = *base_stream;
}

void BaseRealSenseNode::publishStaticTransforms()
{
    rs2::stream_profile base_profile = getAProfile(_base_stream);
    // Publish static transforms
    if (_publish_tf)
    {
        for (std::pair<stream_index_pair, bool> ienable : _enable)
        {
            if (ienable.second)
            {
                calcAndPublishStaticTransform(ienable.first, base_profile);
            }
        }
        // Static transform for non-positive values
        if (_tf_publish_rate > 0)
            _tf_t = std::shared_ptr<std::thread>(new std::thread(boost::bind(&BaseRealSenseNode::publishDynamicTransforms, this)));
        else
            _static_tf_broadcaster.sendTransform(_static_tf_msgs);
    }

    // Publish Extrinsics Topics:
    if (_enable[DEPTH] &&
        _enable[FISHEYE])
    {
        static const char* frame_id = "depth_to_fisheye_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(FISHEYE));

        _depth_to_other_extrinsics[FISHEYE] = ex;
        _depth_to_other_extrinsics_publishers[FISHEYE].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

    if (_enable[DEPTH] &&
        _enable[COLOR])
    {
        static const char* frame_id = "depth_to_color_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(COLOR));
        _depth_to_other_extrinsics[COLOR] = ex;
        _depth_to_other_extrinsics_publishers[COLOR].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

    if (_enable[DEPTH] &&
        _enable[INFRA1])
    {
        static const char* frame_id = "depth_to_infra1_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(INFRA1));
        _depth_to_other_extrinsics[INFRA1] = ex;
        _depth_to_other_extrinsics_publishers[INFRA1].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

    if (_enable[DEPTH] &&
        _enable[INFRA2])
    {
        static const char* frame_id = "depth_to_infra2_extrinsics";
        const auto& ex = base_profile.get_extrinsics_to(getAProfile(INFRA2));
        _depth_to_other_extrinsics[INFRA2] = ex;
        _depth_to_other_extrinsics_publishers[INFRA2].publish(rsExtrinsicsToMsg(ex, frame_id));
    }

}

void BaseRealSenseNode::publishDynamicTransforms()
{
    // Publish transforms for the cameras
    ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", _tf_publish_rate);

    std::mutex mu;
    std::unique_lock<std::mutex> lock(mu);
    while (ros::ok() && _is_running)
    {
        _cv_tf.wait_for(lock, std::chrono::milliseconds((int)(1000.0/_tf_publish_rate)), [&]{return (!(_is_running));});
        {
            ros::Time t = ros::Time::now();
            for(auto& msg : _static_tf_msgs)
                msg.header.stamp = t;
            _dynamic_tf_broadcaster.sendTransform(_static_tf_msgs);
        }
    }
}

void BaseRealSenseNode::publishIntrinsics()
{
    if (_enable[GYRO])
    {
        _info_publisher[GYRO] = _node_handle.advertise<IMUInfo>("gyro/imu_info", 1, true);
        IMUInfo info_msg = getImuInfo(GYRO);
        _info_publisher[GYRO].publish(info_msg);
    }

    if (_enable[ACCEL])
    {
        _info_publisher[ACCEL] = _node_handle.advertise<IMUInfo>("accel/imu_info", 1, true);
        IMUInfo info_msg = getImuInfo(ACCEL);
        _info_publisher[ACCEL].publish(info_msg);
    }
}

void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
    size_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];

}

void BaseRealSenseNode::publishPointCloud(rs2::points pc, const ros::Time& t, const rs2::frameset& frameset)
{
    if (0 == _pointcloud_publisher.getNumSubscribers())
        return;
    ROS_INFO_STREAM_ONCE("publishing " << (_ordered_pc ? "" : "un") << "ordered pointcloud.");

    rs2_stream texture_source_id = static_cast<rs2_stream>(_pointcloud_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
    bool use_texture = texture_source_id != RS2_STREAM_ANY;
    static int warn_count(0);
    static const int DISPLAY_WARN_NUMBER(5);
    rs2::frameset::iterator texture_frame_itr = frameset.end();
    if (use_texture)
    {
        std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8 };

        texture_frame_itr = std::find_if(frameset.begin(), frameset.end(), [&texture_source_id, &available_formats] (rs2::frame f)
                                {return (rs2_stream(f.get_profile().stream_type()) == texture_source_id) &&
                                            (available_formats.find(f.get_profile().format()) != available_formats.end()); });
        if (texture_frame_itr == frameset.end())
        {
            warn_count++;
            std::string texture_source_name = _pointcloud_filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
            ROS_WARN_STREAM_COND(warn_count == DISPLAY_WARN_NUMBER, "No stream match for pointcloud chosen texture " << texture_source_name);
            return;
        }
        warn_count = 0;
    }

    int texture_width(0), texture_height(0);
    int num_colors(0);

    const rs2::vertex* vertex = pc.get_vertices();
    const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();

    rs2_intrinsics depth_intrin = pc.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pc.size());
    if (_ordered_pc)
    {
        _msg_pointcloud.width = depth_intrin.width;
        _msg_pointcloud.height = depth_intrin.height;
        _msg_pointcloud.is_dense = false;
    }

    vertex = pc.get_vertices();
    size_t valid_count(0);
    if (use_texture)
    {
        rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
        texture_width = texture_frame.get_width();
        texture_height = texture_frame.get_height();
        num_colors = texture_frame.get_bytes_per_pixel();
        uint8_t* color_data = (uint8_t*)texture_frame.get_data();
        std::string format_str;
        switch(texture_frame.get_profile().format())
        {
            case RS2_FORMAT_RGB8:
                format_str = "rgb";
                break;
            case RS2_FORMAT_Y8:
                format_str = "intensity";
                break;
            default:
                throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
        }
        _msg_pointcloud.point_step = addPointField(_msg_pointcloud, format_str.c_str(), 1, sensor_msgs::PointField::FLOAT32, _msg_pointcloud.point_step);
        _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
        _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(_msg_pointcloud, format_str);
        color_point = pc.get_texture_coordinates();

        float color_pixel[2];
        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
        {
            float i(color_point->u);
            float j(color_point->v);
            bool valid_color_pixel(i >= 0.f && i <=1.f && j >= 0.f && j <=1.f);
            bool valid_pixel(vertex->z > 0 && (valid_color_pixel || _allow_no_texture_points));
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                if (valid_color_pixel)
                {
                    color_pixel[0] = i * texture_width;
                    color_pixel[1] = j * texture_height;
                    int pixx = static_cast<int>(color_pixel[0]);
                    int pixy = static_cast<int>(color_pixel[1]);
                    int offset = (pixy * texture_width + pixx) * num_colors;
                    reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.
                }
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_color;
                ++valid_count;
            }
        }
    }
    else
    {
        std::string format_str = "intensity";
        _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
        _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");

        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++)
        {
            bool valid_pixel(vertex->z > 0);
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                ++iter_x; ++iter_y; ++iter_z;
                ++valid_count;
            }
        }
    }
    _msg_pointcloud.header.stamp = t;
    if (_align_depth) _msg_pointcloud.header.frame_id = _optical_frame_id[COLOR];
    else              _msg_pointcloud.header.frame_id = _optical_frame_id[DEPTH];
    if (!_ordered_pc)
    {
        _msg_pointcloud.width = valid_count;
        _msg_pointcloud.height = 1;
        _msg_pointcloud.is_dense = true;
        modifier.resize(valid_count);
    }
    _pointcloud_publisher.publish(_msg_pointcloud);
}


Extrinsics BaseRealSenseNode::rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const
{
    Extrinsics extrinsicsMsg;
    for (int i = 0; i < 9; ++i)
    {
        extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
        if (i < 3)
            extrinsicsMsg.translation[i] = extrinsics.translation[i];
    }

    extrinsicsMsg.header.frame_id = frame_id;
    return extrinsicsMsg;
}

rs2::stream_profile BaseRealSenseNode::getAProfile(const stream_index_pair& stream)
{
    const std::vector<rs2::stream_profile> profiles = _sensors[stream].get_stream_profiles();
    return *(std::find_if(profiles.begin(), profiles.end(),
                                            [&stream] (const rs2::stream_profile& profile) {
                                                return ((profile.stream_type() == stream.first) && (profile.stream_index() == stream.second));
                                            }));
}

IMUInfo BaseRealSenseNode::getImuInfo(const stream_index_pair& stream_index)
{
    IMUInfo info{};
    auto sp = _enabled_profiles[stream_index].front().as<rs2::motion_stream_profile>();
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
    info.frame_id = _optical_frame_id[stream_index];
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

void BaseRealSenseNode::publishFrame(rs2::frame f, const ros::Time& t,
                                     const stream_index_pair& stream,
                                     std::map<stream_index_pair, cv::Mat>& images,
                                     const std::map<stream_index_pair, ros::Publisher>& info_publishers,
                                     const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers,
                                     const bool is_publishMetadata,
                                     std::map<stream_index_pair, int>& seq,
                                     std::map<stream_index_pair, sensor_msgs::CameraInfo>& camera_info,
                                     const std::map<rs2_stream, std::string>& encoding,
                                     bool copy_data_from_frame)
{
    ROS_DEBUG("publishFrame(...)");
    unsigned int width = 0;
    unsigned int height = 0;
    auto bpp = 1;
    if (f.is<rs2::video_frame>())
    {
        auto image = f.as<rs2::video_frame>();
        width = image.get_width();
        height = image.get_height();
        bpp = image.get_bytes_per_pixel();
    }
    auto& image = images[stream];

    if (copy_data_from_frame)
    {
        if (images[stream].size() != cv::Size(width, height))
        {
            image.create(height, width, image.type());
        }
        image.data = (uint8_t*)f.get_data();
    }
    if (f.is<rs2::depth_frame>())
    {
        image = fix_depth_scale(image, _depth_scaled_image[stream]);
    }

    ++(seq[stream]);
    auto& info_publisher = info_publishers.at(stream);
    auto& image_publisher = image_publishers.at(stream);

    image_publisher.second->tick();
    if(0 != info_publisher.getNumSubscribers() ||
       0 != image_publisher.first.getNumSubscribers())
    {
        auto& cam_info = camera_info.at(stream);
        if (cam_info.width != width)
        {
            updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
        }
        cam_info.header.stamp = t;
        cam_info.header.seq = seq[stream];
        info_publisher.publish(cam_info);

        sensor_msgs::ImagePtr img;
        img = cv_bridge::CvImage(std_msgs::Header(), encoding.at(stream.first), image).toImageMsg();
        img->width = width;
        img->height = height;
        img->is_bigendian = false;
        img->step = width * bpp;
        img->header.frame_id = cam_info.header.frame_id;
        img->header.stamp = t;
        img->header.seq = seq[stream];

        image_publisher.first.publish(img);
        ROS_DEBUG("%s stream published", rs2_stream_to_string(f.get_profile().stream_type()));
    }
    if (is_publishMetadata)
    {
        auto& cam_info = camera_info.at(stream);
        publishMetadata(f, t, cam_info.header.frame_id);
    }
}

void BaseRealSenseNode::publishMetadata(rs2::frame f, const ros::Time& header_time, const std::string& frame_id)
{
    stream_index_pair stream = {f.get_profile().stream_type(), f.get_profile().stream_index()};    
    if (_metadata_publishers.find(stream) != _metadata_publishers.end())
    {
        auto& md_publisher = _metadata_publishers.at(stream);
        if (0 != md_publisher->getNumSubscribers())
        {
            realsense2_camera::Metadata msg;
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

bool BaseRealSenseNode::getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile)
{
    // Assuming that all D400 SKUs have depth sensor
    auto profiles = _enabled_profiles[stream_index];
    auto it = std::find_if(profiles.begin(), profiles.end(),
                            [&](const rs2::stream_profile& profile)
                            { return (profile.stream_type() == stream_index.first); });
    if (it == profiles.end())
        return false;

    profile =  *it;
    return true;
}

void BaseRealSenseNode::startMonitoring()
{
    for (rs2_option option : _monitor_options)
    {
        _temperature_nodes.push_back({option, std::make_shared<TemperatureDiagnostics>(rs2_option_to_string(option), _serial_no )});
    }

    int time_interval(1000);
    std::function<void()> func = [this, time_interval](){
        std::mutex mu;
        std::unique_lock<std::mutex> lock(mu);
        while(_is_running) {
            _cv_monitoring.wait_for(lock, std::chrono::milliseconds(time_interval), [&]{return !_is_running;});
            if (_is_running)
            {
                publish_temperature();
                publish_frequency_update();
            }
        }
    };
    _monitoring_t = std::make_shared<std::thread>(func);
}

void BaseRealSenseNode::publish_temperature()
{
    rs2::options sensor(_sensors[_base_stream]);
    for (OptionTemperatureDiag option_diag : _temperature_nodes)
    {
        rs2_option option(option_diag.first);
        if (sensor.supports(option))
        {
            try
            {
                auto option_value = sensor.get_option(option);
                option_diag.second->update(option_value);
            }
            catch(const std::exception& e)
            {
                ROS_DEBUG_STREAM("Failed checking for temperature." << std::endl << e.what());
            }
        }
    }
}

void BaseRealSenseNode::publish_frequency_update()
{
    std::chrono::milliseconds timespan(1);
    for (auto &image_publisher : _image_publishers)
    {
        image_publisher.second.second->update();
        std::this_thread::sleep_for(timespan);
    }
}

TemperatureDiagnostics::TemperatureDiagnostics(std::string name, std::string serial_no)
    {
        _updater.add(name, this, &TemperatureDiagnostics::diagnostics);
        _updater.setHardwareID(serial_no);
    }

void TemperatureDiagnostics::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
{
        status.summary(0, "OK");
        status.add("Index", _crnt_temp);
}

void BaseRealSenseNode::publishServices()
{
    _device_info_srv = std::make_shared<ros::ServiceServer>(_pnh.advertiseService("device_info", &BaseRealSenseNode::getDeviceInfo, this));
}

bool BaseRealSenseNode::getDeviceInfo(realsense2_camera::DeviceInfo::Request&,
                                      realsense2_camera::DeviceInfo::Response& res)
{
    res.device_name = _dev.supports(RS2_CAMERA_INFO_NAME) ? create_graph_resource_name(_dev.get_info(RS2_CAMERA_INFO_NAME)) : "";
    res.serial_number = _dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
    res.firmware_version = _dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION) ? _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) : "";
    res.usb_type_descriptor = _dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) ? _dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) : "";
    res.firmware_update_id = _dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) ? _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) : "";

    std::stringstream sensors_names;
    for(auto&& sensor : _dev_sensors)
    {
        sensors_names << create_graph_resource_name(sensor.get_info(RS2_CAMERA_INFO_NAME)) << ",";
    }

    res.sensors = sensors_names.str().substr(0, sensors_names.str().size()-1);
    return true;
}
