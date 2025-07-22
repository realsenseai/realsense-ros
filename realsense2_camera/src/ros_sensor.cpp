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

#include <ros_sensor.h>
#include <assert.h>

using namespace realsense2_camera;
using namespace rs2;

void RosSensor::setupErrorCallback()
{
    set_notifications_callback([&](const rs2::notification& n)
    {
        std::vector<std::string> error_strings({"RT IC2 Config error",
                                                "Left IC2 Config error"
                                                });
        ROS_WARN_STREAM("XXX Hardware Notification:" << n.get_description() << "," << n.get_timestamp() << "," << n.get_severity() << "," << n.get_category());            
        if (n.get_severity() >= RS2_LOG_SEVERITY_ERROR)
        {
            ROS_WARN_STREAM("Hardware Notification:" << n.get_description() << "," << n.get_timestamp() << "," << n.get_severity() << "," << n.get_category());
        }
        if (error_strings.end() != find_if(error_strings.begin(), error_strings.end(), [&n] (std::string err) 
                                    {return (n.get_description().find(err) != std::string::npos); }))
        {
            _hardware_reset_func();
        }
    });
}

RosSensor::RosSensor(rs2::sensor sensor,
    std::shared_ptr<Parameters> parameters, 
    std::function<void(rs2::frame)> frame_callback,
    std::function<void()> update_sensor_func,
    std::function<void()> hardware_reset_func, 
    std::shared_ptr<diagnostic_updater::Updater> diagnostics_updater,
    rclcpp::Logger logger,
    bool force_image_default_qos,
    bool is_rosbag_file):
    rs2::sensor(sensor),
    _logger(logger),
    _origin_frame_callback(frame_callback),
    _params(parameters, _logger),
    _update_sensor_func(update_sensor_func),
    _hardware_reset_func(hardware_reset_func),
    _diagnostics_updater(diagnostics_updater),
    _force_image_default_qos(force_image_default_qos)
{
    _frame_callback = [this](rs2::frame frame)
        {
            auto stream_type = frame.get_profile().stream_type();
            auto stream_index = frame.get_profile().stream_index();
            stream_index_pair sip{stream_type, stream_index};
            try
            {
                _origin_frame_callback(frame);
                if (_frequency_diagnostics.find(sip) != _frequency_diagnostics.end())
                    _frequency_diagnostics.at(sip).Tick();
            }
            catch(const std::exception& ex)
            {
                // don't tick the frequency diagnostics for this publisher
                ROS_ERROR_STREAM("An error has occurred during frame callback: " << ex.what());
            }
        };
    setParameters(is_rosbag_file);
}

RosSensor::~RosSensor()
{
    try
    {
        clearParameters();
        stop();
    }
    catch(...){} // Not allowed to throw from Dtor
}

void RosSensor::setParameters(bool is_rosbag_file)
{
    std::string module_name = create_graph_resource_name(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME)));

    // From FW version 5.14.x.x, if HDR is enabled, updating UVC controls like exposure, gain , etc are restricted.
    // So, during init of the node, forcefully disabling the HDR upfront and update it with new values.
    if((!is_rosbag_file) && supports(RS2_OPTION_HDR_ENABLED))
    {
        set_option(RS2_OPTION_HDR_ENABLED, false);
    }

    _params.registerDynamicOptions(*this, module_name);

    // for rosbag files, don't set hdr(sequence_id) / gain / exposure options
    // since these options can be changed only in real devices
    if(!is_rosbag_file)
        UpdateSequenceIdCallback();
    
    registerSensorParameters();
}

void RosSensor::UpdateSequenceIdCallback()
{
    // Function replaces the trivial parameter callback with one that 
    // also updates ros server about the gain and exposure of the selected sequence id.
    if (!supports(RS2_OPTION_SEQUENCE_ID))
        return;

    std::string module_name = create_graph_resource_name(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME)));
    std::string param_name = module_name + "." + create_graph_resource_name(rs2_option_to_string(RS2_OPTION_ENABLE_AUTO_EXPOSURE));

    bool user_set_enable_ae_value = _params.getParameters()->getParam<bool>(param_name);
    bool is_hdr_enabled = static_cast<bool>(get_option(RS2_OPTION_HDR_ENABLED));

    if (is_hdr_enabled && user_set_enable_ae_value)
    {
        bool is_ae_enabled = static_cast<bool>(get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE));

        // If AE got auto-disabled, update the Enable_Auto_Exposure ROS paramerter as well accordingly.
        if (!is_ae_enabled)
        {
            ROS_WARN_STREAM("Auto Exposure functionality is not supported when HDR is enabled. " << 
                                                "So, disabling the '" << param_name << "' param.");

            try
            {
                std::vector<std::function<void()> > funcs;
                funcs.push_back([this](){set_sensor_parameter_to_ros<bool>(RS2_OPTION_ENABLE_AUTO_EXPOSURE);});
                _params.getParameters()->pushUpdateFunctions(funcs);
            }
            catch(const std::exception& e)
            {
                ROS_WARN_STREAM("Failed to set parameter:" << param_name << " : " << e.what());
            }
        }
    }

    // Deleter to revert back the RS2_OPTION_HDR_ENABLED value at the end.
    auto deleter_to_revert_hdr = std::unique_ptr<bool, std::function<void(bool*)>>(&is_hdr_enabled,
                                                [&](bool* enable_back_hdr) {
                                                    if (enable_back_hdr && *enable_back_hdr)
                                                    {
                                                        set_option(RS2_OPTION_HDR_ENABLED, true);
                                                    }
                                                });

    if (is_hdr_enabled)
    {
        // From FW version 5.14.x.x, if HDR is enabled, updating UVC controls like exposure, gain , etc are restricted.
        // So, disable it before updating. It will be reverted back by the deleter 'deleter_to_revert_hdr'.
        set_option(RS2_OPTION_HDR_ENABLED, false);

        int original_seq_id = static_cast<int>(get_option(RS2_OPTION_SEQUENCE_ID));   // To Set back to default.

        // Read initialization parameters and set to sensor:
        std::vector<rs2_option> options{RS2_OPTION_EXPOSURE, RS2_OPTION_GAIN};
        unsigned int seq_size = get_option(RS2_OPTION_SEQUENCE_SIZE);
        for (unsigned int seq_id = 1; seq_id <= seq_size; seq_id++ )
        {
            set_option(RS2_OPTION_SEQUENCE_ID, seq_id);
            for (rs2_option& option : options)
            {
                std::stringstream param_name_str;
                param_name_str << module_name << "." << create_graph_resource_name(rs2_option_to_string(option)) << "." << seq_id;
                int option_value = get_option(option);
                int user_set_option_value = _params.getParameters()->readAndDeleteParam(param_name_str.str(), option_value);
                if (option_value != user_set_option_value)
                {
                    ROS_INFO_STREAM("Set " << rs2_option_to_string(option) << "." << seq_id << " to " << user_set_option_value);
                    set_option(option, user_set_option_value);
                }
            }
        }
        set_option(RS2_OPTION_SEQUENCE_ID, original_seq_id);   // Set back to default.
    }

    // Set callback to update ros parameters to gain and exposure matching the selected sequence_id:
    const std::string option_name(module_name + "." + create_graph_resource_name(rs2_option_to_string(RS2_OPTION_SEQUENCE_ID)));
    try
    {
        int option_value = static_cast<int>(get_option(RS2_OPTION_SEQUENCE_ID));
        _params.getParameters()->setParam<int>(option_name, option_value, 
            [this](const rclcpp::Parameter& parameter)
            {
                set_option(RS2_OPTION_SEQUENCE_ID, parameter.get_value<int>());
                std::vector<std::function<void()> > funcs;
                funcs.push_back([this](){set_sensor_parameter_to_ros<int>(RS2_OPTION_GAIN);});
                funcs.push_back([this](){set_sensor_parameter_to_ros<int>(RS2_OPTION_EXPOSURE);});
                _params.getParameters()->pushUpdateFunctions(funcs);
            });
    }
    catch(const rclcpp::exceptions::InvalidParameterValueException& e)
    {
        ROS_WARN_STREAM("Setting alternative callback: Failed to set parameter:" << option_name << " : " << e.what());
        return;
    }
}

template<class T> 
void RosSensor::set_sensor_parameter_to_ros(rs2_option option)
{
    std::string module_name = create_graph_resource_name(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME)));
    const std::string option_name(module_name + "." + create_graph_resource_name(rs2_option_to_string(option)));
    auto value = static_cast<T>(get_option(option));
    _params.getParameters()->setRosParamValue(option_name, &value);
}


void RosSensor::registerSensorParameters()
{
    std::vector<stream_profile> all_profiles = get_stream_profiles();
    const std::string module_name(create_graph_resource_name(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME))));

    std::shared_ptr<ProfilesManager> profile_manager = std::make_shared<VideoProfilesManager>(_params.getParameters(), module_name, _logger, _force_image_default_qos);
    profile_manager->registerProfileParameters(all_profiles, _update_sensor_func);
    if (profile_manager->isTypeExist())
    {
        _profile_managers.push_back(profile_manager);
        registerAutoExposureROIOptions();
    }
    profile_manager = std::make_shared<MotionProfilesManager>(_params.getParameters(), _logger);
    profile_manager->registerProfileParameters(all_profiles, _update_sensor_func);
    if (profile_manager->isTypeExist())
    {
        _profile_managers.push_back(profile_manager);
    }
}

bool RosSensor::start(const std::vector<stream_profile>& profiles)
{
    if (get_active_streams().size() > 0)
        return false;
    setupErrorCallback();
    rs2::sensor::open(profiles);

    for (auto& profile : profiles)
    ROS_INFO_STREAM("Open profile: " << ProfilesManager::profile_string(profile));

    rs2::sensor::start(_frame_callback);

    for (auto& profile : profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (_diagnostics_updater)
            _frequency_diagnostics.emplace(sip, FrequencyDiagnostics(STREAM_NAME(sip), profile.fps(), _diagnostics_updater));
    }
    return true;
}

void RosSensor::stop()
{
    if (get_active_streams().size() == 0)
        return;
    ROS_INFO_STREAM("Stop Sensor: " << rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME)));
    _frequency_diagnostics.clear();

    try
    {
        rs2::sensor::stop();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Exception: " << __FILE__ << ":" << __LINE__ << ":" << e.what());
    }
    ROS_INFO_STREAM("Close Sensor. ");
    try
    {
        close();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Exception: " << __FILE__ << ":" << __LINE__ << ":" << e.what());
    }
    ROS_INFO_STREAM("Close Sensor - Done. ");
}

rmw_qos_profile_t RosSensor::getQOS(const stream_index_pair& sip) const
{
    for(auto& profile_manager : _profile_managers)
    {
        if (profile_manager->hasSIP(sip))
        {
            return profile_manager->getQOS(sip);
        }
    }
    throw std::runtime_error("Given stream has no profile manager: " + std::string(rs2_stream_to_string(sip.first)) + "." + std::to_string(sip.second));
}

rmw_qos_profile_t RosSensor::getInfoQOS(const stream_index_pair& sip) const
{
    for(auto& profile_manager : _profile_managers)
    {
        if (profile_manager->hasSIP(sip))
        {
            return profile_manager->getInfoQOS(sip);
        }
    }
    throw std::runtime_error("Given stream has no profile manager: " + std::string(rs2_stream_to_string(sip.first)) + "." + std::to_string(sip.second));
}

bool profiles_equal(const rs2::stream_profile& a, const rs2::stream_profile& b)
{
    if (a.is<rs2::video_stream_profile>() && b.is<rs2::video_stream_profile>())
    {
        auto va = a.as<rs2::video_stream_profile>();
        auto vb = b.as<rs2::video_stream_profile>();
        return (va == vb && va.width() == vb.width() && va.height() == vb.height());
    }
    return ((rs2::stream_profile)a==(rs2::stream_profile)b);
}

bool is_profiles_in_profiles(const std::vector<stream_profile>& sub_profiles, const std::vector<stream_profile>& all_profiles)
{
    for (auto& a : sub_profiles)
    {
        bool found_profile(false);
        for (auto& b : all_profiles)
        {
            if (profiles_equal(a, b))
            {
                found_profile = true;
                break;
            }
        }
        if (!found_profile)
        {
            return false;
        }
    }
    return true;
}

bool compare_profiles_lists(const std::vector<stream_profile>& active_profiles, const std::vector<stream_profile>& wanted_profiles)
{
    return (is_profiles_in_profiles(active_profiles, wanted_profiles) && is_profiles_in_profiles(wanted_profiles, active_profiles));
}

bool RosSensor::getUpdatedProfiles(std::vector<stream_profile>& wanted_profiles)
{
    wanted_profiles.clear();
    std::vector<stream_profile> active_profiles = get_active_streams();
    for (auto profile_manager : _profile_managers)
    {
        profile_manager->addWantedProfiles(wanted_profiles);        
    }

    ROS_DEBUG_STREAM(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME)) << ":" << "active_profiles.size() = " << active_profiles.size());
    for (auto& profile : active_profiles)
    {
        ROS_DEBUG_STREAM("Sensor profile: " << ProfilesManager::profile_string(profile));
    }

    ROS_DEBUG_STREAM(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME)) << ":" << "wanted_profiles");
    for (auto& profile : wanted_profiles)
    {
        ROS_DEBUG_STREAM("Sensor profile: " << ProfilesManager::profile_string(profile));
    }
    if (compare_profiles_lists(active_profiles, wanted_profiles))
    {
        return false;
    }
    return true;
}

void RosSensor::set_sensor_auto_exposure_roi()
{
    try
    {
        rs2_stream stream_type = RS2_STREAM_ANY;
        if (this->rs2::sensor::is<rs2::depth_sensor>())
        {
            stream_type = RS2_STREAM_DEPTH;
        }
        else if (this->rs2::sensor::is<rs2::color_sensor>())
        {
            stream_type = RS2_STREAM_COLOR;
        }
        assert(stream_type != RS2_STREAM_ANY);

        int width = std::dynamic_pointer_cast<VideoProfilesManager>(_profile_managers[0])->getWidth(stream_type);
        int height = std::dynamic_pointer_cast<VideoProfilesManager>(_profile_managers[0])->getHeight(stream_type);

        bool update_roi_range(false);
        if (_auto_exposure_roi.max_x > width)
        {
            _params.getParameters()->setParamValue(_auto_exposure_roi.max_x, width-1);
            update_roi_range = true;
        }
        if (_auto_exposure_roi.max_y > height)
        {
            _params.getParameters()->setParamValue(_auto_exposure_roi.max_y, height-1);
            update_roi_range = true;
        }
        if (update_roi_range)
        {
            registerAutoExposureROIOptions();
        }
        this->as<rs2::roi_sensor>().set_region_of_interest(_auto_exposure_roi);
    }
    catch(const std::runtime_error& e)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << e.what());
    }
}

void RosSensor::registerAutoExposureROIOptions()
{
    std::string module_base_name(rs2_to_ros(get_info(RS2_CAMERA_INFO_NAME)));

    if (this->rs2::sensor::is<rs2::roi_sensor>())
    {
        rs2_stream stream_type = RS2_STREAM_ANY;
        if (this->rs2::sensor::is<rs2::depth_sensor>())
        {
            stream_type = RS2_STREAM_DEPTH;
        }
        else if (this->rs2::sensor::is<rs2::color_sensor>())
        {
            stream_type = RS2_STREAM_COLOR;
        }
        assert(stream_type != RS2_STREAM_ANY);
        
        int width = std::dynamic_pointer_cast<VideoProfilesManager>(_profile_managers[0])->getWidth(stream_type);
        int height = std::dynamic_pointer_cast<VideoProfilesManager>(_profile_managers[0])->getHeight(stream_type);

        int max_x(width-1);
        int max_y(height-1);

        std::string module_name = create_graph_resource_name(module_base_name) +".auto_exposure_roi";
        _auto_exposure_roi = {0, 0, max_x, max_y};

        ROS_DEBUG_STREAM("Publish roi for " << module_name);
        std::string param_name(module_name + ".left");
        _params.getParameters()->setParamT(param_name, _auto_exposure_roi.min_x, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _parameters_names.push_back(param_name);

        param_name = std::string(module_name + ".right");
        _params.getParameters()->setParamT(module_name + ".right", _auto_exposure_roi.max_x, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _parameters_names.push_back(param_name);

        param_name = std::string(module_name + ".top");
        _params.getParameters()->setParamT(module_name + ".top", _auto_exposure_roi.min_y, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _parameters_names.push_back(param_name);

        param_name = std::string(module_name + ".bottom");
        _params.getParameters()->setParamT(module_name + ".bottom", _auto_exposure_roi.max_y, [this](const rclcpp::Parameter&){set_sensor_auto_exposure_roi();});
        _parameters_names.push_back(param_name);
    }
}

void RosSensor::clearParameters()
{
    for (auto profile_manager : _profile_managers)
    {
        profile_manager->clearParameters();
    }

    _params.clearParameters();

    while ( !_parameters_names.empty() )
    {
        auto name = _parameters_names.back();
        _params.getParameters()->removeParam(name);
        _parameters_names.pop_back();        
    }
}
