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
#include <map>
#include "ros_node_base.h"
#include <ros_utils.h>
#include <sensor_params.h>
#include <profile_manager.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>

namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    class FrequencyDiagnostics
    {
    public:
    FrequencyDiagnostics(std::string name, int expected_frequency, std::shared_ptr<diagnostic_updater::Updater> updater):
            _name(name),
            _min_freq(expected_frequency), _max_freq(expected_frequency),
            _freq_status_param(&_min_freq, &_max_freq, 0.1, 10),
            _freq_status(_freq_status_param, _name),
            _p_updater(updater)
            {
                _p_updater->add(_freq_status);
            };

    FrequencyDiagnostics (const FrequencyDiagnostics& other):
            _name(other._name),
            _min_freq(other._min_freq),
            _max_freq(other._max_freq),
            _freq_status_param(&_min_freq, &_max_freq, 0.1, 10),
            _freq_status(_freq_status_param, _name),
            _p_updater(other._p_updater)
            {
                _p_updater->add(_freq_status);
            };
    ~FrequencyDiagnostics()
    {
        _p_updater->removeByName(_name);    
    }

    void Tick()
    {
        _freq_status.tick();    
    }
    
    private:
        std::string _name;
        double _min_freq, _max_freq;
        diagnostic_updater::FrequencyStatusParam _freq_status_param;
        diagnostic_updater::FrequencyStatus _freq_status;
        std::shared_ptr<diagnostic_updater::Updater> _p_updater;
    };

    class RosSensor : public rs2::sensor
    {
        
        public:
            RosSensor(rs2::sensor sensor,
                      std::shared_ptr<Parameters> parameters, 
                      std::function<void(rs2::frame)> frame_callback,
                      std::function<void()> update_sensor_func,
                      std::function<void()> hardware_reset_func, 
                      std::shared_ptr<diagnostic_updater::Updater> diagnostics_updater,
                      rclcpp::Logger logger,
                      bool force_image_default_qos = false,
                      bool is_rosbag_file = false);
            ~RosSensor();
            void registerSensorParameters();
            bool getUpdatedProfiles(std::vector<rs2::stream_profile>& wanted_profiles);
            virtual bool start(const std::vector<rs2::stream_profile>& profiles);
            void stop();
            rmw_qos_profile_t getQOS(const stream_index_pair& sip) const;
            rmw_qos_profile_t getInfoQOS(const stream_index_pair& sip) const;

            template<class T> 
            bool is() const
            {
                return rs2::sensor::is<T>();
            }

        private:
            void setupErrorCallback();
            void setParameters(bool is_rosbag_file = false);
            void clearParameters();
            void set_sensor_auto_exposure_roi();
            void registerAutoExposureROIOptions();
            void UpdateSequenceIdCallback();
            template<class T> 
            void set_sensor_parameter_to_ros(rs2_option option);

        private:
            rclcpp::Logger _logger;
            std::function<void(rs2::frame)> _origin_frame_callback;
            std::function<void(rs2::frame)> _frame_callback;
            SensorParams _params;
            std::function<void()> _update_sensor_func, _hardware_reset_func;
            std::vector<std::shared_ptr<ProfilesManager> > _profile_managers;
            rs2::region_of_interest _auto_exposure_roi;
            std::vector<std::string> _parameters_names;
            std::shared_ptr<diagnostic_updater::Updater> _diagnostics_updater;
            std::map<stream_index_pair, FrequencyDiagnostics> _frequency_diagnostics;
            bool _force_image_default_qos;
    };
}
