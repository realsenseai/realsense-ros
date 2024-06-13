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
#include <sensor_params.h>

#define STREAM_NAME(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << create_graph_resource_name(ros_stream_to_string(sip.first)) << ((sip.second>0) ? std::to_string(sip.second) : ""))).str()

using namespace rs2;
namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    class ProfilesManager
    {
        public:
            ProfilesManager(std::shared_ptr<Parameters> parameters, rclcpp::Logger logger);
            virtual bool isWantedProfile(const rs2::stream_profile& profile) = 0;
            virtual void registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func) = 0;
            bool isTypeExist();
            static std::string profile_string(const rs2::stream_profile& profile);
            
            void registerSensorQOSParam(std::string template_name, 
                                        std::set<stream_index_pair> unique_sips, 
                                        std::map<stream_index_pair, std::shared_ptr<std::string> >& params, 
                                        std::string value);

            template<class T>
            void registerSensorUpdateParam(std::string template_name, 
                                           std::set<stream_index_pair> unique_sips, 
                                           std::map<stream_index_pair, std::shared_ptr<T> >& params, 
                                           T value, 
                                           std::function<void()> update_sensor_func);
            void addWantedProfiles(std::vector<rs2::stream_profile>& wanted_profiles);
            void clearParameters();
            bool hasSIP(const stream_index_pair& sip) const;
            rmw_qos_profile_t getQOS(const stream_index_pair& sip) const;
            rmw_qos_profile_t getInfoQOS(const stream_index_pair& sip) const;

        protected:
            std::map<stream_index_pair, rs2::stream_profile> getDefaultProfiles();

        protected:
            rclcpp::Logger _logger;
            SensorParams _params;
            std::map<stream_index_pair, std::shared_ptr<bool>> _enabled_profiles;
            std::map<stream_index_pair, std::shared_ptr<std::string>> _profiles_image_qos_str, _profiles_info_qos_str;
            std::vector<rs2::stream_profile> _all_profiles;
            std::vector<std::string> _parameters_names;
    };

    class VideoProfilesManager : public ProfilesManager
    {
        public:
            VideoProfilesManager(std::shared_ptr<Parameters> parameters, const std::string& module_name, rclcpp::Logger logger, bool force_image_default_qos = false);
            bool isWantedProfile(const rs2::stream_profile& profile) override;
            void registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func) override;
            int getHeight(rs2_stream stream_type) {return _height[stream_type];};
            int getWidth(rs2_stream stream_type) {return _width[stream_type];};
            int getFPS(rs2_stream stream_type) {return _fps[stream_type];};

        private:
            bool isSameProfileValues(const rs2::stream_profile& profile, const int width, const int height, const int fps, const rs2_format format);
            rs2::stream_profile validateAndGetSuitableProfile(rs2_stream stream_type, rs2::stream_profile given_profile);
            void registerVideoSensorProfileFormat(stream_index_pair sip);
            void registerVideoSensorParams(std::set<stream_index_pair> sips);
            std::string get_profiles_descriptions(rs2_stream stream_type);
            std::string getProfileFormatsDescriptions(stream_index_pair sip);

        private:
            std::string _module_name;
            std::map<stream_index_pair, rs2_format>  _formats;
            std::map<rs2_stream, int> _fps, _width, _height;
            bool _force_image_default_qos;
    };

    class MotionProfilesManager : public ProfilesManager
    {
        public:
            using ProfilesManager::ProfilesManager;
            bool isWantedProfile(const rs2::stream_profile& profile) override;
            void registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func) override;

        private:
            void registerFPSParams();
            bool isSameProfileValues(const rs2::stream_profile& profile, const rs2_stream stype, const int fps);
            std::map<stream_index_pair, std::vector<int>> getAvailableFPSValues();

        protected:
            std::map<stream_index_pair, std::shared_ptr<int> > _fps;
    };
}
