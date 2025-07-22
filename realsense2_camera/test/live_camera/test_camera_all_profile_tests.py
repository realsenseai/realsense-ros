# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import sys
import itertools


import pytest
import rclpy

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2

import numpy as np

sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_parameters
from pytest_rs_utils import delayed_launch_descr_with_parameters
from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_node_heirarchy
import pytest_live_camera_utils
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from pytest_live_camera_utils import debug_print
def check_if_skip_test(profile, format):
    '''
    if profile == 'Color':
        if "BGRA8" == format:
            return True
        if "RGBA8" == format:
            return True
        if "Y8" == format:
            return True
    elif profile == 'Depth':
        if "Z16" == format:
            return True
    el
    if profile == 'Infrared':
        if "Y8" == format:
            return True
        if "Y16" == format:
            return True
        if "BGRA8" == format:
            return True
        if "RGBA8" == format:
            return True
        if "Y10BPACK" == format:
            return True
        if "UYVY" == format:
            return True
        if "BGR8" == format:
            return True
        if "RGB8" == format:
            return True
        if "RAW10" == format:
            return True
    elif profile == 'Infrared1':
        if "Y8" ==format:
            return True
        if "Y16" ==format:
            return True
        if "Y10BPACK"  == format:
            return True
        if "UYVY" ==format:
            return True
        if "BGR8" ==format:
            return True
        if "RGB8" ==format:
            return True
        if "RAW10" ==format:
            return True
    if profile == 'Infrared2':                    
        if "Y8" == format:
            return True
        if "Y16" == format:
            return True
        if "Y10BPACK" == format:
            return True
        if "UYVY" == format:
            return True
        if "BGR8" == format:
            return True
        if "RGB8" == format:
            return True
        if "RAW10" == format:
            return True
    '''
    if profile == 'Infrared':
        if "Y8" == format:
            return True
        if "Y16" == format:
            return True
    if profile == 'Infrared1':
        if "Y8" ==format:
            return True
        if "Y16" ==format:
            return True
    if profile == 'Infrared2':                    
        if "Y8" == format:
            return True
        if "Y16" == format:
            return True
    return False


test_params_all_profiles_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    }
test_params_all_profiles_d415 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    }
test_params_all_profiles_d435i = {
    'camera_name': 'D435I',
    'device_type': 'D435I',
    }


'''
This test was implemented as a template to set the parameters and run the test.
This directory is not added to the CMakeLists.txt so as to avoid the colcon failure in the
machines that don't have the D455 connected.
1. Only a subset of parameter types are implemented in py_rs_utils, it has to be extended for others
2. After setting the param, rclpy.spin_once may be needed.Test passes even without this though.
'''
@pytest.mark.skipif (os.getenv('RS_ROS_REGRESSION', "not found") == "not found",reason="Regression is not enabled, define RS_ROS_REGRESSION")
@pytest.mark.parametrize("launch_descr_with_parameters", [
    pytest.param(test_params_all_profiles_d455, marks=pytest.mark.d455),
    pytest.param(test_params_all_profiles_d415, marks=pytest.mark.d415),
    pytest.param(test_params_all_profiles_d435i, marks=pytest.mark.d435i),]
    ,indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestLiveCamera_Change_Resolution(pytest_rs_utils.RsTestBaseClass):
    def test_LiveCamera_Change_Resolution(self,launch_descr_with_parameters):
        skipped_tests = []
        failed_tests = []
        num_passed = 0
        num_failed = 0
        params = launch_descr_with_parameters[1]
        if pytest_live_camera_utils.check_if_camera_connected(params['device_type']) == False:
            print("Device not found? : " + params['device_type'])
            assert False
            return
        themes = [{'topic':get_node_heirarchy(params)+'/color/image_raw', 'msg_type':msg_Image,'expected_data_chunks':1}]
        config = pytest_live_camera_utils.get_profile_config(get_node_heirarchy(params))
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            self.spin_for_time(wait_time=1.0)
            cap = pytest_live_camera_utils.get_camera_capabilities(params['device_type'])
            if cap == None:
                debug_print("Device not found? : " + params['device_type'])
                return
            self.create_service_client_ifs(get_node_heirarchy(params))
            config["Color"]["default_profile1"],config["Color"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["color_profile"], "Color")
            config["Depth"]["default_profile1"],config["Depth"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["depth_profile"], "Depth")
            config["Infrared"]["default_profile1"],config["Infrared"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["depth_profile"], "Infrared")
            config["Infrared1"]["default_profile1"],config["Infrared1"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["depth_profile"], "Infrared1")
            config["Infrared2"]["default_profile1"],config["Infrared2"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["depth_profile"], "Infrared2")
            for key in cap["color_profile"]:
                profile_type = key[0]
                profile = key[1]
                format = key[2]
                if check_if_skip_test(profile_type, format):
                    skipped_tests.append(" ".join(key))
                    continue
                print("Testing " + " ".join(key))
                themes[0]['topic'] = config[profile_type]['topic']
                themes[0]['width'] = int(profile.split('x')[0])
                themes[0]['height'] = int(profile.split('x')[1])
                if themes[0]['width'] == int(config[profile_type]["default_profile2"].split('x')[0]):
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile1"])
                else:
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile2"])
                self.set_bool_param(config[profile_type]["param"], True)
                self.disable_all_params()
                self.spin_for_time(wait_time=0.2)
                self.set_string_param(config[profile_type]["profile"], profile)
                self.set_string_param(config[profile_type]["format"], format)
                self.set_bool_param(config[profile_type]["param"], True)
                try:
                    ret = self.run_test(themes)
                    assert ret[0], ret[1]
                    assert self.process_data(themes), " ".join(key) + " failed"
                    num_passed += 1
                except Exception as e:
                    exc_type, exc_obj, exc_tb = sys.exc_info()
                    fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                    print("Test failed")
                    print(e)
                    print(exc_type, fname, exc_tb.tb_lineno)
                    num_failed += 1
                    failed_tests.append(" ".join(key))
            debug_print("Color tests completed")
            for key in cap["depth_profile"]:
                profile_type = key[0]
                profile = key[1]
                format = key[2]
                if check_if_skip_test(profile_type, format):
                    skipped_tests.append(" ".join(key))
                    continue
                print("Testing " + " ".join(key))
                
                themes[0]['topic'] = config[profile_type]['topic']
                themes[0]['width'] = int(profile.split('x')[0])
                themes[0]['height'] = int(profile.split('x')[1])
                if themes[0]['width'] == int(config[profile_type]["default_profile2"].split('x')[0]):
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile1"])
                else:
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile2"])
                self.set_bool_param(config[profile_type]["param"], True)


                self.disable_all_params()
                self.spin_for_time(wait_time=0.2)
                self.set_string_param(config[profile_type]["profile"], profile)
                self.set_string_param(config[profile_type]["format"], format)
                self.set_bool_param(config[profile_type]["param"], True)
                try:
                    ret = self.run_test(themes)
                    assert ret[0], ret[1]
                    assert self.process_data(themes), " ".join(key) + " failed"
                    num_passed += 1
                except Exception as e:
                    print("Test failed")
                    print(e)
                    num_failed += 1
                    failed_tests.append(" ".join(key))
            debug_print("Depth tests completed")
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()
            print("Tests passed " + str(num_passed))
            print("Tests skipped " + str(len(skipped_tests)))
            if len(skipped_tests):
                debug_print("\nSkipped tests:" + params['device_type'])
                debug_print("\n".join(skipped_tests))
            print("Tests failed " + str(num_failed))
            if len(failed_tests):
                print("\nFailed tests:" + params['device_type'])
                print("\n".join(failed_tests))

    def disable_all_params(self):
        self.set_bool_param('enable_color', False)
        self.set_bool_param('enable_depth', False)
        self.set_bool_param('enable_infra', False)
        self.set_bool_param('enable_infra1', False)
        self.set_bool_param('enable_infra2', False)

