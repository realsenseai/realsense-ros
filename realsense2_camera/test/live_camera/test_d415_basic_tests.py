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
from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_node_heirarchy
import pytest_live_camera_utils

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters

test_params_depth_avg_1 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    }
'''
This test was implemented as a template to set the parameters and run the test.
This directory is not added to the CMakeLists.txt so as to avoid the colcon failure in the
machines that don't have the D415 connected.
1. Only a subset of parameter types are implemented in py_rs_utils, it has to be extended for others
2. After setting the param, rclpy.spin_once may be needed.Test passes even without this though.
'''
@pytest.mark.d415
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_depth_avg_1],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestD415_Change_Resolution(pytest_rs_utils.RsTestBaseClass):
    def test_D415_Change_Resolution(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        if pytest_live_camera_utils.check_if_camera_connected(params['device_type']) == False:
            print("Device not found? : " + params['device_type'])
            assert False
            return
        failed_tests = []
        num_passed = 0

        num_failed = 0
        themes = [{'topic':get_node_heirarchy(params)+'/color/image_raw', 'msg_type':msg_Image,'expected_data_chunks':1}]
        config = pytest_live_camera_utils.get_profile_config(get_node_heirarchy(params))
        config["Color"]["default_profile1"] = "640x480x6"
        config["Color"]["default_profile2"] = "1280x720x6" 
        config["Depth"]["default_profile1"] = "640x480x6"
        config["Depth"]["default_profile2"] = "1280x720x6" 
        config["Infrared"]["default_profile1"] = "640x480x6"
        config["Infrared"]["default_profile2"] = "1280x720x6" 
        config["Infrared1"]["default_profile1"] = "640x480x6"
        config["Infrared1"]["default_profile2"] = "1280x720x6" 
        config["Infrared2"]["default_profile1"] = "640x480x6"
        config["Infrared2"]["default_profile2"] = "1280x720x6" 

        cap = [
            ['Infrared', '848x100x100', 'BGRA8'],
            ['Infrared', '848x480x60', 'RGBA8'],
            ['Infrared', '640x480x60', 'RGBA8'],
            ['Infrared', '640x480x60', 'BGR8'],
            ['Infrared', '640x360x60', 'BGRA8'],
            ['Infrared', '640x360x60', 'BGR8'],
            ['Infrared', '640x360x30', 'UYVY'],
            ['Infrared', '480x270x60', 'BGRA8'],
            ['Infrared', '480x270x60', 'RGB8'],
            ['Infrared', '424x240x60', 'BGRA8'],
            ['Infrared1', '848x100x100', 'Y8'],
            ['Infrared1', '848x480x6', 'Y8'],
            ['Infrared1', '1920x1080x25', 'Y16'],
            ['Infrared2', '848x100x100', 'Y8'],
            ['Infrared2', '848x480x6', 'Y8'],
            ['Infrared2', '1920x1080x25', 'Y16'],
        ]

        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            self.spin_for_time(wait_time=1.0)
            self.create_param_ifs(get_node_heirarchy(params))
            self.spin_for_time(wait_time=1.0)

            for key in cap:
                profile_type = key[0]
                profile = key[1]
                format = key[2]
                print("Testing " + " ".join(key))
                themes[0]['topic'] = config[profile_type]['topic']
                themes[0]['width'] = int(profile.split('x')[0])
                themes[0]['height'] = int(profile.split('x')[1])
                #'''
                self.disable_all_streams()
                if themes[0]['width'] == int(config[profile_type]["default_profile2"].split('x')[0]):
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile1"])
                else:
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile2"])
                self.set_bool_param(config[profile_type]["param"], True)
                self.spin_for_time(wait_time=1.0)

                self.set_string_param(config[profile_type]["profile"], profile)
                self.spin_for_time(wait_time=1.0)
                self.set_string_param(config[profile_type]["format"], format)
                self.spin_for_time(wait_time=1.0)
                self.set_bool_param(config[profile_type]["param"], True)
                self.spin_for_time(wait_time=1.0)
                try:
                    ret = self.run_test(themes, timeout=10.0)
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
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()
            if num_failed != 0:
                print("Failed tests:")
                print("\n".join(failed_tests))
                assert False, " Tests failed"


    def disable_all_streams(self):
        
        self.set_bool_param('enable_color', False)
        self.spin_for_time(wait_time=1.0)
        self.set_bool_param('enable_depth', False)
        self.spin_for_time(wait_time=1.0)
        self.set_bool_param('enable_infra', False)
        self.spin_for_time(wait_time=1.0)
        self.set_bool_param('enable_infra1', False)
        self.spin_for_time(wait_time=1.0)
        self.set_bool_param('enable_infra2', False)
        self.spin_for_time(wait_time=1.0)
        
        pass

