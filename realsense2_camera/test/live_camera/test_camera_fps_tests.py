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
import pytest_live_camera_utils
from pytest_rs_utils import launch_descr_with_parameters

from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_node_heirarchy

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters

test_params_test_fps_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',

    }
test_params_test_fps_d415 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    }
test_profiles ={
    'D455':{
    'depth_test_profiles':['640x480x5','640x480x15', '640x480x30', '640x480x90'],
    'color_test_profiles':['640x480x5','640x480x15', '640x480x30', '1280x720x30']
    },
    'D415':{
    'depth_test_profiles':['640x480x6','640x480x15', '640x480x30', '640x480x90'],
    'color_test_profiles':['640x480x6','640x480x15', '640x480x30', '1280x720x30']
    }
}
'''
The test was implemented to check the fps of Depth and Color frames. The RosTopicHz had to be
modified to make it work, see py_rs_utils for more details.
To check the fps, a value 'expected_fps_in_hz' has to be added to the corresponding theme
'''
@pytest.mark.parametrize("launch_descr_with_parameters", [    
    pytest.param(test_params_test_fps_d455, marks=pytest.mark.d455),
    pytest.param(test_params_test_fps_d415, marks=pytest.mark.d415),
    #pytest.param(test_params_test_fps_d435, marks=pytest.mark.d435),
    ],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestCamera_TestFPS(pytest_rs_utils.RsTestBaseClass):
    def test_camera_test_fps(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        if pytest_live_camera_utils.check_if_camera_connected(params['device_type']) == False:
            print("Device not found? : " + params['device_type'])
            assert False
            return
        try:
            ''' 
            initialize, run and check the data 
            '''
            print("Starting camera test...")
            self.init_test("RsTest"+params['camera_name'])
            self.wait_for_node(params['camera_name'])
            self.create_param_ifs(get_node_heirarchy(params))
            #assert self.set_bool_param('enable_color', False)
            
            themes = [
            {'topic':get_node_heirarchy(params)+'/depth/image_rect_raw',
            'msg_type':msg_Image,
            'expected_data_chunks':100,
            }
            ]
            profiles = test_profiles[params['camera_name']]['depth_test_profiles']
            self.spin_for_time(wait_time=0.5)
            assert self.set_bool_param('enable_color', False)
            self.spin_for_time(wait_time=0.5)

            for profile in profiles:
                print("Testing profile: ", profile)
                themes[0]['expected_fps_in_hz']  = float(profile.split('x')[2])
                assert self.set_string_param('depth_module.depth_profile', profile)
                self.spin_for_time(wait_time=0.5)
                assert self.set_bool_param('enable_depth', True)
                self.spin_for_time(wait_time=0.5)
                ret = self.run_test(themes, timeout=25.0)
                assert ret[0], ret[1]
                assert self.process_data(themes)

            themes = [
            {'topic':get_node_heirarchy(params)+'/color/image_raw',
            'msg_type':msg_Image,
            'expected_data_chunks':100,
            }
            ]
            assert self.set_bool_param('enable_depth', False)
            self.spin_for_time(wait_time=0.5)
            profiles = test_profiles[params['camera_name']]['color_test_profiles']
            for profile in profiles:
                print("Testing profile: ", profile)
                themes[0]['expected_fps_in_hz']  = float(profile.split('x')[2])
                assert self.set_string_param('rgb_camera.color_profile', profile)
                self.spin_for_time(wait_time=0.5)
                assert self.set_bool_param('enable_color', True)
                self.spin_for_time(wait_time=0.5)
                ret = self.run_test(themes, timeout=25.0)
                assert ret[0], ret[1]
                assert self.process_data(themes)

        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()


