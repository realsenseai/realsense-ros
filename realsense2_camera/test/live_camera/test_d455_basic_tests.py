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

test_params_depth_avg_1 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    }
'''
This test was implemented as a template to set the parameters and run the test.
This directory is not added to the CMakeLists.txt so as to avoid the colcon failure in the
machines that don't have the D455 connected.
1. Only a subset of parameter types are implemented in py_rs_utils, it has to be extended for others
2. After setting the param, rclpy.spin_once may be needed.Test passes even without this though.
'''
@pytest.mark.d455
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_depth_avg_1],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestD455_Change_Resolution(pytest_rs_utils.RsTestBaseClass):
    def test_D455_Change_Resolution(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        if pytest_live_camera_utils.check_if_camera_connected(params['device_type']) == False:
            print("Device not found? : " + params['device_type'])
            assert False
            return

        themes = [
        {'topic':get_node_heirarchy(params)+'/color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'width':640,
         'height':480,
         #'data':data
        }
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            print("Starting camera test...")
            self.init_test("RsTest"+params['camera_name'])
            self.wait_for_node(params['camera_name'])
            self.create_service_client_ifs(get_node_heirarchy(params))
            self.spin_for_time(0.5)
            assert self.set_bool_param('enable_color', False)
            self.spin_for_time(0.5)
            assert self.set_string_param('rgb_camera.color_profile', '640x480x30')
            self.spin_for_time(0.5)
            assert self.set_bool_param('enable_color', True)
            self.spin_for_time(0.5)
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
            self.set_string_param('rgb_camera.color_profile', '1280x800x5')
            self.set_bool_param('enable_color', True)
            themes[0]['width'] = 1280
            themes[0]['height'] = 800

            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()


test_params_seq_id_update = {
    'camera_name': 'D455',
    'device_type': 'D455',
    'exposure_1' : 2000,
    'gain_1'     : 20,
    'exposure_2' : 3000,
    'gain_2'     : 30,
    }
'''
This test sets the sequence ID param and validates the corresponding Exposure & Gain values.
'''
@pytest.mark.d455
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_seq_id_update],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class Test_D455_Seq_ID_Update(pytest_rs_utils.RsTestBaseClass):
    def test_D455_Seq_ID_update(self,launch_descr_with_parameters):
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
            self.create_service_client_ifs(get_node_heirarchy(params))

            assert self.set_bool_param('depth_module.hdr_enabled', False)

            assert self.set_integer_param('depth_module.sequence_id', 1)
            assert self.set_integer_param('depth_module.exposure', params['exposure_1'])
            assert self.set_integer_param('depth_module.gain', params['gain_1'])

            assert self.set_integer_param('depth_module.sequence_id', 2)
            assert self.set_integer_param('depth_module.exposure', params['exposure_2'])
            assert self.set_integer_param('depth_module.gain', params['gain_2'])

            assert self.set_integer_param('depth_module.sequence_id', 1)
            assert self.get_integer_param('depth_module.exposure') == params['exposure_1']
            assert self.get_integer_param('depth_module.gain') == params['gain_1']

            assert self.set_integer_param('depth_module.sequence_id', 2)
            assert self.get_integer_param('depth_module.exposure') == params['exposure_2']
            assert self.get_integer_param('depth_module.gain') == params['gain_2']

        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()



test_params_reset_device = {
    'camera_name': 'D455',
    'device_type': 'D455',
    'rgb_camera.color_profile': '640x480x30',
    }
'''
This test was implemented as a template to set the parameters and run the test.
This directory is not added to the CMakeLists.txt so as to avoid the colcon failure in the
machines that don't have the D455 connected.
1. Only a subset of parameter types are implemented in py_rs_utils, it has to be extended for others
2. After setting the param, rclpy.spin_once may be needed.Test passes even without this though.
'''
@pytest.mark.d455
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_reset_device],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestD455_reset_device(pytest_rs_utils.RsTestBaseClass):
    def test_D455_Reset_Device(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        if pytest_live_camera_utils.check_if_camera_connected(params['device_type']) == False:
            print("Device not found? : " + params['device_type'])
            assert False
            return

        themes = [
        {'topic':get_node_heirarchy(params)+'/color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'width':640,
         'height':480,
         #'data':data
        }
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            print("Starting camera test...")
            self.init_test("RsTest"+params['camera_name'])
            self.wait_for_node(params['camera_name'])
            self.create_service_client_ifs(get_node_heirarchy(params))
            self.spin_for_time(0.5)
            assert self.set_bool_param('enable_color', False)
            self.spin_for_time(0.5)
            assert self.set_string_param('rgb_camera.color_profile', '640x480x30')
            self.spin_for_time(0.5)
            assert self.set_bool_param('enable_color', True)
            self.spin_for_time(0.5)
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
            self.set_string_param('rgb_camera.color_profile', '1280x800x5')
            self.set_bool_param('enable_color', True)
            themes[0]['width'] = 1280
            themes[0]['height'] = 800

            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)

            self.reset_device()

            themes[0]['width'] = int(params['rgb_camera.color_profile'].split('x')[0])
            themes[0]['height'] = int(params['rgb_camera.color_profile'].split('x')[1])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)

        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()
