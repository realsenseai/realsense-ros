# Copyright 2023 RealSense, Inc. All Rights Reserved.
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

import pytest

from realsense2_camera_msgs.msg import Metadata as msg_Metadata
from sensor_msgs.msg import Image as msg_Image


sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_parameters, get_node_heirarchy

import pytest_live_camera_utils

test_params_test_default_run_mode = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    'initial_reset': 'true',
    'safety_camera.safety_mode': 0,
    'depth_module.exposure': 1000,
    'depth_module.enable_auto_exposure': 'false',
    }
test_params_test_run_to_standby_mode = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    'safety_camera.safety_mode': 1,
    'depth_module.exposure': 2000,
    'depth_module.enable_auto_exposure': 'false',
    }
test_params_test_standby_to_service_mode = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    'safety_camera.safety_mode': 2,
    'depth_module.exposure': 3000,
    'depth_module.enable_auto_exposure': 'false',
    }
'''
The test was implemented to check whether ROS wrapper can automatically switch to service mode during
launch of the node and configure the launch params and again switch back to user requested mode.
'''
@pytest.mark.parametrize("launch_descr_with_parameters", [    
    pytest.param(test_params_test_default_run_mode, marks=pytest.mark.d585s),
    pytest.param(test_params_test_run_to_standby_mode, marks=pytest.mark.d585s),
    pytest.param(test_params_test_standby_to_service_mode, marks=pytest.mark.d585s),
    ],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestD585s_TestSafetyMode(pytest_rs_utils.RsTestBaseClass):
    def test_d585s_test_safety_mode(self,launch_descr_with_parameters):
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

            if params['initial_reset'] == 'true':
                self.spin_for_time(wait_time=10)
            else:
                self.spin_for_time(wait_time=5)

            assert self.get_integer_param('safety_camera.safety_mode') == params['safety_camera.safety_mode']
            assert self.get_integer_param('depth_module.exposure') == params['depth_module.exposure']

            depth_metadata = msg_Metadata()
            depth_metadata.json_data = '{"actual_exposure":'+str(params['depth_module.exposure']) +'}'

            themes = [
                {'topic':get_node_heirarchy(params)+'/depth/metadata',
                'msg_type':msg_Metadata,
                'expected_data_chunks':1,
                'data':depth_metadata
                }
            ]

            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)

        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()



test_params_reset_device = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    'rgb_camera.color_profile': '640x360x30',
    }
'''
This test was implemented as a template to set the parameters and run the test.
This directory is not added to the CMakeLists.txt so as to avoid the colcon failure in the
machines that don't have the D455 connected.
1. Only a subset of parameter types are implemented in py_rs_utils, it has to be extended for others
2. After setting the param, rclpy.spin_once may be needed.Test passes even without this though.
'''
@pytest.mark.d585s
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_reset_device],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestD585S_reset_device(pytest_rs_utils.RsTestBaseClass):
    def test_D585S_Reset_Device(self,launch_descr_with_parameters):
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
         'height':360,
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
            assert self.set_string_param('rgb_camera.color_profile', '640x360x30')
            self.spin_for_time(0.5)
            assert self.set_bool_param('enable_color', True)
            self.spin_for_time(0.5)
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
            self.set_string_param('rgb_camera.color_profile', '1280x720x5')
            self.set_bool_param('enable_color', True)
            themes[0]['width'] = 1280
            themes[0]['height'] = 720

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
