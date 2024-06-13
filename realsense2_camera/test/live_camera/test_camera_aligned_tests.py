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



test_params_align_depth_color_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    'enable_color':'true',
    'enable_depth':'true',
    'depth_module.depth_profile':'848x480x30',    
    'rgb_camera.color_profile':'640x480x30',    
    'align_depth.enable':'true'
    }
test_params_align_depth_color_d415 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    'enable_color':'true',
    'enable_depth':'true',
    'depth_module.depth_profile':'848x480x30',    
    'rgb_camera.color_profile':'640x480x30',    
    'align_depth.enable':'true'
    }
'''
This test was implemented as a template to set the parameters and run the test.
This directory is not added to the CMakeLists.txt so as to avoid the colcon failure in the
machines that don't have the D455 connected.
1. Only a subset of parameter types are implemented in py_rs_utils, it has to be extended for others
2. After setting the param, rclpy.spin_once may be needed.Test passes even without this though.
'''

@pytest.mark.parametrize("launch_descr_with_parameters",[
    pytest.param(test_params_align_depth_color_d455, marks=pytest.mark.d455),
    pytest.param(test_params_align_depth_color_d415, marks=pytest.mark.d415),
    #pytest.param(test_params_align_depth_color_d435, marks=pytest.mark.d435),
    ]
    ,indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestCamera_AlignDepthColor(pytest_rs_utils.RsTestBaseClass):
    def test_camera_align_depth_color(self,launch_descr_with_parameters):
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
        },
        {'topic':get_node_heirarchy(params)+'/depth/image_rect_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'width':848,
         'height':480,
        },
        {'topic':get_node_heirarchy(params)+'/aligned_depth_to_color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'width':640,
         'height':480,
        },
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            print("Starting camera test...")
            self.init_test("RsTest"+params['camera_name'])
            self.wait_for_node(params['camera_name'])
            self.create_param_ifs(get_node_heirarchy(params))
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
            self.set_string_param('rgb_camera.color_profile', '1280x720x30')
            self.set_bool_param('enable_color', True)
            themes[0]['width'] = 1280
            themes[0]['height'] = 720
            themes[2]['width'] = 1280
            themes[2]['height'] = 720

            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()

test_params_all_profiles_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    'enable_color':'true',
    'enable_depth':'true',
    'depth_module.depth_profile':'848x480x30',    
    'rgb_camera.color_profile':'640x480x30',    
    'align_depth.enable':'true'
    }
test_params_all_profiles_d415 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    'enable_color':'true',
    'enable_depth':'true',
    'depth_module.depth_profile':'848x480x30',    
    'rgb_camera.color_profile':'640x480x30',    
    'align_depth.enable':'true'
    }
test_params_all_profiles_d435i = {
    'camera_name': 'D435I',
    'device_type': 'D435I',
    'enable_color':'true',
    'enable_depth':'true',
    'depth_module.depth_profile':'848x480x30',    
    'rgb_camera.color_profile':'640x480x30',    
    'align_depth.enable':'true'
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
class TestCamera_AllAlignDepthColor(pytest_rs_utils.RsTestBaseClass):
    def test_camera_all_align_depth_color(self,launch_descr_with_parameters):
        skipped_tests = []
        failed_tests = []
        num_passed = 0
        num_failed = 0
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
        },
        {'topic':get_node_heirarchy(params)+'/depth/image_rect_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'width':848,
         'height':480,
        },
        {'topic':get_node_heirarchy(params)+'/aligned_depth_to_color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'width':640,
         'height':480,
        },
        ]
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
            self.create_param_ifs(get_node_heirarchy(params))
            color_profiles = set([i[1] for i in cap["color_profile"] if i[2] == "RGB8"])
            depth_profiles = set([i[1] for i in cap["depth_profile"] if i[0] == "Depth"])
            for color_profile in color_profiles:
                for depth_profile in depth_profiles:
                    if depth_profile == color_profile:
                        continue
                    print("Testing the alignment of Depth:", depth_profile, " and Color:", color_profile)
                    self.set_bool_param('enable_color', False)
                    self.set_bool_param('enable_color', False)
                    self.set_bool_param('align_depth.enable', False)

                    themes[0]['width'] = themes[2]['width'] = int(color_profile.split('x')[0])
                    themes[0]['height'] = themes[2]['height'] = int(color_profile.split('x')[1])
                    themes[1]['width'] = int(depth_profile.split('x')[0])
                    themes[1]['height'] = int(depth_profile.split('x')[1])
                    dfps = int(depth_profile.split('x')[2])
                    cfps = int(color_profile.split('x')[2])
                    if dfps > cfps:
                        fps = cfps
                    else:
                        fps = dfps
                    timeout=100.0/fps
                    #for the changes to take effect
                    self.spin_for_time(wait_time=timeout/20)
                    self.set_string_param('rgb_camera.color_profile', color_profile)
                    self.set_string_param('depth_module.depth_profile', depth_profile)
                    self.set_bool_param('enable_color', True)
                    self.set_bool_param('enable_color', True)
                    self.set_bool_param('align_depth.enable', True)
                    #for the changes to take effect
                    self.spin_for_time(wait_time=timeout/20)
                    try:
                        ret = self.run_test(themes, timeout=timeout)
                        assert ret[0], ret[1]
                        assert self.process_data(themes), "".join(str(depth_profile) + " " + str(color_profile)) + " failed"
                        num_passed += 1
                    except Exception as e:
                        exc_type, exc_obj, exc_tb = sys.exc_info()
                        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                        print("Test failed")
                        print("Tested the alignment of Depth:", depth_profile, " and Color:", color_profile, " with timeout ", timeout)
                        print(e)
                        print(exc_type, fname, exc_tb.tb_lineno)
                        num_failed += 1
                        failed_tests.append("".join(str(depth_profile) + " " + str(color_profile)))
                    
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

