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
import subprocess
import time

import pytest
from launch import LaunchDescription
import launch_ros.actions
import launch_pytest
import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu

sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_yaml
from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_node_heirarchy

''' 
This is a testcase simiar to the integration_fn testcase, the only difference is that
this one uses the launch configuration to launch the nodes.  
'''
test_params = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'test_Cam',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }

#skipping as the test doesn't work in industrial_ci
@pytest.mark.skip
@pytest.mark.launch(fixture=launch_descr_with_yaml)
@pytest.mark.parametrize("launch_descr_with_yaml", [test_params],indirect=True)
def test_using_function(launch_context,launch_descr_with_yaml):
    params = launch_descr_with_yaml[1]
    time.sleep(0.1)
    # by now, the camera would have started
    start = time.time()
    timeout = 4.0
    camera_name = get_node_heirarchy(params)+'/'+params['camera_name'] 
    while (time.time() - start) < timeout:
        service_list = subprocess.check_output(['ros2', 'node', 'list']).decode("utf-8")
        is_node_up = camera_name in service_list
        if is_node_up == True:
            break
        else:
            print("node list output:"+service_list)
        time.sleep(timeout/5)
    else:
        print("Timed out searching for node")

    assert is_node_up, 'Node is NOT UP'
    print ('Node is UP')
    print ('*'*8 + ' Killing ROS ' + '*'*9)
    yield
    pytest_rs_utils.kill_realsense2_camera_node()
    time.sleep(0.1)
    assert True


''' 
This is another test that can be used as a template. The expectation is that most of the
integration tests will use this format where the tester can create a test node to subscribe
to different published topics and decide whether the test passed or failed.  
'''

'''
use the launch description from the utils and also inherit from basic test class RsTestBaseClass
'''
test_params = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'test_Cam2',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }
@pytest.mark.rosbag
@pytest.mark.launch(fixture=pytest_rs_utils.launch_descr_with_yaml)
@pytest.mark.parametrize("launch_descr_with_yaml", [test_params],indirect=True)
class TestCamera1(pytest_rs_utils.RsTestBaseClass):
    def test_camera_1(self, launch_descr_with_yaml):
        params = launch_descr_with_yaml[1]
        themes = [
            #{'topic':'/camera/color/image_raw','msg_type':msg_Image,'expected_data_chunks':1},
        {'topic':get_node_heirarchy(params)+'/depth/image_rect_raw','msg_type':msg_Image,'expected_data_chunks':1}
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test('RsTest'+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)
test_params = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'test_Cam3',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }
@pytest.mark.rosbag
@pytest.mark.launch(fixture=pytest_rs_utils.launch_descr_with_yaml)
@pytest.mark.parametrize("launch_descr_with_yaml", [test_params],indirect=True)
class TestCamera2(pytest_rs_utils.RsTestBaseClass):
    def test_camera_2(self,launch_descr_with_yaml):
        params = launch_descr_with_yaml[1]
        themes = [
            {'topic':get_node_heirarchy(params)+'/depth/image_rect_raw',
                'msg_type':msg_Image,
                'store_raw_data':True,
                'expected_data_chunks':1, 
                'frame_id':params['camera_name']+'_depth_optical_frame',
                'height':720,
                'width':1280},
            {'topic':get_node_heirarchy(params)+'/color/image_raw',
                'msg_type':msg_Image,
                'store_raw_data':True,
                'expected_data_chunks':1, 
                'frame_id':params['camera_name']+'_color_optical_frame',
                'height':480,
                'width':640},
        ]
        ''' # TODO: find a rosbag file that has accel/sample to test this
                    {'topic': '/camera/accel/sample', 
                        'msg_type': msg_Imu,
                        'expected_data_chunks':1, 
                        },
        '''
        try:
            self.init_test('RsTest'+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()

    ''' 
    override the process_data and check if the data is correct or not 
    '''
    def process_data(self, themes):
        for theme in themes:
            data = self.node.pop_first_chunk(theme['topic'])
            #message format can be found at /opt/ros/humble/share/sensor_msgs/msg/Image.msg
            print(data.header)
            if (data.header.frame_id!=theme['frame_id']) or (data.height != theme['height']) or (data.width != theme['width']):
                return False
        return True
