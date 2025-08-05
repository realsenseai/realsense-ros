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
This test imitates the ros2 launch rs_launch.py realsense2_camera with the given parameters below
Full command to reproduce locally
ros2 launch rs_launch.py realsense2_camera camera_name:=camera enable_color:=true \
    enable_depth:=true rgb_camera.profile:=1280x720x30 depth_module.profile:=640x480x30 \
    align_depth.enable:true rosbag_filename:=`realpath outdoors_1color.bag`

Then we check if these topics exist:
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/aligned_depth_to_color/image_raw

Also we check that the recieved frames of each topic are in the right width and height
'''
test_params = {
    "rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'camera_1',
    'enable_color': 'true',
    'enable_depth': 'true',
    'depth_module.profile': '1280x720x30',
    'rgb_camera.profile': '640x480x30',
    'align_depth.enable': 'true',
    }


@pytest.mark.rosbag
@pytest.mark.launch(fixture=pytest_rs_utils.launch_descr_with_yaml)
@pytest.mark.parametrize("launch_descr_with_yaml", [test_params],indirect=True)
class TestBasicAlignDepthEnable(pytest_rs_utils.RsTestBaseClass):
    def test_align_depth_on(self, launch_descr_with_yaml):
        params = launch_descr_with_yaml[1]
        themes = [
            {'topic': get_node_heirarchy(params)+'/color/image_raw', 'msg_type': msg_Image,
             'expected_data_chunks': 1, 'width': 640, 'height': 480},
            {'topic': get_node_heirarchy(params)+'/depth/image_rect_raw', 'msg_type': msg_Image,
             'expected_data_chunks': 1, 'width': 1280, 'height': 720},
            {'topic': get_node_heirarchy(params)+'/aligned_depth_to_color/image_raw', 'msg_type': msg_Image,
             'expected_data_chunks': 1, 'width': 640, 'height': 480}
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
