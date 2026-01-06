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
from pytest_rs_utils import launch_descr_with_parameters
from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_node_heirarchy



test_params = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'TestCamera1',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }

test_params1 = {"rosbag_filename":get_rosbag_file_path("D435i_Depth_and_IMU_Stands_still.bag"),
    'camera_name': 'TestCamera2',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }
@pytest.mark.rosbag
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params,test_params1],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestCamera2(pytest_rs_utils.RsTestBaseClass):
    def test_node_start(self, launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        themes = [
            {'topic':get_node_heirarchy(params)+'/depth/image_rect_raw',
                'msg_type':msg_Image,
                'store_raw_data':True,
                'expected_data_chunks':1, 
                'frame_id':params['camera_name']+'_depth_optical_frame',
            },
        ]
        try:
            self.init_test("RsTest"+params['camera_name'])
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
            if (data.header.frame_id!=theme['frame_id']):
                return False
        return True
