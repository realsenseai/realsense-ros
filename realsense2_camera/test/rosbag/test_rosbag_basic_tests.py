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

test_params = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'Vis2_Cam',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }
'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py vis_avg_2"
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestVis2(pytest_rs_utils.RsTestBaseClass):
    def test_vis_2(self,delayed_launch_descr_with_parameters):
        params = delayed_launch_descr_with_parameters[1]
        data = pytest_rs_utils.ImageColorGetData(params["rosbag_filename"])
        themes = [
        {'topic':get_node_heirarchy(params)+'/color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'data':data
        }
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)

    
test_params_depth = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'Depth_W_Cloud',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    'enable_pointcloud': 'true'
    }
'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py depth_w_cloud_1"
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_depth],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestDepthWCloud(pytest_rs_utils.RsTestBaseClass):
    def test_depth_w_cloud_1(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        data = pytest_rs_utils.ImageDepthGetData(params["rosbag_filename"])
        themes = [
        {'topic':get_node_heirarchy(params)+'/depth/image_rect_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'data':data
        }
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)


test_params_depth_avg_1 = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'Depth_Avg_1',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }
'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py depth_avg_1"
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_depth_avg_1],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestDepthAvg1(pytest_rs_utils.RsTestBaseClass):
    def test_depth_avg_1(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        data = pytest_rs_utils.ImageDepthGetData(params["rosbag_filename"])
        themes = [
        {'topic':get_node_heirarchy(params)+'/depth/image_rect_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'data':data
        }
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)
    

