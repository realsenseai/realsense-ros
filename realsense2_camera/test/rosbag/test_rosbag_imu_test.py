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

import numpy as np
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
from realsense2_camera_msgs.msg import Extrinsics as msg_Extrinsics
from realsense2_camera_msgs.msg import Metadata as msg_Metadata

from array import array
from builtin_interfaces.msg import Time
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo


sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_parameters

from pytest_rs_utils import delayed_launch_descr_with_parameters
from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_node_heirarchy


test_params_accel = {"rosbag_filename":get_rosbag_file_path("D435i_Depth_and_IMU_Stands_still.bag"),
    'camera_name': 'Accel_Cam',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    'enable_accel': 'true',
    'accel_fps': '0.0'
    }
'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py accel_up_1"
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_accel],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestAccelUp1(pytest_rs_utils.RsTestBaseClass):
    def test_accel_up_1(self,delayed_launch_descr_with_parameters):
        params = delayed_launch_descr_with_parameters[1]
        data = pytest_rs_utils.AccelGetDataDeviceStandStraight(params["rosbag_filename"])
        themes = [
        {'topic':get_node_heirarchy(params)+'/accel/sample',
         'msg_type':msg_Imu,
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

test_params_imu_topics = {#"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
                          "rosbag_filename":get_rosbag_file_path("D435i_Depth_and_IMU_Stands_still.bag"),
    'camera_name': 'ImuTopics',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    'enable_accel':True,
    'enable_gyro':True,
    'unite_imu_method':1,
    'delay_ms':3000, #delay the start
    }
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_imu_topics],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestImuTopics(pytest_rs_utils.RsTestBaseClass):
    def test_imu_topics(self,delayed_launch_descr_with_parameters):
        ''' 
        current rosbag file doesn't have color data 
        '''
        params = delayed_launch_descr_with_parameters[1]
        self.rosbag = params["rosbag_filename"]
        themes = [{
         'topic':get_node_heirarchy(params)+'/imu',
         'msg_type':msg_Imu,
         'expected_data_chunks':1,
         #'data':depth_to_color_data
        },
        {
         'topic':get_node_heirarchy(params)+'/gyro/sample',
         'msg_type':msg_Imu,
         'expected_data_chunks':1,
         #'data':depth_to_color_data
        },
        {
         'topic':get_node_heirarchy(params)+'/accel/sample',
         'msg_type':msg_Imu,
         'expected_data_chunks':1,
         #'data':depth_to_color_data
        },
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes), "Data check failed, probably the rosbag file changed?"
        finally:
            self.shutdown()
