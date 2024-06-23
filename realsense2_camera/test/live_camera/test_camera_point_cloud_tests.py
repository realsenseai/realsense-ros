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

from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
import tf2_ros


from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
from tf2_msgs.msg import TFMessage as msg_TFMessage

import numpy as np

sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_parameters

import pytest_live_camera_utils
from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_node_heirarchy

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters

'''
The test was implemented to check the tf_static before and after infra1 was enabled. There shouldn't be any
related data before enabling.
'''
test_params_points_cloud_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    'pointcloud.enable': 'true'
    }
test_params_points_cloud_d435i = {
    'camera_name': 'D435I',
    'device_type': 'D435I',
    'pointcloud.enable': 'true'
    }

test_params_points_cloud_d415 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    'enable_color':'true',
    'enable_depth':'true',
    'depth_module.profile':'848x480x30',
    'pointcloud.enable': 'true'
    }    

'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py points_cloud_1"
'''
@pytest.mark.parametrize("launch_descr_with_parameters", [
    pytest.param(test_params_points_cloud_d455, marks=pytest.mark.d455),
    pytest.param(test_params_points_cloud_d435i, marks=pytest.mark.d435i),
    pytest.param(test_params_points_cloud_d415, marks=pytest.mark.d415),
    ],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestCamera_TestPointCloud(pytest_rs_utils.RsTestBaseClass):
    def test_camera_test_point_cloud(self,launch_descr_with_parameters):
        self.params = launch_descr_with_parameters[1]
        if pytest_live_camera_utils.check_if_camera_connected(self.params['device_type']) == False:
            print("Device not found? : " + self.params['device_type'])
            assert False
            return
        themes = [
        {
         'topic':get_node_heirarchy(self.params)+'/depth/color/points',
         'msg_type':msg_PointCloud2,
         'expected_data_chunks':5,
        },
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+self.params['camera_name'])
            self.wait_for_node(self.params['camera_name'])
            self.create_service_client_ifs(get_node_heirarchy(self.params))
            ret = self.run_test(themes, timeout=10)
            assert ret[0], ret[1]
            ret = self.process_data(themes, False)
            assert ret[0], ret[1]
        finally:
            self.shutdown()
    def process_data(self, themes, enable_infra1):
        for count in range(self.node.get_num_chunks(get_node_heirarchy(self.params)+'/depth/color/points')):
            data = self.node.pop_first_chunk(get_node_heirarchy(self.params)+'/depth/color/points')
            print(data)#the frame counter starts with zero, jumps to 2 and continues. To be checked
        return True,""

