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
test_params_tf_static_change_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    'enable_infra1': 'false',
    'enable_infra2': 'true',
    'enable_accel': 'true',
    'enable_gyro': 'true',
    }
test_params_tf_static_change_d435i = {
    'camera_name': 'D435I',
    'device_type': 'D435I',
    'enable_infra1': 'false',
    'enable_infra2': 'true',
    'enable_accel': 'true',
    'enable_gyro': 'true',
    }

test_params_tf_static_change_d415 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    'enable_infra1': 'false',
    'enable_infra2': 'true',
    'enable_accel': 'true',
    'enable_gyro': 'true',
    }    
@pytest.mark.parametrize("launch_descr_with_parameters", [
    #LRS-1181 [ROS2] To debug inconsistent TF (transform) test that fails on Jenkin 219 NUC on D455
    #pytest.param(test_params_tf_static_change_d455, marks=pytest.mark.d455),
    pytest.param(test_params_tf_static_change_d435i, marks=pytest.mark.d435i),
    pytest.param(test_params_tf_static_change_d415, marks=pytest.mark.d415),
    ],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestCamera_TestTF_Static_change(pytest_rs_utils.RsTestBaseClass):
    def test_camera_test_tf_static_change(self,launch_descr_with_parameters):
        self.params = launch_descr_with_parameters[1]
        if pytest_live_camera_utils.check_if_camera_connected(self.params['device_type']) == False:
            print("Device not found? : " + self.params['device_type'])
            assert False
            return
        themes = [
        {'topic':'/tf_static',
         'msg_type':msg_TFMessage,
         'expected_data_chunks':1,
         'qos' :  QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST,)#static
        }
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
            assert self.set_bool_param('enable_infra1', True)

            ret = self.run_test(themes, timeout=10)
            assert ret[0], ret[1]
            ret = self.process_data(themes, True)
            assert ret[0], ret[1]
        finally:
            self.shutdown()
    def process_data(self, themes, enable_infra1):
        frame_ids = [self.params['camera_name']+'_link', 
            self.params['camera_name']+'_depth_frame', 
            self.params['camera_name']+'_infra2_frame', 
            self.params['camera_name']+'_color_frame']
        if self.params['device_type'] == 'D455':
            frame_ids.append(self.params['camera_name']+'_gyro_frame')
            frame_ids.append(self.params['camera_name']+'_accel_frame')
        frame_ids.append(self.params['camera_name']+'_infra1_frame')
        data = self.node.pop_first_chunk('/tf_static')
        coupled_frame_ids = [xx for xx in itertools.combinations(frame_ids, 2)]
        res = self.get_transform_data(data, coupled_frame_ids, is_static=True)
        for couple in coupled_frame_ids:
            if self.params['camera_name']+'_infra1_frame' in couple:
                if enable_infra1 == True and res[couple] != None:
                        continue
                if enable_infra1 == False and res[couple] == None:
                        continue
                return False, str(couple) + ": tf_data not as expected"
            if res[couple] == None:
                return False, str(couple) + ": didn't get any tf data"
        return True,""


test_params_tf_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    'publish_tf': 'true',
    'tf_publish_rate': '1.1',
    }

test_params_tf_d435i = {
    'camera_name': 'D435I',
    'device_type': 'D435I',
    'publish_tf': 'true',
    'tf_publish_rate': '1.1',
    }

test_params_tf_d415 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    'publish_tf': 'true',
    'tf_publish_rate': '1.1',
    }
@pytest.mark.parametrize("launch_descr_with_parameters", [
    #LRS-1181 [ROS2] To debug inconsistent TF (transform) test that fails on Jenkin 219 NUC on D455
    #pytest.param(test_params_tf_d455, marks=pytest.mark.d455),
    pytest.param(test_params_tf_d435i, marks=pytest.mark.d435i),
    pytest.param(test_params_tf_d415, marks=pytest.mark.d415),
    ],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestCamera_TestTF_DYN(pytest_rs_utils.RsTestBaseClass):
    def test_camera_test_tf_dyn(self,launch_descr_with_parameters):
        self.params = launch_descr_with_parameters[1]
        if pytest_live_camera_utils.check_if_camera_connected(self.params['device_type']) == False:
            print("Device not found? : " + self.params['device_type'])
            assert False
            return
        themes = [
        {'topic':'/tf',
         'msg_type':msg_TFMessage,
         'expected_data_chunks':3,
         'qos' :  QoSProfile(depth=100,durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST,)
         #'qos' :  QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST,)#static
        },
        {'topic':'/tf_static',
         'msg_type':msg_TFMessage,
         'expected_data_chunks':1,
         #'qos' :  QoSProfile(depth=100,durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST,)
         'qos' :  QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST,)#static
        }
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
        frame_ids = [self.params['camera_name']+'_link', 
            self.params['camera_name']+'_depth_frame', 
            self.params['camera_name']+'_color_frame']
        data = self.node.pop_first_chunk('/tf_static')
        ret = self.check_transform_data(data, frame_ids, True)
        assert ret[0], ret[1]
        data = self.node.pop_first_chunk('/tf')
        ret = self.check_transform_data(data, frame_ids)
        assert ret[0], ret[1]
        data = self.node.pop_first_chunk('/tf')
        ret = self.check_transform_data(data, frame_ids)
        assert ret[0], ret[1]
        data = self.node.pop_first_chunk('/tf')
        ret = self.check_transform_data(data, frame_ids)
        assert ret[0], ret[1]
        #return True, ""

        return True, ""
