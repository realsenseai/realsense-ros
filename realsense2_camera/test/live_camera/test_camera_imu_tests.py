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

test_params_all_profiles_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    'enable_accel':"True",
    'enable_gyro':"True",
    'unite_imu_method':1,
    }

@pytest.mark.parametrize("launch_descr_with_parameters", [
    pytest.param(test_params_all_profiles_d455, marks=pytest.mark.d455)
    ],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestLiveCamera_TestMotionSensor(pytest_rs_utils.RsTestBaseClass):
    def test_LiveCamera_check_motion_sensor(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        if pytest_live_camera_utils.check_if_camera_connected(params['device_type']) == False:
            print("Device not found? : " + params['device_type'])
            assert False
            return
        themes = [{'topic':get_node_heirarchy(params)+'/imu', 'msg_type':msg_Imu,'expected_data_chunks':1},
                {'topic':get_node_heirarchy(params)+'/gyro/sample', 'msg_type':msg_Imu,'expected_data_chunks':1},
                {'topic':get_node_heirarchy(params)+'/accel/sample', 'msg_type':msg_Imu,'expected_data_chunks':1}]
        IMU_TOPIC = 0
        GYRO_TOPIC = 1
        ACCEL_TOPIC = 2
        if params['unite_imu_method'] == '0':
            themes[IMU_TOPIC]['expected_data_chunks'] = 0
        try:
            #initialize 
            self.init_test("RsTest"+params['camera_name'])
            self.create_service_client_ifs(get_node_heirarchy(params))


            #run with default params and check the data
            msg = "Test with the default params "
            print(msg)
            ret = self.run_test(themes, timeout=50)
            assert ret[0], msg + str(ret[1])
            assert self.process_data(themes), msg + " failed"

            msg = "Test with the accel false "
            self.set_integer_param('unite_imu_method', 0)
            self.set_bool_param('enable_accel', False)
            self.set_bool_param('enable_gyro', True)
            themes[ACCEL_TOPIC]['expected_data_chunks'] = 0
            themes[IMU_TOPIC]['expected_data_chunks'] = 0
            print(msg)
            ret = self.run_test(themes)
            assert ret[0], msg + str(ret[1])
            assert self.process_data(themes), msg + " failed"

            msg =  "Test with the gyro false "
            self.set_bool_param('enable_accel', True)
            self.set_bool_param('enable_gyro', False)
            themes[IMU_TOPIC]['expected_data_chunks'] = 0
            themes[ACCEL_TOPIC]['expected_data_chunks'] = 1
            themes[GYRO_TOPIC]['expected_data_chunks'] = 0
            print(msg)
            self.spin_for_time(wait_time=1.0)
            ret = self.run_test(themes)
            assert ret[0], msg + str(ret[1])
            assert self.process_data(themes), msg + " failed"

            msg = "Test with both gyro and accel false "
            self.set_bool_param('enable_accel', False)
            self.set_bool_param('enable_gyro', False)
            self.set_integer_param('unite_imu_method', 1)
            self.spin_for_time(wait_time=1.0)
            themes[ACCEL_TOPIC]['expected_data_chunks'] = 0
            themes[GYRO_TOPIC]['expected_data_chunks'] = 0
            themes[IMU_TOPIC]['expected_data_chunks'] = 0
            print(msg)
            ret = self.run_test(themes, initial_wait_time=1.0) 
            assert ret[0], msg + " failed"
            assert self.process_data(themes), msg +" failed"
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()
