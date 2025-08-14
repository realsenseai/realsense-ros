# Copyright 2024 RealSense, Inc. All Rights Reserved.
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
import logging
#LOGGER = logging.getLogger(__name__)
LOGGER = logging.getLogger()
import pytest


import rclpy
sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
from safety_camera_client import TriggeredCalibrationCameraClient
from pytest_rs_utils import launch_descr_with_parameters
import pytest_live_camera_utils

test_params_align_depth_color_d585s = {
    'camera_namespace': "robot1",
    'camera_name': 'c_333622320031',
    'device_type': 'D585S',
    }

@pytest.mark.parametrize("launch_descr_with_parameters",[
    pytest.param(test_params_align_depth_color_d585s, marks=pytest.mark.d585s),
    ]
    ,indirect=True)

def test_dryrun_triggered_calibration(launch_descr_with_parameters):
    params = launch_descr_with_parameters[1]
    tester = TriggeredCalibrationCameraClient()
    try:
        if pytest_live_camera_utils.check_if_camera_connected(params['device_type']) == False:
            LOGGER.error("Device not found? : " + params['device_type'])
            assert False
            return
        tester.wait_for_node(params['camera_name'])
        time.sleep(3)

        tester.prepare_for_calibration()
        tester.dryrun_calibration()
        
        while tester.tc_done == False:
            rclpy.spin_once(tester)
            time.sleep(1)

        calib_result = tester.calibration_result

        assert calib_result.success == True, "First dryrun calibration run failed"
        time.sleep(0.5)

        tester.dryrun_calibration()
        while tester.tc_done == False:
            rclpy.spin_once(tester)
            time.sleep(1)
        assert tester.calibration_result.success == True, "Second dryrun calibration run failed"

        assert calib_result.calibration == tester.calibration_result.calibration, "Two calibrations gave the same data"

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        LOGGER.warning("Test failed")
        LOGGER.error(exc_type, fname, exc_tb.tb_lineno)
    finally:
        LOGGER.warning("Test completed")
        tester.destroy_node()
        rclpy.shutdown()

import time
if __name__ == '__main__':
    test_triggered_calibration()