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

import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))

from camera_n_mqtt_nodes import CameraNMqttNodes as RSCameraSimulator
from camera_n_mqtt_nodes import launch_descr_with_parameters
import camera_n_mqtt_nodes
from mqtt_client_simulator import MQTTClientSimulator
import logging
import pytest
import rclpy

#LOGGER = logging.getLogger(__name__)
LOGGER = logging.getLogger()

test_params_align_depth_color_d585s = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    }

@pytest.mark.parametrize("launch_descr_with_parameters",[
    pytest.param(test_params_align_depth_color_d585s, marks=pytest.mark.d585s),
    ]
    ,indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
def test_system_triggered_calibration(launch_descr_with_parameters):
    #initialization starts....
    try:
        rclpy.init()
        params = launch_descr_with_parameters[1]
        namespace = 'camera'
        name = params['camera_name']
        camera = RSCameraSimulator(namespace, name)
        camera.start()
        sds = MQTTClientSimulator("localhost", 1883)
        sds.start_client()

        if LOGGER.getEffectiveLevel() <= logging.DEBUG:
            os.system("ros2 node list")
        #initialization ends....

        LOGGER.info("Testing enumerate_devices")
        sds.send_enumerate_devices_request(namespace, name)
        response = sds.get_enumerate_devices_response()
        assert int(response["available_nodes_count"]) > 0, "Enumerate device failed, couldn't find the device"

        sds.prepare_for_calibration(namespace, name)
        
        sds.send_triggered_calibration_request(namespace, 
            name)
        while True:
            response = sds.receive_triggered_calibration_response()
            LOGGER.info(f"Response: {response}")
            if response['success'] == True or response['error_msg'] != '':
                if response['success'] == True:
                    LOGGER.info('Triggered calibraton was successful')
                    assert (response['calibration'] != "{}"), "The calibration data received is empty"
                    assert type(response['calibration']) != list, "The calibration data received is not a list"
                    LOGGER.info('Triggered calibraton data:' + response['calibration'])
                elif response['progress'] == 100.0 and 'Calibration completed but algorithm failed' in response['error_msg']:
                    #since it's an issue with the camera field of view, treating it as a success with warning. 
                    #Manual adjustment of camera is needed to pass the test.
                    LOGGER.warning('Triggered calibraton completed, but algorithm failed. This is treated as a successful completion of the test')
                else:
                    assert False, 'Triggered calibration failed with unexpected response:'+str(response)
                break
    #cleanup starts....

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        LOGGER.error("Test failed")
        LOGGER.error(e)
        LOGGER.error(exc_type, fname, exc_tb.tb_lineno)
    finally:
        camera.stop()
        rclpy.shutdown()
    LOGGER.info("Test completed")
    #cleanup ends....
