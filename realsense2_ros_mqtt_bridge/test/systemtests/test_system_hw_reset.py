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
import json
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

test_params_hmc_d585s = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    'rgb_camera.color_profile': '640x360x30',

    }

@pytest.mark.parametrize("launch_descr_with_parameters",[
    pytest.param(test_params_hmc_d585s, marks=pytest.mark.d585s),
    ]
    ,indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
def test_system_hw_reset(launch_descr_with_parameters):
    #initialization starts....
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
    sds.set_string_param(namespace, name,'rgb_camera.color_profile', '640x360x5')
    param = sds.get_string_param(namespace, name, 'rgb_camera.color_profile')
    assert param == '640x360x5', "Couldn't set the parameter"
    val = sds.send_hw_reset(namespace,name)
    import time
    #can't use the enumerate_devices for this purpose, so banking on sleep
    time.sleep(5)
    param = sds.get_string_param(namespace, name, 'rgb_camera.color_profile')
    assert param == '640x360x30', "Reset didn't work it seems. Please check the logs manually for reset..."
    time.sleep(2)
    #cleanup starts....
    camera.stop()
    rclpy.shutdown()
    LOGGER.info("Test completed")
    #cleanup ends....
