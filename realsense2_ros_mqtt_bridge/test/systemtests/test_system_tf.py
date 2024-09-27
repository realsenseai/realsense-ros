# Copyright 2024 Intel Corporation. All Rights Reserved.
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

test_params_tf_d585s = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    }

@pytest.mark.parametrize("launch_descr_with_parameters",[
    pytest.param(test_params_tf_d585s, marks=pytest.mark.d585s),
    ]
    ,indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
def test_system_tf(launch_descr_with_parameters):
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
    assert int(response["available_nodes_count"]) > 0, "Enumerate device failed, couldn't find the device"
    links = {
        name+'_link':name+'_depth_frame',
        name+'_depth_frame':name+'_depth_optical_frame',
        name+'_link':name+'_color_frame',
        name+'_color_frame':name+'_color_optical_frame',
    }
    for source, destination in links.items():
        sds.get_transformation(source, destination)
    #cleanup starts....
    camera.stop()
    rclpy.shutdown()
    LOGGER.info("Test completed")
    #cleanup ends....
