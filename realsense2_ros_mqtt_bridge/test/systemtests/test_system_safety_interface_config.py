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
import rclpy 

import pytest 
sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))

from camera_n_mqtt_nodes import CameraNMqttNodes as RSCameraSimulator
from camera_n_mqtt_nodes import launch_descr_with_parameters
import camera_n_mqtt_nodes
from mqtt_client_simulator import MQTTClientSimulator
import logging

#LOGGER = logging.getLogger(__name__)
LOGGER = logging.getLogger()

test_params_d585s = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    }

#@pytest.mark.timeout(20)
@pytest.mark.parametrize("launch_descr_with_parameters",[
    pytest.param(test_params_d585s, marks=pytest.mark.d585s),
    ]
    ,indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
def test_system_safety_interface_config(launch_descr_with_parameters):
    #initialization starts....
    rclpy.init()
    params = launch_descr_with_parameters[1]
    namespace = 'camera'
    name = params['camera_name']
    camera = RSCameraSimulator(namespace, name)
    sds = MQTTClientSimulator("localhost", 1883)
    sds.start_client()
    camera.start()
    if LOGGER.getEffectiveLevel() <= logging.DEBUG:
        os.system("ros2 node list")
    #initialization ends....

    LOGGER.info("Testing enumerate_devices")
    sds.send_enumerate_devices_request(namespace, name)
    response = sds.get_enumerate_devices_response()
    assert int(response["available_nodes_count"]) > 0, "Enumerate device failed, couldn't find the device"

    camera.create_safety_interface_config_service()
    #just to ensure no streams are running
    sds.set_safety_mode(namespace, name, 2)

    #safety interface config read works only for RAM and FLASH.
    for index in [1,2]:
        sds.send_get_safety_interface_config_request(namespace, 
            name, 
            index)
        
        response = sds.receive_get_safety_interface_config_response()
        original_data = response["safety_interface_config"]
        import json
        sc_data = json.loads(original_data)
        LOGGER.info(f"safety interface config {index} first read: {sc_data}")
        if sc_data["safety_interface_config"]['smcu_arbitration_params']['l_0_total_threshold'] == 10000:
            sc_data["safety_interface_config"]['smcu_arbitration_params']['l_0_total_threshold'] = 10
        else:
            sc_data["safety_interface_config"]['smcu_arbitration_params']['l_0_total_threshold'] = 10000
        sc_data1 = json.dumps(sc_data)

        LOGGER.debug(f"safety interface config written: {sc_data}")

        #change sp
        sds.send_set_safety_interface_config_request(namespace, 
            name,
            sc_data1)

        response = sds.receive_set_safety_interface_config_response()

        sds.send_get_safety_interface_config_request(namespace, 
            name, 
            index)
        response = sds.receive_get_safety_interface_config_response()
        sc_read = json.loads(response["safety_interface_config"])
        LOGGER.info(f"safety interface config {index} second read: {sc_read}")
        assert sc_read == sc_data, "Written safety interface config is not matching with the read one"
        sds.send_set_safety_interface_config_request(namespace, 
            name,
            original_data)
        response = sds.receive_set_safety_interface_config_response()
    #cleanup starts....
    camera.stop()
    rclpy.shutdown()
    LOGGER.info("Test completed")
    #cleanup ends....
