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
import numpy as np
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
def test_system_calib_config(launch_descr_with_parameters):
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

    sds.set_safety_mode(namespace, name, 2)

    sds.send_get_calib_config_request(namespace, 
        name)
    
    response = sds.receive_get_calib_config_response()
    print("Response:", response)
    original_data = response['calib_config']
    import json
    original_data = original_data.replace("null",str(np.finfo(np.float32).max))
    calib_config = json.loads(original_data)
    if calib_config['calibration_config']['roi_0']['vertex_0'][1] == 35:
        calib_config['calibration_config']['roi_0']['vertex_0'][1] = 36
    else:
        calib_config['calibration_config']['roi_0']['vertex_0'][1] = 35
    calib_config1 = json.dumps(calib_config)

    sds.send_set_calib_config_request(namespace, 
        name,
        calib_config1)

    response = sds.receive_set_calib_config_response()
    
    sds.send_get_calib_config_request(namespace, 
        name)
    
    response = sds.receive_get_calib_config_response()
    cc_read = json.loads(response['calib_config'])
    LOGGER.debug("calib config data written: ",calib_config)
    LOGGER.debug("calib config data readback:",cc_read)
    assert cc_read == calib_config, "Written calib config is not matching with the read one"
    sds.send_set_calib_config_request(namespace, 
        name,
        original_data)

    response = sds.receive_set_calib_config_response()
    #cleanup starts....
    camera.stop()
    rclpy.shutdown()
    LOGGER.info("Test completed")
    #cleanup ends....
