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
def test_system_frame_types(launch_descr_with_parameters):
    #initialization starts....
    try:
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
        params = [
            {"param_name":'safety_camera.safety_mode', "default_value":0, "param_type":"int"},
        ]
        camera.add_parameters(params)
        
        LOGGER.info("Testing enumerate_devices")
        sds.send_enumerate_devices_request(namespace, name)
        response = sds.get_enumerate_devices_response()
        assert int(response["available_nodes_count"]) > 0, "Enumerate device failed, couldn't find the device"
        '''safety stream suport is not required
        LOGGER.info("Testing safety_frame...")
        sds.start_stop_safety_stream(namespace, name, True)
        frame = sds.get_frame_msg(namespace, name, "safety")
        LOGGER.debug(frame)
        '''
        LOGGER.info("Testing color_frame...")
        sds.start_stop_color_stream(namespace, name, True)
        frame = sds.get_frame_msg(namespace, name, "color")
        LOGGER.debug(frame)
        sds.start_stop_color_stream(namespace, name, False)

        LOGGER.info("Testing depth_frame...")
        sds.start_stop_depth_stream(namespace, name, True)
        frame = sds.get_frame_msg(namespace, name, "depth")
        LOGGER.debug(frame)
        sds.start_stop_depth_stream(namespace, name, False)

        LOGGER.info("Testing infra1_frame...")
        sds.start_stop_infra1_stream(namespace, name, True)
        frame = sds.get_frame_msg(namespace, name, "infra1")
        LOGGER.debug(frame)

        LOGGER.info("Testing infra2_frame...")
        sds.start_stop_infra2_stream(namespace, name, True)
        frame = sds.get_frame_msg(namespace, name, "infra2")
        LOGGER.debug(frame)
    except:
        raise
    finally:
        #cleanup starts....
        camera.stop()
        rclpy.shutdown()
        LOGGER.info("Test completed")
        #cleanup ends....
