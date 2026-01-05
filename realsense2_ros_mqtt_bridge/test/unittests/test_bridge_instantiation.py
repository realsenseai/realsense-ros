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

from camera_node_simulator import RSCameraSimulator
from mqtt_client_simulator import MQTTClientSimulator
import logging

#LOGGER = logging.getLogger(__name__)
LOGGER = logging.getLogger()


def test_bridge_instatiation():
    #initialization starts....
    try:
        namespace = 'camera'
        name = 'camera'
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


        LOGGER.info("Testing get_param...")

        sds.send_get_param_request(namespace,
                name,
                'safety_camera.safety_mode')
        
        response = sds.receive_get_param_response()

        LOGGER.debug("safety_camera.safety_mode Param received: " + str(response["parameter_value"]))    

        LOGGER.info("Testing set_param...")

        sds.send_set_param_request(namespace,
                name,
                'safety_camera.safety_mode',
                '2',
                'int')
        response = sds.receive_set_param_response()

        response = sds.get_param(namespace,
            name,
            'safety_camera.safety_mode')
        assert int(response["parameter_value"]) == 2, "Get or Set param failed, didn't get the written value"
        LOGGER.debug("safety_camera.safety_mode Param received: " + str(response))  

        LOGGER.info("Testing get_frame...")
        camera.start_publish_color_frame()
        frame = sds.get_frame(namespace, name, "color")
        LOGGER.debug(frame)

    #cleanup starts....

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        LOGGER.error("Test failed")
        LOGGER.error(e)
        LOGGER.error(exc_type, fname, exc_tb.tb_lineno)
    camera.stop()
    LOGGER.info("Test completed")
    #cleanup ends....
