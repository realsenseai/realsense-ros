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


def test_frame_types():
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

        LOGGER.info("Testing color_frame...")
        camera.start_publish_color_frame()
        response = sds.get_frame_msg(namespace, name, "color")
        assert response['success'] == True, 'Receiving color frame was not successful'
        assert response['frame'] != "", 'Invalid frame received'

        LOGGER.info("Testing depth_frame...")
        camera.start_publish_depth_frame()
        response = sds.get_frame_msg(namespace, name, "depth")
        assert response['success'] == True, 'Receiving depth frame was not successful'
        assert response['frame'] != "", 'Invalid frame received'

        LOGGER.info("Testing infra1_frame...")
        camera.start_publish_infra1_frame()
        response = sds.get_frame_msg(namespace, name, "infra1")
        assert response['success'] == True, 'Receiving infra1 frame was not successful'
        assert response['frame'] != "", 'Invalid frame received'

        LOGGER.info("Testing infra2_frame...")
        camera.start_publish_infra2_frame()
        response = sds.get_frame_msg(namespace, name, "infra2")
        assert response['success'] == True, 'Receiving infra2 frame was not successful'
        assert response['frame'] != "", 'Invalid frame received'

        LOGGER.info("Testing invalid stream...")
        response = sds.get_frame_msg(namespace, name, "invalid")
        assert response['success'] != True, 'Expected a failure, but received frame'
        assert response['frame'] == "", 'Frame received for invalid stream'

        LOGGER.info("Testing depth_frame...")
        camera.start_publish_depth_frame()
        response = sds.get_frame_msg(namespace, name, "depth")
        assert response['success'] == True, 'Receiving depth frame was not successful'
        assert response['frame'] != "", 'Invalid frame received'
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
