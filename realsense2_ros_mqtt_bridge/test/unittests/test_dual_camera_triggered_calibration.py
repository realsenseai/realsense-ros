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




def test_dual_camera_triggered_calibration():
    #initialization starts....
    try:
        namespace = 'camera'
        name = 'camera2'
        camera2 = RSCameraSimulator(namespace, name)
        name1 = 'camera1'
        camera1 = RSCameraSimulator(namespace, name1)
        sds = MQTTClientSimulator("localhost", 1883)
        sds.start_client()
        camera2.start()
        camera1.start()
        if LOGGER.getEffectiveLevel() <= logging.DEBUG:
            os.system("ros2 node list")
    #initialization ends....

        LOGGER.info("Testing enumerate_devices")
        sds.send_enumerate_devices_request(namespace, name)
        response = sds.get_enumerate_devices_response()
        assert int(response["available_nodes_count"]) > 0, f"Enumerate device failed, couldn't find the device {name}"

        sds.send_enumerate_devices_request(namespace, name1)
        response = sds.get_enumerate_devices_response()
        assert int(response["available_nodes_count"]) > 0, f"Enumerate device failed, couldn't find the device {name1}"

        camera2.create_triggered_calibration_action()
        camera1.create_triggered_calibration_action()

        sds.send_triggered_calibration_request(namespace, 
            name)

        while True:
            response = sds.receive_triggered_calibration_response()
            LOGGER.info(f"Response: {response}")
            if response['progress'] > 5.0:
                break

        sds.send_triggered_calibration_request(namespace, 
            name1)
        camera1_response = False
        camera2_response = False

        while True:
            response = sds.receive_triggered_calibration_response()
            LOGGER.info(f"Response: {response}")
            if response['progress'] == 100.0 and response['camera_name'] == 'camera2':
                assert response['success'] == True, 'Triggered calibraton was not successful'
                assert response['calibration'] == "calib run", 'Unexpected calibration value received'
                camera2_response = True
                if camera1_response:
                    break
            if response['progress'] == 100.0 and response['camera_name'] == 'camera1':
                assert response['success'] == True, 'Triggered calibraton was not successful'
                assert response['calibration'] == "calib run", 'Unexpected calibration value received'
                camera1_response = True
                if camera2_response:
                    break

    #cleanup starts....

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        LOGGER.warning("Test failed")
        LOGGER.warning(e)
        LOGGER.error(exc_type, fname, exc_tb.tb_lineno)
    camera2.stop()
    camera1.stop()
    LOGGER.info("Test completed")
    #cleanup ends....
