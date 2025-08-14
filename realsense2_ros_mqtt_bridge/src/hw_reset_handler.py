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
"""HwResetHandler for handling device-related MQTT requests."""
import json
import numpy as np

from .service_handler import ServiceHandler
from std_srvs.srv import Empty


class HwResetHandler(ServiceHandler):
    """
    Handler for hardware reset MQTT request.

    This class processes MQTT requests related to sending hardware reset request to the device.
    """

    def __init__(self, mqtt_ros_node):
        """
        Initialize the HwResetHandler.

        Args:
            mqtt_ros_node: Instance of the MQTTBridgeNode.
        """
        self.mqtt_ros_node = mqtt_ros_node

    def handle_hw_reset_send_request(self, mqtt_request):
        """
        Handle the get device info MQTT request.

        Args:
            mqtt_request (dict): The MQTT request message containing the
            camera namespace and camera name of the device and the hwm command to run
        """
        self.mqtt_ros_node.ROS_DEBUG('send_hw_reset_request message received')
        camera_namespace = mqtt_request['camera_namespace']
        camera_name = mqtt_request['camera_name']

        service_name = f'/{camera_namespace}/{camera_name}/hw_reset'

        ros_client_hw_reset_command_send = self.create_ros_client(Empty, service_name)
        if not ros_client_hw_reset_command_send:
            return

        ros_request = Empty.Request()

        future = ros_client_hw_reset_command_send.call_async(ros_request)
        self.wait_for_future(future)
        ros_response = future.result()
        print("Pras", str(ros_response))

        mqtt_response = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'success' : True,
            'error_msg': ""
        }

        self.mqtt_ros_node.mqtt_client.publish('send_hw_reset_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('send_hw_reset_response message sent')
        ros_client_hw_reset_command_send.destroy()
