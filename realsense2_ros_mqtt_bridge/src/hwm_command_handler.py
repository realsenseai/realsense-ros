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
"""HWMCommandHandler for handling device-related MQTT requests."""
import json
import numpy as np

from .service_handler import ServiceHandler
from realsense2_camera_msgs.srv import HardwareMonitorCommandSend

class HWMCommandHandler(ServiceHandler):
    """
    Handler for HWM command-related MQTT requests.

    This class processes MQTT requests related to sending hardware monitor commands to the device.
    """

    def __init__(self, mqtt_ros_node):
        """
        Initialize the HWMCommandHandler.

        Args:
            mqtt_ros_node: Instance of the MQTTBridgeNode.
        """
        self.mqtt_ros_node = mqtt_ros_node

    def handle_hwm_command_send_request(self, mqtt_request):
        """
        Handle the get device info MQTT request.

        Args:
            mqtt_request (dict): The MQTT request message containing the
            camera namespace and camera name of the device and the hwm command to run
        """
        self.mqtt_ros_node.ROS_DEBUG('send_hwm_command_request message received')
        camera_namespace = mqtt_request['camera_namespace']
        camera_name = mqtt_request['camera_name']

        service_name = f'/{camera_namespace}/{camera_name}/hardware_monitor_command_send'

        ros_client_hwm_command_send = self.create_ros_client(HardwareMonitorCommandSend, service_name)
        if not ros_client_hwm_command_send:
            return

        ros_request = HardwareMonitorCommandSend.Request()
        # ros_request
        ros_request.opcode = mqtt_request['opcode']
        ros_request.param1 = mqtt_request.get('param1', 0)
        ros_request.param2 = mqtt_request.get('param2', 0)
        ros_request.param3 = mqtt_request.get('param3', 0)
        ros_request.param4 = mqtt_request.get('param4', 0)
        ros_request.data = np.array(mqtt_request.get('data', [])).tolist()

        future = ros_client_hwm_command_send.call_async(ros_request)
        self.wait_for_future(future)
        ros_response = future.result()

        mqtt_response = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'success' : ros_response.success,
            'result': np.array(ros_response.result).tolist(),
            'error_msg': ros_response.error_message
        }

        self.mqtt_ros_node.mqtt_client.publish('send_hwm_command_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('hwm_command_send_response message sent')
        ros_client_hwm_command_send.destroy()
