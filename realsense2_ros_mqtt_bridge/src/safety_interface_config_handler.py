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
"""docstring."""
import json

from realsense2_camera_msgs.srv import SafetyInterfaceConfigRead
from realsense2_camera_msgs.srv import SafetyInterfaceConfigWrite

from .service_handler import ServiceHandler


class SafetyInterfaceConfigHandler(ServiceHandler):
    """docstring."""

    def __init__(self, mqtt_ros_node):
        """docstring."""
        super().__init__(mqtt_ros_node)

    def handle_get_safety_interface_config_request(self, mqtt_request):
        """docstring."""
        self.mqtt_ros_node.ROS_DEBUG('get_safety_interface_config_request \
            message received')
        camera_namespace = mqtt_request['camera_namespace']
        camera_name = mqtt_request['camera_name']
        service_name = f'/{camera_namespace}/{camera_name}/safety_interface_config_read'

        ros_client_safety_interface_config_read = self.create_ros_client(SafetyInterfaceConfigRead, service_name)
        if not ros_client_safety_interface_config_read:
            return

        ros_request = SafetyInterfaceConfigRead.Request(calib_location=mqtt_request['calib_location'])

        future = ros_client_safety_interface_config_read.call_async(ros_request)
        self.wait_for_future(future)

        ros_response = future.result()

        mqtt_response = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'safety_interface_config': ros_response.safety_interface_config,
            'success': ros_response.success,
            'error_msg': ros_response.error_message
        }
        self.mqtt_ros_node.mqtt_client.publish('get_safety_interface_config_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('get_safety_interface_config_response message sent')
        ros_client_safety_interface_config_read.destroy()

    def handle_set_safety_interface_config_request(self, mqtt_request):
        """docstring."""
        self.mqtt_ros_node.ROS_DEBUG(
            'set_safety_interface_config_request message received')

        camera_namespace = mqtt_request['camera_namespace']
        camera_name = mqtt_request['camera_name']
        safety_interface_config = mqtt_request['safety_interface_config']
        service_name = f'/{camera_namespace}/{camera_name}/safety_interface_config_write'

        ros_client_safety_interface_config_write = self.create_ros_client(SafetyInterfaceConfigWrite, service_name)
        if not ros_client_safety_interface_config_write:
            return

        ros_request = SafetyInterfaceConfigWrite.Request()
        ros_request.safety_interface_config = safety_interface_config

        future = ros_client_safety_interface_config_write.call_async(ros_request)
        self.wait_for_future(future)

        ros_response = future.result()

        mqtt_response = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'success': ros_response.success,
            'error_msg': ros_response.error_message
        }
        self.mqtt_ros_node.mqtt_client.publish('set_safety_interface_config_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('set_safety_interface_config_response message sent')
        ros_client_safety_interface_config_write.destroy()
