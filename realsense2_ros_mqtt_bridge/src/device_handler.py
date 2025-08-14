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
"""DeviceHandler for handling device-related MQTT requests."""
import json

from .service_handler import ServiceHandler
from realsense2_camera_msgs.srv import DeviceInfo

class DeviceHandler(ServiceHandler):
    """
    Handler for device-related MQTT requests.

    This class processes MQTT requests related to enumerating devices and
    retrieves available nodes that match the specified namespace and name
    prefixes.
    """

    def __init__(self, mqtt_ros_node):
        """
        Initialize the DeviceHandler.

        Args:
            mqtt_ros_node: Instance of the MQTTBridgeNode.
        """
        self.mqtt_ros_node = mqtt_ros_node

    def get_available_nodes(self, camera_namespace_prefix, camera_name_prefix):
        """
        Get available nodes that match the given namespace and name prefixes.

        Args:
            camera_namespace_prefix (str): Prefix for the camera namespace.
            camera_name_prefix (str): Prefix for the camera name.

        Returns:
            tuple: A tuple containing the count of available nodes and a string
            representation of the available nodes.
        """
        available_nodes_count = 0
        available_nodes_str = ''
        available_nodes = self.mqtt_ros_node.get_node_names_and_namespaces()
        for node_name, node_namespace in available_nodes:
            if node_namespace[1:].startswith(camera_namespace_prefix) and\
                    node_name.startswith(camera_name_prefix):
                available_nodes_count += 1
                available_nodes_str += '{camera_namespace: ' +\
                    node_namespace[1:] + \
                    ', camera_name: ' + node_name + '}, '

        if available_nodes_count:
            available_nodes_str = available_nodes_str[:-2]

        return available_nodes_count, available_nodes_str

    def handle_enumerate_devices_request(self, mqtt_request):
        """
        Handle the enumerate devices MQTT request.

        Args:
            mqtt_request (dict): The MQTT request message containing the
            camera namespace and name prefixes.
        """
        self.mqtt_ros_node.ROS_DEBUG('enumerate_devices_request \
            message received')

        camera_namespace_prefix = mqtt_request['camera_namespace_prefix']
        camera_name_prefix = mqtt_request['camera_name_prefix']

        nodes_count, nodes_str = self.get_available_nodes(
            camera_namespace_prefix,
            camera_name_prefix)

        mqtt_response = {
            'success': True,
            'error_msg': '',
            'available_nodes_count': str(nodes_count),
            'available_nodes': '[' + nodes_str + ']'
        }

        self.mqtt_ros_node.mqtt_client.publish('enumerate_devices_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('enumerate_devices_response message sent')

    def handle_get_device_info_request(self, mqtt_request):
        """
        Handle the get device info MQTT request.

        Args:
            mqtt_request (dict): The MQTT request message containing the
            camera namespace and camera name of the device.
        """
        self.mqtt_ros_node.ROS_DEBUG('get_device_info message received')

        camera_namespace = mqtt_request['camera_namespace']
        camera_name = mqtt_request['camera_name']

        service_name = f'/{camera_namespace}/{camera_name}/device_info'

        ros_client_get_device_info = self.create_ros_client(DeviceInfo, service_name)
        if not ros_client_get_device_info:
            return

        ros_request = DeviceInfo.Request()

        future = ros_client_get_device_info.call_async(ros_request)
        self.wait_for_future(future)

        ros_response = future.result()

        mqtt_response = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'device_name' : ros_response.device_name,
            'serial_number': ros_response.serial_number,
            'firmware_version': ros_response.firmware_version,
            'usb_type_descriptor': ros_response.usb_type_descriptor,
            'firmware_update_id': ros_response.firmware_update_id,
            'sensors': ros_response.sensors,
            'physical_port': ros_response.physical_port
        }
        self.mqtt_ros_node.mqtt_client.publish('get_device_info_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('get_device_info_response message sent')
        ros_client_get_device_info.destroy()
