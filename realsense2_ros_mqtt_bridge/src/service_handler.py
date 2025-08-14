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
"""Handler for managing ROS service clients and futures."""
import time


class ServiceHandler:
    """
    Handler for managing ROS service clients and futures.

    This class provides methods for creating ROS service clients and waiting
    for futures to complete.
    """

    def __init__(self, mqtt_ros_node):
        """
        Initialize the ServiceHandler.

        Args:
            mqtt_ros_node: Instance of the MQTTBridgeNode.
        """
        self.mqtt_ros_node = mqtt_ros_node

    def create_ros_client(self, srv_type, service_name):
        """
        Create a ROS service client and wait for the service to be available.

        Args:
            srv_type: The type of the ROS service.
            service_name: The name of the ROS service.

        Returns:
            The ROS service client if available, otherwise None.
        """
        try:
            ros_client = self.mqtt_ros_node.create_client(srv_type, service_name)
            if not self.mqtt_ros_node.wait_for_service(ros_client, service_name):
                return None
            return ros_client
        except Exception as e:
            self.mqtt_ros_node.ROS_ERROR(f'[create_ros_client] An exception occurred: {e}')
            return None

    def wait_for_future(self, future):
        """
        Wait for a future to complete.

        Args:
            future: The future to wait for.
        """
        while not future.done():
            time.sleep(0.5)
            self.mqtt_ros_node.ROS_DEBUG('Waiting for ROS service call to finish...')
