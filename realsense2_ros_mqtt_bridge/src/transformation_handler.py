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
import tf2_ros
from rclpy.time import Time
import json

class TransformationHandler:
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

        # Initialize transform listener
        self.mqtt_ros_node.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.mqtt_ros_node.tf_buffer, self.mqtt_ros_node)

    def handle_get_transformation_request(self, mqtt_request):

        self.mqtt_ros_node.ROS_DEBUG('get_transformation_request message received')
        source = mqtt_request['source']
        destination = mqtt_request['destination']

        try:
            # get transform from 'base_link' to 'child_link'
            transform_stamped = self.mqtt_ros_node.tf_buffer.lookup_transform(source, destination, Time())
            translation_dict = {
                "x": transform_stamped.transform.translation.x,
                "y": transform_stamped.transform.translation.y,
                "z": transform_stamped.transform.translation.z
            }

            rotation_dict = {
                "x": transform_stamped.transform.rotation.x,
                "y": transform_stamped.transform.rotation.y,
                "z": transform_stamped.transform.rotation.z,
                "w": transform_stamped.transform.rotation.w
            }

            self.mqtt_ros_node.get_logger().debug(f'Transform found: {transform_stamped.transform}')
            mqtt_response = {
                'rotation': rotation_dict,
                'translation': translation_dict,
                'success': True,
                'error_msg': ''
            }

        except Exception as e:
            self.mqtt_ros_node.get_logger().error(f'Could not get transform: {e}')
            mqtt_response = {
                'rotation': '{}',
                'translation': '{}',
                'success': False,
                'error_msg': f'Could not get transform: {e}'
            }

        self.mqtt_ros_node.mqtt_client.publish('get_transformation_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('get_transformation_response message sent')


