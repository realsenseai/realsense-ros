# Copyright 2024 Intel Corporation. All Rights Reserved.
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
"""FrameHandler for handling frame-related MQTT requests."""
import json

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from sensor_msgs.msg import Image


class FrameHandler:
    """
    Handler for frame-related MQTT requests.

    This class processes MQTT requests related to getting frames from a camera
    and retrieves the corresponding image frames.
    """

    def __init__(self, mqtt_ros_node):
        """
        Initialize the FrameHandler.

        Args:
            mqtt_ros_node: Instance of the MQTTBridgeNode.
        """
        self.mqtt_ros_node = mqtt_ros_node
        self.image = None

        # Create a QoS profile for sensor data
        self.sensor_data_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

    def handle_get_frame_request(self, mqtt_request):
        """
        Handle the get frame MQTT request.

        Args:
            mqtt_request (dict): The MQTT request message containing the
            camera namespace, camera name, and stream name.
        """
        self.mqtt_ros_node.ROS_DEBUG('get_frame_request message received')

        camera_namespace = mqtt_request['camera_namespace']
        camera_name = mqtt_request['camera_name']
        stream_name = mqtt_request['stream_name']
        topic_name = f'{camera_namespace}/{camera_name}/{stream_name}'
        if stream_name == 'depth' or\
           stream_name == 'infra1' or\
           stream_name == 'infra2':
            topic_name += '/image_rect_raw'
        elif stream_name == 'color':
            topic_name += '/image_raw'
        else:
            self.mqtt_ros_node.ROS_ERROR('Unsupported stream')
            return

        self.mqtt_ros_node.create_subscription(Image,
                                               topic_name,
                                               self.image_callback,
                                               self.sensor_data_qos_profile)

        mqtt_response = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'stream_name': stream_name,
            'success': True,
            'error_msg': '',
            'frame': ''
        }

        if stream_name in ['color', 'depth', 'infra1', 'infra2']:
            self.mqtt_ros_node.ROS_DEBUG('Frame sent successfully')
            mqtt_response['frame'] = str(self.image)
        else:
            self.mqtt_ros_node.ROS_WARN('Failed to get frame')
            mqtt_response['success'] = False
            mqtt_response['error_msg'] = 'unsupported type'

        self.mqtt_ros_node.mqtt_client.publish('get_frame_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('get_frame_response message sent')
        self.mqtt_ros_node.destroy_subscription(topic_name)

    def image_callback(self, msg):
        """
        Image subscription callback.

        Args:
            msg (Image): The received image message.
        """
        self.image = msg
