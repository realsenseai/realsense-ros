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
import json
from rclpy.action import ActionClient 
from realsense2_camera_msgs.action import TriggeredCalibration

class TriggeredCalibrationHandler:
    """Brdige For Triggered Calibration ROS Action."""

    def __init__(self, mqtt_ros_node):
        self.action_client = None
        self.camera_namespace = None
        self.camera_name = None
        self.mqtt_ros_node = mqtt_ros_node

    def ros_action_send_goal(self):
        goal_msg = TriggeredCalibration.Goal()
        self.action_client.wait_for_server()
        self.send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.ros_action_feedback_callback)
        self.send_goal_future.add_done_callback(self.ros_action_goal_response_callback)

    def ros_action_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.mqtt_ros_node.ROS_DEBUG('TriggeredCalibrationHandler: Goal rejected')
            return

        self.mqtt_ros_node.ROS_DEBUG('TriggeredCalibrationHandler: Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.ros_action_result_callback)

    def ros_action_result_callback(self, future):
        result = future.result().result
        self.mqtt_ros_node.ROS_DEBUG('Success: {0}'.format(result.success))
        self.mqtt_ros_node.ROS_DEBUG('Error: {0}'.format(result.error_msg))
        self.mqtt_ros_node.ROS_DEBUG('Calibration: {0}'.format(result.calibration))
        self.mqtt_ros_node.ROS_DEBUG('Health: {0}'.format(result.health))
        self.mqtt_ros_node.ROS_DEBUG('Progress: 100.0')

        mqtt_response = {
            'camera_namespace': self.camera_namespace,
            'camera_name': self.camera_name,
            'success': result.success,
            'error_msg': result.error_msg,
            'calibration': result.calibration,
            'health': result.health,
            'progress': 100.0
        }
        self.mqtt_ros_node.mqtt_client.publish('triggered_calibration_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('triggered_calibration_response message sent')
        self._action_client.destroy()

    def ros_action_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.mqtt_ros_node.ROS_DEBUG('TriggeredCalibrationHandler: Received feedback: {0}'.format(feedback.progress))
        mqtt_response = {
            'camera_namespace': self.camera_namespace,
            'camera_name': self.camera_name,
            'success': 'false',
            'error_msg': '',
            'calibration': '{}',
            'health': '0',
            'progress': feedback.progress
        }
        self.mqtt_ros_node.mqtt_client.publish('triggered_calibration_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
        self.mqtt_ros_node.ROS_DEBUG('triggered_calibration_response message sent')

    def handle_triggered_calibration_request(self, mqtt_request):
        self.mqtt_ros_node.ROS_DEBUG('triggered_calibration_request \
            message received')

        self._camera_namespace = mqtt_request['camera_namespace']
        self._camera_name = mqtt_request['camera_name']
        action_name = f'/{self.camera_namespace}/{self.camera_name}/triggered_calibration'

        self.action_client = ActionClient(self.mqtt_ros_node, TriggeredCalibration, action_name)
        self.ros_action_send_goal()
