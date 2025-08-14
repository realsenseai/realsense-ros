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
import json
from functools import partial

from rclpy.action import ActionClient 
from realsense2_camera_msgs.action import TriggeredCalibration


class TriggeredCalibrationHandler:
    """Brdige For Triggered Calibration ROS Action."""

    def __init__(self, mqtt_ros_node):
        self.mqtt_ros_node = mqtt_ros_node
        self.action_clients = {}
        self.goal_handles = {}
    
    def get_action_client(self, camera_namespace, camera_name):
        client_name = f"{camera_namespace}_{camera_name}"
        if client_name in self.action_clients:
            return self.action_clients[client_name]
        return None 

    def set_action_client(self, camera_namespace, camera_name, client):
        client_name = f"{camera_namespace}_{camera_name}"
        self.action_clients[client_name] = client

    #assumption is that there is only one goal handle per action client.
    def get_goal_handle(self, camera_namespace, camera_name):
        client_name = f"{camera_namespace}_{camera_name}"
        if client_name in self.goal_handles:
            return self.goal_handles[client_name]
        return None 

    def set_goal_handle(self, camera_namespace, camera_name, client):
        client_name = f"{camera_namespace}_{camera_name}"
        self.goal_handles[client_name] = client

    def ros_action_send_goal(self, goal, namespace, name):
        goal_msg = TriggeredCalibration.Goal(json=goal)
        self.get_action_client(namespace,name).wait_for_server()
        self.send_goal_future = self.get_action_client(namespace,name).send_goal_async(goal_msg, feedback_callback=partial(self.ros_action_feedback_callback, namespace=namespace,name=name))
        self.send_goal_future.add_done_callback(partial(self.ros_action_goal_response_callback, namespace=namespace,name=name))

    def ros_action_goal_response_callback(self, future, namespace,name):
        goal_handle = future.result()
        self.set_goal_handle(namespace,name, goal_handle)
        if not goal_handle.accepted:
            self.mqtt_ros_node.ROS_DEBUG('TriggeredCalibrationHandler: Goal rejected')
            return

        self.mqtt_ros_node.ROS_DEBUG('TriggeredCalibrationHandler: Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(partial(self.ros_action_result_callback, namespace=namespace,name=name))

    def ros_action_result_callback(self, future, namespace,name):
        result = future.result().result
        self.mqtt_ros_node.ROS_DEBUG('ros_action_result_callback result{result}')
        self.mqtt_ros_node.ROS_DEBUG('Success: {0}'.format(result.success))
        self.mqtt_ros_node.ROS_DEBUG('Error: {0}'.format(result.error_msg))
        self.mqtt_ros_node.ROS_DEBUG('Calibration: {0}'.format(result.calibration))
        self.mqtt_ros_node.ROS_DEBUG('Health: {0}'.format(result.health))
        self.mqtt_ros_node.ROS_DEBUG('Progress: 100.0')

        mqtt_response = {
            'camera_namespace': namespace,
            'camera_name': name,
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
        self.get_action_client(namespace,name).destroy()
        self.set_action_client(namespace,name, None)
        self.set_goal_handle(namespace,name, None)

    def ros_action_feedback_callback(self, feedback_msg, namespace,name):
        feedback = feedback_msg.feedback
        self.mqtt_ros_node.ROS_DEBUG('TriggeredCalibrationHandler: Received feedback: {0}'.format(feedback.progress))
        mqtt_response = {
            'camera_namespace': namespace,
            'camera_name': name,
            'success': False,
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
        camera_namespace = mqtt_request['camera_namespace']
        camera_name = mqtt_request['camera_name']
        if mqtt_request['json'] == 'calib abort':
            #received am abort request, check if there was a pending goal.
            goal_handle = self.get_goal_handle(mqtt_request['camera_namespace'],mqtt_request['camera_name']) 
            if self.get_action_client(camera_namespace,camera_name) is None or goal_handle is None:
                self.mqtt_ros_node.ROS_DEBUG("No goal in progress, cancelled too early?")
                mqtt_response = {
                    'camera_namespace': camera_namespace,
                    'camera_name': camera_name,
                    'success': True,
                    'error_msg': "No goal in progress, cancelled too early?",
                    'calibration': '{}',
                    'health': '0',
                    'progress': 100.0
                }
                self.mqtt_ros_node.mqtt_client.publish('triggered_calibration_response',
                                               json.dumps(mqtt_response),
                                               qos=2)
            else:
                #cancel the goal in progress
                self.mqtt_ros_node.ROS_DEBUG("cancelling the goal..")
                future = goal_handle.cancel_goal_async()
        else:
            #all the requests other abort request is passed to the ActionServer 
            action_name = f'/{camera_namespace}/{camera_name}/triggered_calibration'

            self.set_action_client(mqtt_request['camera_namespace'],mqtt_request['camera_name'],ActionClient(self.mqtt_ros_node, TriggeredCalibration, action_name))
            self.ros_action_send_goal(mqtt_request['json'], camera_namespace, camera_name)
