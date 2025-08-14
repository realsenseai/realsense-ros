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


from realsense2_camera_msgs.action import TriggeredCalibration
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
#from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
'''
humble doesn't have the SetParametersResult and SetParameters_Response imported using 
__init__.py. The below two lines can be used for iron and hopefully succeeding ROS2 versions
'''
#from rcl_interfaces.msg import SetParametersResult
#from rcl_interfaces.srv import SetParameters_Response
from rcl_interfaces.msg._set_parameters_result import SetParametersResult
from rcl_interfaces.srv._set_parameters  import SetParameters_Response

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue

import os
import sys
import logging
#LOGGER = logging.getLogger(__name__)
LOGGER = logging.getLogger()
import time

class CameraClient(Node):
    def __init__(self, testname, namespace="robot1", name='c_333622320031'):
        if not rclpy.ok():
            rclpy.init()
        super().__init__(testname)
    
    def wait_for_node(self, node_name, timeout=8.0):
        start = time.time()
        flag = False
        print('Waiting for node... ' + node_name)
        while time.time() - start < timeout:
            print(node_name + ": waiting for the node to come up")
            flag = node_name in self.get_node_names()
            if flag:
                return True, ""
            time.sleep(timeout/5)
        return False, "Timed out waiting for "+ str(timeout)+  "seconds"

    def create_service_client_ifs(self, camera_name):
        self.set_param_if = self.create_client(SetParameters, camera_name + '/set_parameters')
        self.get_param_if = self.create_client(GetParameters, camera_name + '/get_parameters')
        while not self.get_param_if.wait_for_service(timeout_sec=1.0):
            LOGGER.info('service not available, waiting again...') 
        while not self.set_param_if.wait_for_service(timeout_sec=1.0):
            LOGGER.info('service not available, waiting again...') 

    def send_param(self, req):
        future = self.set_param_if.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                response = future.result()
                if response.results[0].successful:
                    return True
        return False

    def get_param(self, req):
        future = self.get_param_if.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                response = future.result()
                return response.values[0]
        return None
    
    def set_integer_param(self, param_name, param_value):
        req = SetParameters.Request()
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=param_value)
        req.parameters = [Parameter(name=param_name, value=new_param_value)]
        return self.send_param(req)

    def get_integer_param(self, param_name):
        req = GetParameters.Request()
        req.names = [param_name]
        value = self.get_param(req)
        if (value == None) or (value.type != ParameterType.PARAMETER_INTEGER):
            return None
        else:
            return value.integer_value
        
    def set_bool_param(self, param_name, param_value):
        req = SetParameters.Request()
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=param_value)
        req.parameters = [Parameter(name=param_name, value=new_param_value)]
        return self.send_param(req)
    
    def get_bool_param(self, param_name):
        req = GetParameters.Request()
        req.names = [param_name]
        value = self.get_param(req)
        if (value == None) or (value.type != ParameterType.PARAMETER_BOOL):
            return None
        else:
            return value.bool_value
        

class TriggeredCalibrationCameraClient(CameraClient):
    def __init__(self, namespace="robot1", name='c_333622320031'):
        testname = name+"Test"
        super().__init__(testname, namespace, name)
        action_name = f'/{namespace}/{name}/triggered_calibration'
        self.action_client = ActionClient(self, TriggeredCalibration, action_name)
        self.create_service_client_ifs(f'/{namespace}/{name}')
        self.calibration_result = None


    def prepare_for_calibration(self):
        self.set_integer_param('safety_camera.safety_mode',2)
        time.sleep(0.5)
        LOGGER.info("Param safety_camera.safety_mode: %s", str(self.get_integer_param('safety_camera.safety_mode')))
        # switch to visual preset #1
        self.set_integer_param('depth_module.visual_preset',1)
        time.sleep(0.5)
        LOGGER.info("Param depth_module.visual_preset: %d", self.get_integer_param('depth_module.visual_preset'))

        # enable emitter
        self.set_bool_param('depth_module.emitter_enabled',True)
        time.sleep(0.5)
        LOGGER.info("Param depth_module.emitter_enabled: %d", self.get_bool_param('depth_module.emitter_enabled'))

        # enable auto exposuretc_done   CAMERA_NAME,
        self.set_bool_param( 'depth_module.enable_auto_exposure',True)
        time.sleep(0.5)
        LOGGER.info("Param depth_module.enable_auto_exposure: %s", self.get_bool_param('depth_module.enable_auto_exposure'))

        # turn off depth streaming
        self.set_bool_param('enable_depth',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_depth: %s", self.get_bool_param('enable_depth'))
        
        # turn off infra1 streaming
        self.set_bool_param('enable_infra1',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_infra1: %s", self.get_bool_param('enable_infra1'))
        # turn off infra2 streaming
        self.set_bool_param('enable_infra2',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_infra2: %s", self.get_bool_param('enable_infra2'))
        # turn off safety streaming
        self.set_bool_param('enable_safety',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_safety: %s", self.get_bool_param('enable_safety'))
        # turn off occupancy streaming
        self.set_bool_param('enable_occupancy',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_occupancy: %s", self.get_bool_param('enable_occupancy'))

        self.set_bool_param('enable_color',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_color: %s", self.get_bool_param('enable_color'))

        # turn off lpcl streaming
        self.set_bool_param('enable_labeled_point_cloud',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_labeled_point_cloud: %s", self.get_bool_param('enable_labeled_point_cloud'))

    def start_calibration(self, with_feedback=True):
        self.tc_done = False
        goal_msg = TriggeredCalibration.Goal()
        LOGGER.info(goal_msg)
        self.action_client.wait_for_server()
        if with_feedback ==True:
            self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.ros_action_feedback_callback)
        else:
            self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.ros_action_goal_response_callback)

    def ros_action_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        LOGGER.info('TriggeredCalibrationHandler: Received feedback: {0}'.format(feedback.progress))

    def ros_action_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            LOGGER.info('TriggeredCalibrationHandler: Goal rejected')
            return
        LOGGER.info('TriggeredCalibrationHandler: Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.ros_action_result_callback)

    def ros_action_result_callback(self, future):
        result = future.result().result
        LOGGER.info('Success: {0}'.format(result.success))
        LOGGER.info('Error: {0}'.format(result.error_msg))
        LOGGER.info('Calibration: {0}'.format(result.calibration))
        LOGGER.info('Health: {0}'.format(result.health))
        LOGGER.info('Progress: 100.0')
        self.tc_done = True
        self.calibration_result = result

    def abort_calibration(self):
        self.tc_done = False
        goal_msg = TriggeredCalibration.Goal()
        goal_msg.json = "calib abort"
        LOGGER.info(goal_msg)
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.ros_action_abort_goal_response_callback)

    def ros_action_abort_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            LOGGER.error('TriggeredCalibrationHandler: abort rejected')
            return
        LOGGER.info('TriggeredCalibrationHandler: abort accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.ros_action_abort_result_callback)

    def ros_action_abort_result_callback(self, future):
        result = future.result().result
        LOGGER.info('Result: {0}'.format(result))
        self.tc_done = True
        self.calibration_result = result
    def dryrun_calibration(self):
        self.tc_done = False
        goal_msg = TriggeredCalibration.Goal()
        goal_msg.json = "calib dry run"
        LOGGER.info(goal_msg)
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.ros_action_dryrun_feedback_callback)
        self.send_goal_future.add_done_callback(self.ros_action_dryrun_goal_response_callback)

    def ros_action_dryrun_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        LOGGER.info('TriggeredCalibrationHandler: Received feedback for dryrun: {0}'.format(feedback.progress))

    def ros_action_dryrun_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            LOGGER.info('TriggeredCalibrationHandler: Goal for dryrun rejected')
            return
        LOGGER.info('TriggeredCalibrationHandler: Goal for dryrun accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.ros_action_dryrun_result_callback)

    def ros_action_dryrun_result_callback(self, future):
        result = future.result().result
        LOGGER.info('Success: {0}'.format(result.success))
        LOGGER.info('Error: {0}'.format(result.error_msg))
        LOGGER.info('Calibration: {0}'.format(result.calibration))
        LOGGER.info('Health: {0}'.format(result.health))
        LOGGER.info('Progress: 100.0')
        self.tc_done = True
        self.calibration_result = result

