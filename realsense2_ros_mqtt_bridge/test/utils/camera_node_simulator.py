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

import collections
import time
import threading
import numpy as np
from rclpy.node import Node
import rclpy

from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image

from realsense2_camera_msgs.srv import SafetyPresetRead
from realsense2_camera_msgs.srv import SafetyPresetWrite
from realsense2_camera_msgs.srv import SafetyInterfaceConfigRead
from realsense2_camera_msgs.srv import SafetyInterfaceConfigWrite
from realsense2_camera_msgs.srv import CalibConfigRead
from realsense2_camera_msgs.srv import CalibConfigWrite
from realsense2_camera_msgs.srv import ApplicationConfigRead
from realsense2_camera_msgs.srv import ApplicationConfigWrite
from realsense2_camera_msgs.srv import DeviceInfo
from realsense2_camera_msgs.action import TriggeredCalibration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

import logging
LOGGER = logging.getLogger()

''' 
This is that holds the test node that listens to a subscription created by a test.  
'''
class RSCameraSimulator(Node, threading.Thread):
    def __init__(self, namespace="camera", name='RSCameraSimulator'):
        LOGGER.debug('Creating node... /' + namespace + '/' + name)
        if not rclpy.ok():
            rclpy.init()
        Node.__init__(self,namespace=namespace, node_name=name)
        threading.Thread.__init__(self)
        self._stop_event = threading.Event()
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.color_frame = None
        self.depth_frame = None
        self.infra1_frame = None
        self.infra2_frame = None
        self.namespace = namespace 
        self.name = name
        self.goal_queue = collections.deque()
        self.goal_queue_lock = threading.Lock()
        self.current_goal = None

    def run(self):
        LOGGER.debug("Thread started...")
        loop_count = 0
        executor = MultiThreadedExecutor()
        while(self._stop_event.is_set() == False):
            if loop_count > 10:
                LOGGER.debug("Spinning...")
                loop_count = 0
            if self.color_frame != None:
                self.publish_color_frame()
            if self.depth_frame != None:
                self.publish_depth_frame()
            if self.infra1_frame != None:
                self.publish_infra1_frame()
            if self.infra2_frame != None:
                self.publish_infra2_frame()
            rclpy.spin_once(self, timeout_sec=0.01, executor=executor)

        LOGGER.info("destroying the publisher")
        if self.color_frame != None:
            self.destroy_publisher(self.color_frame)
        if self.depth_frame != None:
            self.destroy_publisher(self.depth_frame)
        if self.infra1_frame != None:
            self.destroy_publisher(self.infra1_frame)
        if self.infra2_frame != None:
            self.destroy_publisher(self.infra2_frame)
        self.destroy_node()


    def stop(self):
        LOGGER.debug("Setting the stop event...")
        self._stop_event.set()
        self.join()

    def publish_color_frame(self):
        msg = Image()
        frame = np.zeros((4,4,0), dtype=int)
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'test'
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding = "rgb"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] * np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()
        # publishes message
        # LOGGER.debug("Publishing color frame...")
        self.color_frame.publish(msg)

    def publish_depth_frame(self):
        msg = Image()
        frame = np.zeros((4,4,0), dtype=int)
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'test'
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding = "rgb"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] * np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()
        # publishes message
        # LOGGER.debug("Publishing depth frame...")
        self.depth_frame.publish(msg)

    def publish_infra1_frame(self):
        msg = Image()
        frame = np.zeros((4,4,0), dtype=int)
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'test'
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding = "rgb"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] * np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()
        # publishes message
        # LOGGER.debug("Publishing infra1 frame...")
        self.infra1_frame.publish(msg)

    def publish_infra2_frame(self):
        msg = Image()
        frame = np.zeros((4,4,0), dtype=int)
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'test'
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding = "rgb"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] * np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()
        # publishes message
        # LOGGER.debug("Publishing infra2 frame...")
        self.infra2_frame.publish(msg)

    def start_publish_color_frame(self):
        if self.color_frame != None:
            LOGGER.warning(f'Color frame is already being published..')
            return
        queue = 1
        self.color_frame = self.create_publisher(Image, '/' + self.namespace + '/' + self.name + '/color/image_raw', queue)


    def start_publish_depth_frame(self):
        if self.depth_frame != None:
            LOGGER.warning(f'Depth frame is already being published..')
            return
        queue = 1
        self.depth_frame = self.create_publisher(Image, '/' + self.namespace + '/' + self.name + '/depth/image_rect_raw', queue)


    def start_publish_infra1_frame(self):
        if self.infra1_frame != None:
            LOGGER.warning(f'Infra1 frame is already being published..')
            return
        queue = 1
        self.infra1_frame = self.create_publisher(Image, '/' + self.namespace + '/' + self.name + '/infra1/image_rect_raw', queue)

    def start_publish_infra2_frame(self):
        if self.infra2_frame != None:
            LOGGER.warning(f'Infra2 frame is already being published..')
            return
        queue = 1
        self.infra2_frame = self.create_publisher(Image, '/' + self.namespace + '/' + self.name + '/infra2/image_rect_raw', queue)

    def add_parameters(self, params):
        try:
            for param in params:
                self.declare_parameter(param['param_name'], param['default_value'])
        except Exception as e:
            LOGGER.warning(f'An unexpected error occurred: {e}')

    def parameter_callback(self, params):
        LOGGER.debug("Params changed: " + str(params))
        for param in params:
            if param.type_ == rclpy.Parameter.Type.INTEGER:
                if param.name == 'safety_camera.safety_mode':
                    self.safety_camera_safety_mode = param.value
                    LOGGER.info("Safety mode changed to " + str(self.safety_camera_safety_mode))
                else:
                    LOGGER.info(param.name + " (int)value changed to " + str(param.value))
            elif param.type_ == rclpy.Parameter.Type.STRING:
                LOGGER.info(param.name + " (string)value changed to " + str(param.value))
            elif param.type_ == rclpy.Parameter.Type.BOOL:
                LOGGER.info(param.name + " (bool)value changed to " + str(param.value))
            else:
                LOGGER.warning("Unexpected param type: " + str(param.type_) + ". "+  param.name + " (bool)value changed to " + str(param.value))
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)
    
    def create_safety_preset_service(self):
        service_name = f'/{self.namespace}/{self.name}/safety_preset_read'
        self.safety_preset_read_srv = self.create_service(SafetyPresetRead, service_name, self.safety_preset_read_cb)
        service_name = f'/{self.namespace}/{self.name}/safety_preset_write'
        self.safety_preset_write_srv = self.create_service(SafetyPresetWrite, service_name, self.safety_preset_write_cb)
        self.safety_preset = ["Uninitialized"] * 64
    
    def safety_preset_read_cb(self, request, response):
        LOGGER.info(f'Safety preset read for index {request.index}')
        response.success = True
        response.error_message = ''
        if self.safety_preset[request.index] == None:
            response.safety_preset =  str(request.index)
        else:
            response.safety_preset =  self.safety_preset[request.index]
        return response

    def safety_preset_write_cb(self, request, response):
        LOGGER.info(f'Safety preset write for index {request.index} with data {request.safety_preset}')
        response.success = True
        response.error_message = ''
        self.safety_preset[request.index] = request.safety_preset
        return response

    def create_safety_interface_config_service(self):
        service_name = f'/{self.namespace}/{self.name}/safety_interface_config_read'
        self.safety_interface_config_read_srv = self.create_service(SafetyInterfaceConfigRead, service_name, self.safety_interface_config_read_cb)
        service_name = f'/{self.namespace}/{self.name}/safety_interface_config_write'
        self.safety_interface_config_write_srv = self.create_service(SafetyInterfaceConfigWrite, service_name, self.safety_interface_config_write_cb)
        self.safety_interface_config = ["Uninitialized"] * 3
    
    def safety_interface_config_read_cb(self, request, response):
        LOGGER.info(f'Safety interface config read for calib location {request.calib_location}')
        response.success = True
        response.error_message = ''
        response.safety_interface_config =  self.safety_interface_config[request.calib_location]
        return response

    def safety_interface_config_write_cb(self, request, response):
        LOGGER.info(f'Safety interface config write with data {request.safety_interface_config}')
        response.success = True
        response.error_message = ''
        self.safety_interface_config[2] = request.safety_interface_config
        return response

    def create_calib_config_service(self):
        service_name = f'/{self.namespace}/{self.name}/calib_config_read'
        self.calib_config_read_srv = self.create_service(CalibConfigRead, service_name, self.calib_config_read_cb)
        service_name = f'/{self.namespace}/{self.name}/calib_config_write'
        self.calib_config_srv = self.create_service(CalibConfigWrite, service_name, self.calib_config_write_cb)
        self.calib_config = "Uninitialized"
    
    def calib_config_read_cb(self, request, response):
        LOGGER.info(f'Calib config read called {response}')
        response.success = True
        response.error_message = ''
        response.calib_config =  self.calib_config
        return response

    def calib_config_write_cb(self, request, response):
        LOGGER.info(f'Calibration config write data {request.calib_config}')
        response.success = True
        response.error_message = ''
        self.calib_config = request.calib_config
        return response

    def create_application_config_service(self):
        service_name = f'/{self.namespace}/{self.name}/application_config_read'
        self.application_config_read_srv = self.create_service(ApplicationConfigRead, service_name, self.application_config_read_cb)
        service_name = f'/{self.namespace}/{self.name}/application_config_write'
        self.application_config_srv = self.create_service(ApplicationConfigWrite, service_name, self.application_config_write_cb)
        self.application_config = "Uniinitialized"
    
    def application_config_read_cb(self, request, response):
        LOGGER.info(f'Application config read called')
        response.success = True
        response.error_message = ''
        response.application_config =  self.application_config
        return response

    def application_config_write_cb(self, request, response):
        LOGGER.info(f'Application config write with data {request.application_config}')
        response.success = True
        response.error_message = ''
        self.application_config = request.application_config
        return response

    def create_device_info_service(self):
        LOGGER.info(f'Created the device info service')
        service_name = f'/{self.namespace}/{self.name}/device_info'
        self.device_info_srv = self.create_service(DeviceInfo, service_name, self.device_info_read_cb)

    
    def device_info_read_cb(self, request, response):
        LOGGER.debug(f'DeviceInfo read called \n{request} \n{response}')
        response.device_name = "Camera" #string device_name
        response.serial_number = "1234" #string serial_number
        response.firmware_version = "1.2.3" #string firmware_version
        response.usb_type_descriptor = "USB 3.1" #string usb_type_descriptor
        response.firmware_update_id = "1" #string firmware_update_id
        response.sensors = "color"#string sensors
        response.physical_port = "1" #string physical_port
        LOGGER.info(f'DeviceInfo response: {response}')
        return response
    
    def create_triggered_calibration_action(self):
        action_name = f'/{self.namespace}/{self.name}/triggered_calibration'
        '''
        self._action_server = ActionServer(
            self,
            TriggeredCalibration,
            action_name,
            self.triggered_calibration_handler)
        '''
        self._action_server = ActionServer(
            self,
            TriggeredCalibration,
            action_name,
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.triggered_calibration_handler,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
    def handle_accepted_callback(self, goal_handle):
        """Start or defer execution of an already accepted goal."""
        with self.goal_queue_lock:
            if self.current_goal is not None:
                # Put incoming goal in the queue
                self.goal_queue.append(goal_handle)
                LOGGER.info('Goal put in the queue')
            else:
                # Start goal execution right away
                self.current_goal = goal_handle
                LOGGER.info('Start the execution')
                self.current_goal.execute()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        LOGGER.info(f'Received goal request {goal_request}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        LOGGER.info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def triggered_calibration_handler(self, goal_handle):
        LOGGER.info(f'Calibration request: {goal_handle.request.json}')
        """Execute a goal."""
        try:
            LOGGER.info('Executing goal...')
            # Append the seeds for the Fibonacci sequence
            feedback_msg = TriggeredCalibration.Feedback()

            # Start executing the action
            for i in range(1, 100):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    LOGGER.info('Goal canceled')
                    result = TriggeredCalibration.Result()
                    result.success = False
                    result.error_msg = 'Canceled'
                    result.calibration = '{}'
                    LOGGER.warning('Calibration aborted and senting status success and error message aborted. This needs to be ratified after understanding the actual implementation')
                    return result

                # Update Fibonacci sequence
                feedback_msg.progress = float(i)

                LOGGER.info(
                    'Publishing feedback: {0}'.format(feedback_msg.progress))

                # Publish the feedback
                goal_handle.publish_feedback(feedback_msg)

                # Sleep for demonstration purposes
                time.sleep(0.01)

            goal_handle.succeed()

            # Populate result message
            result = TriggeredCalibration.Result()
            result.success = True
            result.calibration = f'{goal_handle.request.json}'

            LOGGER.info(
                'Returning result: {0}'.format(result))

            return result
        finally:
            with self.goal_queue_lock:
                try:
                    # Start execution of the next goal in the queue.
                    self.current_goal = self.goal_queue.popleft()
                    LOGGER.info('Next goal pulled from the queue')
                    self.current_goal.execute()
                except IndexError:
                    # No goal in the queue.
                    self.current_goal = None
    
if __name__ == '__main__':
    rclpy.init()
    import os
    namespace = 'camera'
    name = 'camera'
    camera = RSCameraSimulator(namespace, name)
    camera.start()
    os.system("ros2 node list")
    import time
    rclpy.spin(camera)
    time.sleep(10)
    rclpy.shutdown()
    camera.stop()
    LOGGER.info("Test completed")
