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

import numpy as np
from rclpy.node import Node
import rclpy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
import threading
import logging
LOGGER = logging.getLogger()

''' 
This is that holds the test node that listens to a subscription created by a test.  
'''
class RSCameraSimulator(Node, threading.Thread):
    def __init__(self, namespace="camera", name='RSCameraSimulator'):
        LOGGER.debug('Creating node... /' + namespace + '/' + name)
        queue = 1
        if not rclpy.ok():
            rclpy.init()
        Node.__init__(self,namespace=namespace, node_name=name)
        threading.Thread.__init__(self)
        self._stop_event = threading.Event()
        self.color_frame = self.create_publisher(Image, '/' + namespace + '/' + name + '/color/image_raw', queue)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def run(self):
        LOGGER.debug("Thread started...")
        loop_count = 0
        while(self._stop_event.is_set() == False):
            if loop_count > 10:
                LOGGER.debug("Spinning...")
                loop_count = 0
            self.publish_frame()
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop(self):
        LOGGER.debug("Setting the stop event...")
        self._stop_event.set()
        self.join()
    def publish_frame(self):
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
