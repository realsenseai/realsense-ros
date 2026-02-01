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
import random

import paho.mqtt.client as mqtt

from rclpy.node import Node

from .device_handler import DeviceHandler
from .frame_handler import FrameHandler
from .parameter_handler import ParameterHandler
from .safety_preset_handler import SafetyPresetHandler
from .safety_interface_config_handler import SafetyInterfaceConfigHandler
from .calib_config_handler import CalibConfigHandler
from .application_config_handler import ApplicationConfigHandler
from .hw_reset_handler import HwResetHandler
from .hwm_command_handler import HWMCommandHandler
from .triggered_calibration_handler import TriggeredCalibrationHandler
from .transformation_handler import TransformationHandler

class MQTTBridgeNode(Node):
    """
    MQTT <-> ROS brdige node.

    MQTTBridgeNode is a ROS Node that acts as a MQTT <-> ROS brdige for
    RealSense cameras. The node listens to pre-defined MQTT Broker
    messages, translates them into a ROS ones, handling these messages against
    the realsense2_camera node and then sending the response back to the
    MQTT Broker
    """

    def __init__(self,
                 default_mqtt_broker_ip='localhost',
                 default_mqtt_broker_port=1883):
        """
        Initialize the MQTTBridgeNode.

        Args:
            default_mqtt_broker_ip (str): Default MQTT broker IP address.
            default_mqtt_broker_port (int): Default MQTT broker port.
        """
        super().__init__('realsense_ros_mqtt_bridge_node')
        self.setup_node_parameters(default_mqtt_broker_ip, default_mqtt_broker_port)
        self.setup_mqtt_topics()
        self.setup_mqtt_handlers()
        self.setup_mqtt_connection()

    def setup_node_parameters(self, default_mqtt_broker_ip, default_mqtt_broker_port):
        """
        Declare and set node parameters.

        Args:
            default_mqtt_broker_ip (str): Default MQTT broker IP address.
            default_mqtt_port (int): Default MQTT broker port.
        """
        try:
            self.declare_parameter('broker_ip', default_mqtt_broker_ip)
            self.broker_ip = str(self.get_parameter(
                'broker_ip').get_parameter_value().string_value)

            self.declare_parameter('broker_port', default_mqtt_broker_port)
            self.broker_port = self.get_parameter(
                'broker_port').get_parameter_value().integer_value
        except Exception as e:
            print(f'An unexpected error occurred: {e}')

    def setup_mqtt_connection(self):
        """Create MQTT client and connect it to MQTT broker."""
        self.mqtt_client_id = (f'realsense_ros_mqtt_bridge_node-'
                               f'{random.randint(0, 1000)}')
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1,
                                       self.mqtt_client_id)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.ROS_INFO(f'Trying to connect to MQTT broker at '
                          f'ip:{self.broker_ip} port:{self.broker_port}')
            self.mqtt_client.connect(self.broker_ip, self.broker_port)
            self.mqtt_client.loop_start()
            self.ROS_INFO('Successfully connected to MQTT Broker')
        except Exception as e:
            self.ROS_ERROR(f'Cannot connect to Broker ip:\
                {self.broker_ip} port:{self.broker_port}')
            self.ROS_ERROR(f'An exception occurred: {e}')
            raise e

    def setup_mqtt_topics(self):
        """Define the topics our MQTTBridgeNode needs to listen to."""
        self.mqtt_requests_topics = [
            'enumerate_devices_request',
            'send_hw_reset_request',
            'send_hwm_command_request',
            'get_device_info_request',
            'get_transformation_request',
            'get_param_request',
            'set_param_request',
            'get_frame_request',
            'get_safety_preset_request',
            'set_safety_preset_request',
            'get_safety_interface_config_request',
            'set_safety_interface_config_request',
            'get_calib_config_request',
            'set_calib_config_request',
            'get_application_config_request',
            'set_application_config_request',
            'hwm_command_send_request',
            'triggered_calibration_request'
        ]

    def setup_mqtt_handlers(self):
        """
        Create handlers for each supported MQTT request type.

        e.g., device_handler will handle enumerate device MQTT requests,
        which arrive on the 'enumerate_devices_request' topic
        """
        self.device_handler = DeviceHandler(self)
        self.transformation_handler = TransformationHandler(self)
        self.parameter_handler = ParameterHandler(self)
        self.frame_handler = FrameHandler(self)
        self.safety_preset_handler = SafetyPresetHandler(self)
        self.safety_interface_config_handler = SafetyInterfaceConfigHandler(self)
        self.calib_config_handler = CalibConfigHandler(self)
        self.application_config_handler = ApplicationConfigHandler(self)
        self.hw_reset_handler = HwResetHandler(self)
        self.hwm_command_handler = HWMCommandHandler(self)
        self.triggered_calibration_handler = TriggeredCalibrationHandler(self)

    def ROS_INFO(self, msg):  # pylint: disable=invalid-name
        """
        Log an info message.

        Args:
            msg (str): Message to log.
        """
        self.get_logger().info(msg)

    def ROS_DEBUG(self, msg):  # pylint: disable=invalid-name
        """
        Log a debug message.

        Args:
            msg (str): Message to log.
        """
        self.get_logger().debug(msg)

    def ROS_WARN(self, msg):  # pylint: disable=invalid-name
        """
        Log a warning message.

        Args:
            msg (str): Message to log.
        """
        self.get_logger().warn(msg)

    def ROS_ERROR(self, msg):  # pylint: disable=invalid-name
        """
        Log an error message.

        Args:
            msg (str): Message to log.
        """
        self.get_logger().error(msg)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """
        Callback for when the MQTT client connects to the broker.

        Args:
            client: The MQTT client instance.
            userdata: User-defined data of any type that is
                      passed to the callbacks.
            flags: Response flags sent by the broker.
            rc: The connection result.
        """
        del client, userdata, flags, rc  # delete unsed params
        self.ROS_DEBUG('on_mqtt_connect')
        self.ROS_INFO('Trying to subscribe to requested MQTT topics')
        for topic in self.mqtt_requests_topics:
            self.mqtt_client.subscribe(topic)
        self.ROS_INFO('Successfully subscribed to requested MQTT topics')
        self.ROS_INFO('Waiting for new incoming MQTT requets')

    def on_mqtt_message(self, client, userdata, msg):
        """
        Callback for when a PUBLISH message is received from the server.

        Args:
            client:     The MQTT client instance.
            userdata:   User-defined data of any type that is passed
                        to the callbacks.
            msg:        An instance of MQTTMessage, which has members
                        topic, payload, qos, retain.
        """
        del client, userdata  # delete unsed params
        decoded_msg = msg.payload.decode('utf-8')
        self.ROS_INFO(f'Received new MQTT request on topic:'
                      f'{msg.topic} msg:{decoded_msg}')
        mqtt_request = json.loads(decoded_msg)
        err_msg = ''
        response_topic = ''
        if msg.topic == 'enumerate_devices_request':
            if 'camera_namespace_prefix' not in mqtt_request:
                err_msg = "camera_namespace_prefix not found in the mqtt request"
            elif 'camera_name_prefix' not in mqtt_request:
                err_msg = "camera_name_prefix not found in the mqtt request"
        elif msg.topic != 'get_transformation_request':
            if 'camera_namespace' not in mqtt_request:
                err_msg = "camera_namespace not found in the mqtt request"
            elif 'camera_name' not in mqtt_request:
                err_msg = "camera_name not found in the mqtt request"
            if msg.topic == 'get_param_request':
                response_topic = 'get_parameter_response'
            if msg.topic == 'set_param_request':
                response_topic = 'set_parameter_response'

        if err_msg == '':
            if msg.topic == 'enumerate_devices_request':
                self.device_handler.handle_enumerate_devices_request(mqtt_request)
            elif msg.topic == 'get_transformation_request':
                if 'source' not in mqtt_request:
                    err_msg = "source not found in the mqtt request"
                elif 'destination' not in mqtt_request:
                    err_msg = "destination not found in the mqtt request"
                else:
                    self.transformation_handler.handle_get_transformation_request(mqtt_request)
            elif msg.topic == 'send_hw_reset_request':
                self.hw_reset_handler.handle_hw_reset_send_request(mqtt_request)
            elif msg.topic == 'send_hwm_command_request':
                if 'opcode' not in mqtt_request:
                    err_msg = "opcode not found in the mqtt request"
                else:
                    self.hwm_command_handler.handle_hwm_command_send_request(mqtt_request)
            elif msg.topic == 'get_device_info_request':
                self.device_handler.handle_get_device_info_request(mqtt_request)
            elif msg.topic == 'get_param_request':
                if 'parameter_name' not in mqtt_request:
                    err_msg = "parameter_name not found in the mqtt request"
                    response_topic = 'get_parameter_response'
                else:
                    self.parameter_handler.handle_get_param_request(mqtt_request)
            elif msg.topic == 'set_param_request':
                response_topic = 'set_parameter_response'
                if 'parameter_name' not in mqtt_request:
                    err_msg = "parameter_name not found in the mqtt request"
                elif 'parameter_value' not in mqtt_request:
                    err_msg = "parameter_value not found in the mqtt request"
                elif 'parameter_type' not in mqtt_request:
                    err_msg = "parameter_type not found in the mqtt request"
                else:
                    self.parameter_handler.handle_set_param_request(mqtt_request)
            elif msg.topic == 'get_frame_request':
                if 'stream_name' not in mqtt_request:
                    err_msg = "stream_name not found in the mqtt request"
                else:
                    self.frame_handler.handle_get_frame_request(mqtt_request)
            elif msg.topic == 'get_safety_preset_request':
                if 'index' not in mqtt_request:
                    err_msg = "index not found in the mqtt request"
                else:
                    self.safety_preset_handler.handle_get_safety_preset_request(
                        mqtt_request)
            elif msg.topic == 'set_safety_preset_request':
                if 'index' not in mqtt_request:
                    err_msg = "index not found in the mqtt request"
                elif 'safety_preset' not in mqtt_request:
                    err_msg = "preset not found in the mqtt request"
                else:
                    self.safety_preset_handler.handle_set_safety_preset_request(
                        mqtt_request)
            elif msg.topic == 'get_safety_interface_config_request':
                if 'calib_location' not in mqtt_request:
                    mqtt_request['calib_location'] = 2
                    self.ROS_WARN('calib_location  was not found in MQTT request, using the default value 2')
                self.safety_interface_config_handler.handle_get_safety_interface_config_request(
                    mqtt_request)
            elif msg.topic == 'set_safety_interface_config_request':
                if 'safety_interface_config' not in mqtt_request:
                    err_msg = "safety_interface_config not found in the mqtt request"
                else:
                    self.safety_interface_config_handler.handle_set_safety_interface_config_request(
                        mqtt_request)
            elif msg.topic == 'get_calib_config_request':
                self.calib_config_handler.handle_get_calib_config_request(
                    mqtt_request)
            elif msg.topic == 'set_calib_config_request':
                if 'calib_config' not in mqtt_request:
                    err_msg = "calib_config not found in the mqtt request"
                else:
                    self.calib_config_handler.handle_set_calib_config_request(
                        mqtt_request)
            elif msg.topic == 'get_application_config_request':
                self.application_config_handler.handle_get_application_config_request(
                    mqtt_request)
            elif msg.topic == 'set_application_config_request':
                if 'application_config' not in mqtt_request:
                    err_msg = "application_config not found in the mqtt request"
                else:
                    self.application_config_handler.handle_set_application_config_request(
                        mqtt_request)
            elif msg.topic == 'triggered_calibration_request':
                #default is 'calib run'
                if 'json' not in mqtt_request:
                    mqtt_request['json'] = 'calib run'
                self.triggered_calibration_handler.handle_triggered_calibration_request(
                    mqtt_request)
            else:
                err_msg = 'Unsupported MQTT Message'

        if err_msg == '':
            self.ROS_INFO('MQTT request handling done')
            self.ROS_INFO('Waiting for new incoming MQTT requets')
        else:
            mqtt_response = mqtt_request
            mqtt_response['success'] = False
            mqtt_response['error_msg'] = err_msg
            if response_topic == '':
                response_topic = msg.topic.replace('request', 'response')
            self.mqtt_client.publish(response_topic,
                    json.dumps(mqtt_response),
                    qos=1)
            self.ROS_ERROR(err_msg + f' Response in {response_topic}:{mqtt_response}')

            

    def wait_for_service(self, client, service_name, timeout=5.0):
        """
        Wait for a ROS service to become available.

        Args:
            client:             The service client instance.
            service_name (str): The name of the service to wait for.
            timeout (float):    Time to wait for the service to
                                become available.

        Returns:
            bool: True if the service is available, False otherwise.
        """
        if not client.wait_for_service(timeout):
            self.ROS_ERROR(f'Service not available: {service_name}')
            return False
        self.ROS_DEBUG(f'Service available: {service_name}')
        return True
