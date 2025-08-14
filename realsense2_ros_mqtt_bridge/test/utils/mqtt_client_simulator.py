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
"""reused from minimal_mqtt_client example to mimic the SDS MQTT client functionality"""
import json
import random
import time
import logging
LOGGER = logging.getLogger()

from paho.mqtt import client as paho_mqtt_client

class MQTTClientSimulator:
    def __init__(self, mqtt_broker_ip, mqtt_broker_port):
        """
        Initialize the client.

        Args:
            mqtt_broker_ip: The IP address of the MQTT broker.
            mqtt_broker_port: The port number of the MQTT broker.
        """
        self.mqtt_broker_ip = mqtt_broker_ip
        self.mqtt_broker_port = mqtt_broker_port

        # Generate a Client ID with a random user-id suffix.
        mqtt_client_id = f'mqtt-client-user-{random.randint(0, 1000)}'

        self.mqtt_client = paho_mqtt_client.Client(
            paho_mqtt_client.CallbackAPIVersion.VERSION1, mqtt_client_id)
        # client.username_pw_set(username, password)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.connect(mqtt_broker_ip, mqtt_broker_port)
        self.tc_done = False
        self.msg = dict()
        self.locked = False

    def on_connect(self, client, userdata, flags, rc):
        """
        Handle the MQTT connection event.

        Args:
            client: The MQTT client instance.
            userdata: The user data associated with the connection.
            flags: The connection flags.
            rc: The result code of the connection attempt.
        """
        del client, userdata, flags  # delete unused params
        if rc == 0:
            LOGGER.info(f'Connected to MQTT Broker on'
                  f' ip:{ self.mqtt_broker_ip} port:{self.mqtt_broker_port}')
        else:
            LOGGER.warning(f'Could not Connect to MQTT Broker on'
                  f' ip:{ self.mqtt_broker_ip} port:{self.mqtt_broker_port}'
                  f' return_code: {rc}')

    def publish(self, msg, topic):
        """
        Publish a message to the specified MQTT topic.

        Args:
            msg: The message to publish.
            topic: The MQTT topic to publish to.
        """
        msg_count = 1
        while True:
            result = self.mqtt_client.publish(topic, msg, qos=2)
            # result: [0, 1]
            status = result[0]
            if status == 0:
                LOGGER.debug(f'Send {msg} to topic {topic}')
            else:
                LOGGER.warning(f'Failed to send message to topic {topic}')
            msg_count += 1
            if msg_count > 1:
                break
            time.sleep(0.01)

    def on_message(self, client, userdata, msg):
        """
        Handle the MQTT message event.

        Args:
            client: The MQTT client instance.
            userdata: The user data associated with the message.
            msg: The received MQTT message.
        """
        del client, userdata  # delete unused params
        content = msg.payload.decode('utf-8')
        LOGGER.debug(f'Received {content} to topic {msg.topic}')
        self.msg = msg
        LOGGER.debug("Unlocking...")
        self.locked = False

    def start_client(self):
        """Start the MQTT client."""
        self.mqtt_client.loop_start()
        self.mqtt_client.subscribe('enumerate_devices_response')
        self.mqtt_client.subscribe('send_hw_reset_response')
        self.mqtt_client.subscribe('send_hwm_command_response')
        self.mqtt_client.subscribe('get_transformation_response')
        self.mqtt_client.subscribe('get_device_info_response')
        self.mqtt_client.subscribe('get_parameter_response')
        self.mqtt_client.subscribe('set_parameter_response')
        self.mqtt_client.subscribe('get_frame_response')
        self.mqtt_client.subscribe('get_safety_preset_response')
        self.mqtt_client.subscribe('set_safety_preset_response')
        self.mqtt_client.subscribe('get_safety_interface_config_response')
        self.mqtt_client.subscribe('set_safety_interface_config_response')
        self.mqtt_client.subscribe('get_calib_config_response')
        self.mqtt_client.subscribe('set_calib_config_response')
        self.mqtt_client.subscribe('get_application_config_response')
        self.mqtt_client.subscribe('set_application_config_response')
        self.mqtt_client.subscribe('triggered_calibration_response')

        self.mqtt_client.on_message = self.on_message

    def stop_client(self):
        """Stop the MQTT client."""
        self.mqtt_client.loop_stop()

    def get_message(self):
        print_once = True
        while self.locked:
            if print_once:
                LOGGER.debug("Waiting for message..")
                print_once = False
            pass
        msg = self.msg
        self.msg = None
        LOGGER.debug("message retrieved:{msg}")
        return msg


    def send_enumerate_devices_request(self, camera_namespace_prefix, camera_name_prefix):
        """
        Send a request to enumerate devices.

        Args:
            camera_namespace_prefix: The prefix of the camera namespace.
            camera_name_prefix: The prefix of the camera name.
        """
        request_dict = {
            'camera_namespace_prefix': camera_namespace_prefix,
            'camera_name_prefix': camera_name_prefix
        }
        j = json.dumps(request_dict)
        self.msg = None
        self.locked = True
        self.publish(j, 'enumerate_devices_request')

    def get_enumerate_devices_response(self):
        """
        Get response to the enumerate devices request.

        Args:
            None
        """
        msg = self.get_message()
        assert msg.topic == "enumerate_devices_response", "Unexpected topic: Enumerate device expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "Enumerate device failed:" + payload["error_msg"]
        return payload
    
    def enumerate_devices(self, camera_namespace_prefix, camera_name_prefix):
        """
        Send a request to enumerate devices.

        Args:
            camera_namespace_prefix: The prefix of the camera namespace.
            camera_name_prefix: The prefix of the camera name.
        """
        self.send_enumerate_devices_request(camera_namespace_prefix, camera_name_prefix)
        return self.get_enumerate_devices_response()

    def send_hw_reset_request(self, camera_namespace, camera_name):
        """
        Send a request to reset the device

        Args:
            camera_namespace
            camera_name
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'send_hw_reset_request')

    def receive_hw_reset_response(self):
        """
        Get response to the hw_reset_request.

        Args:
            None
        """
        msg = self.get_message()
        assert msg.topic == "send_hw_reset_response", "Unexpected topic: send_hw_reset_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "send_hw_reset_response failed:" + payload["error_msg"]
        return payload

    def send_hw_reset(self, camera_namespace, camera_name):
        """
        Send a request to reset the device and get the response

        Args:
            camera_namespace
            camera_name
        """
        self.send_hw_reset_request(camera_namespace, camera_name)
        return self.receive_hw_reset_response()



    def send_hwm_command_request(self, camera_namespace, camera_name, opcode, param1=None, param2=None, param3=None, param4=None, data=None):
        """
        Send a request to send the hwm command

        Args:
            opcode: hwm command
            param1, param2, param3, param4: parameters for hwm command
            data for hwm command
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'opcode': opcode,
        }
        if param1 is not None:request_dict['param1'] = param1
        if param2 is not None:request_dict['param2'] = param2
        if param3 is not None:request_dict['param3'] = param3
        if param4 is not None:request_dict['param4'] = param4
        if data is not None:request_dict['data'] = data
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'send_hwm_command_request')

    def receive_hwm_command_response(self):
        """
        Get response to the send_hwm_command_request.

        Args:
            None
        """
        msg = self.get_message()
        assert msg.topic == "send_hwm_command_response", "Unexpected topic: send_hwm_command_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "send_hwm_command_response failed:" + payload["error_msg"]
        return payload
    def send_hwm_command(self, camera_namespace, camera_name, opcode, param1=None, param2=None, param3=None, param4=None, data=None):
        """
        Send the Hwm command and get the response
        Args:
            opcode: hwm command
            param1, param2, param3, param4: parameters for hwm command
            data for hwm command
        """
        self.send_hwm_command_request(camera_namespace, camera_name, opcode, param1, param2, param3, param4, data)
        return self.receive_hwm_command_response()


    def send_get_transformation_request(self, camera_namespace, camera_name, source, destination):
        """
        Send a request to find the ROS2 transformation from source frame to destination frame

        Args:
            source: source frame id.
            destination: destination frame id
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'source': source,
            'destination': destination
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'get_transformation_request')

    def receive_get_transformation_response(self):
        """
        Get response to the get_transformation_request.

        Args:
            None
        """
        msg = self.get_message()
        assert msg.topic == "get_transformation_response", "Unexpected topic: get_transformation_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "get_transformation failed:" + payload["error_msg"]
        return payload

    def get_transformation(self, camera_namespace, camera_name, source, destination):
        """
        Send a request to find the ROS2 transformation from source frame to destination frame

        Args:
            source: source frame id.
            destination: destination frame id
        """
        self.send_get_transformation_request(camera_namespace, camera_name,source, destination)
        return self.receive_get_transformation_response()

    def send_set_param_request(self, camera_namespace, camera_name,
                  parameter_name, parameter_value, parameter_type):
        """
        Send a request to set a parameter.

        Args:
            camera_namespace: The namespace of the camera.
            camera_name: The name of the camera.
            parameter_name: The name of the parameter to set.
            parameter_value: The value to set for the parameter.
            parameter_type: The type of the parameter.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'parameter_name': parameter_name,
            'parameter_value': parameter_value,
            'parameter_type': parameter_type
        }
        j = json.dumps(request_dict)
        self.msg = None
        self.locked = True
        self.publish(j, 'set_param_request')

    def receive_set_param_response(self):
        """
        Get response to the set param request.

        Args:
            None
        """
        msg = self.get_message()
        assert msg.topic == "set_parameter_response", "Unexpected topic: set_param expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "set_param failed:" + payload["error_msg"]
        return payload

    def set_param(self, camera_namespace, camera_name,
                  parameter_name, parameter_value, parameter_type):
        """
        Send a request to set a parameter.

        Args:
            camera_namespace: The namespace of the camera.
            camera_name: The name of the camera.
            parameter_name: The name of the parameter to set.
            parameter_value: The value to set for the parameter.
            parameter_type: The type of the parameter.
        """
        self.send_set_param_request(camera_namespace, camera_name,
                  parameter_name, parameter_value, parameter_type)
        return self.receive_set_param_response()

    def set_string_param(self, camera_namespace, camera_name,
                  parameter_name, parameter_value):
        return self.set_param(camera_namespace, camera_name,
                  parameter_name, str(parameter_value), parameter_type="string")


    def set_integer_param(self, camera_namespace, camera_name,
                  parameter_name, parameter_value):
        return self.set_param(camera_namespace, camera_name,
                  parameter_name, str(parameter_value), parameter_type="int")

    def set_bool_param(self, camera_namespace, camera_name,
                  parameter_name, parameter_value):
        return self.set_param(camera_namespace, camera_name,
                  parameter_name, str(parameter_value), parameter_type="bool")
    
    def send_get_param_request(self, camera_namespace, camera_name, parameter_name):
        """
        Send a request to get a parameter.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            parameter_name (str): The name of the parameter to get.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'parameter_name': parameter_name,
        }
        j = json.dumps(request_dict)
        self.msg = None
        self.locked = True
        self.publish(j, 'get_param_request')

    def receive_get_param_response(self):
        """
        Get response to the set param request.
        info
        Args:
            None
        """
        msg = self.get_message()
        assert msg.topic == "get_parameter_response", "Unexpected topic: get_param expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "get_param failed:" + payload["error_msg"]
        return payload

    def get_param_msg(self, camera_namespace, camera_name, parameter_name):
        """
        Send a request to get a parameter.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            parameter_name (str): The name of the parameter to get.
        """
        self.send_get_param_request(camera_namespace, camera_name, parameter_name)
        return self.receive_get_param_response()

    def get_param(self, camera_namespace, camera_name, parameter_name):
        """
        Send a request to get a parameter.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            parameter_name (str): The name of the parameter to get.
        """
        msg = self.get_param_msg(camera_namespace, camera_name, parameter_name)
        return msg['parameter_value']

    def get_bool_param(self, camera_namespace, camera_name,
                  parameter_name):
        param = self.get_param(camera_namespace, camera_name, parameter_name)
        if param.lower() == 'true':
            return True
        if param.lower() == 'false':
            return True
        assert False, "Invalid boolean parameter:" + param

    def get_integer_param(self, camera_namespace, camera_name,
                  parameter_name):
        param = self.get_param(camera_namespace, camera_name,
                  parameter_name)
        return int(param)

    def get_string_param(self, camera_namespace, camera_name,
                  parameter_name):
        param = self.get_param(camera_namespace, camera_name,
                  parameter_name)
        return str(param)


    def send_get_frame_request(self, camera_namespace, camera_name, stream_name):
        """
        Send a request to get a frame.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            stream_name (str): The name of the stream.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'stream_name': stream_name,
        }
        j = json.dumps(request_dict)
        self.msg = None
        self.locked = True
        self.publish(j, 'get_frame_request')

    def receive_get_frame_response(self):
        msg = self.get_message()
        LOGGER.debug("received frame")
        assert msg.topic == "get_frame_response", "Unexpected topic: get_frame_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        LOGGER.info(payload)
        assert payload["success"] == True, "get_frame_response failed. message:" + payload["error_msg"]
        return payload


    def get_frame_msg(self, camera_namespace, camera_name, stream_name):
        """
        Send a request to get a frame.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            stream_name (str): The name of the stream.
        """
        self.send_get_frame_request(camera_namespace, camera_name, stream_name)
        return  self.receive_get_frame_response()
    

    def send_set_safety_preset_request(self, camera_namespace, camera_name, sp, index):
        """
        Send a request to set a send_set_safety_preset_request.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            sp (str): The safety preset.
            index (int): The index of the safety preset.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'safety_preset': sp,
            'index':index,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'set_safety_preset_request')

    def receive_set_safety_preset_response(self):
        """
        receive response to get to set a set_safety_preset.

        Args:
        """
        msg = self.get_message()
        assert msg.topic == "set_safety_preset_response", "Unexpected topic: set_safety_preset_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "set_safety_preset_response failed:" + payload["error_msg"]
        return payload


    def send_get_safety_preset_request(self, camera_namespace, camera_name, index):
        """
        Send a request to get a safety preset.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            index (int): The index of the safety preset.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'index': index,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'get_safety_preset_request')

    def receive_get_safety_preset_response(self):
        """
        receive response to get to get a safety preset.

        Args:
        """
        msg = self.get_message()
        assert msg.topic == "get_safety_preset_response", "Unexpected topic: get_safety_preset_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "get_safety_preset_response failed:" + payload["error_msg"]
        return payload

    def get_safety_preset(self, camera_namespace, camera_name, index):
        """
        Send a request to get a safety preset.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            index (int): The index of the safety preset.
        """
        self.send_get_safety_preset_request(camera_namespace, camera_name, index)
        return self.receive_get_safety_preset_response()


 


    def send_set_safety_interface_config_request(self, camera_namespace, camera_name, sp):
        """
        Send a request to set a safety interface config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            sp (str): The safety preset.
            index (int): The index of the safety preset.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'safety_interface_config': sp,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'set_safety_interface_config_request')

    def receive_set_safety_interface_config_response(self):
        """
        receive response to get to set a safety interface config.

        Args:
        """
        msg = self.get_message()
        assert msg.topic == "set_safety_interface_config_response", "Unexpected topic: set_safety_interface_config_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "set_safety_interface_config_response failed:" + payload["error_msg"]
        return payload

    def set_safety_interface_config(self, camera_namespace, camera_name, sp):
        """
        Send a request to set a safety interface config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            sp (str): The safety config.
        """
        self.send_set_safety_interface_config_request(camera_namespace, camera_name, sp)
        return self.receive_set_safety_interface_config_response()

 
    def send_get_safety_interface_config_request(self, camera_namespace, camera_name, index=2):
        """
        Send a request to set a safety interface config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            sp (str): The safety config.
            index (int): The index of the safety config (EEPROM/RAM/FLASH).
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'calib_location': index,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'get_safety_interface_config_request')

    def receive_get_safety_interface_config_response(self):
        """
       receive response to get a safety interface config.

        Args:
        """
        msg = self.get_message()
        assert msg.topic == "get_safety_interface_config_response", "Unexpected topic: get_safety_interface_config_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "get_safety_interface_config_response failed:" + payload["error_msg"]
        return payload



    def get_safety_interface_config(self, camera_namespace, camera_name, index):
        """
        Send a request to set a safety interface config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            sp (str): The safety preset.
            index (int): The index of the safety config.
        """
        self.send_get_safety_interface_config_request(camera_namespace, camera_name, index)
        return self.receive_get_safety_interface_config_response()


    def send_get_calib_config_request(self, camera_namespace, camera_name):
        """
        Send a request to get a calib config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'get_calib_config_request')

    def receive_get_calib_config_response(self):
        """
        receive the response to a request to get a calib config.

        Args:
        """
        msg = self.get_message()
        assert msg.topic == "get_calib_config_response", "Unexpected topic: get_calib_config_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "get_calib_config_response failed:" + payload["error_msg"]
        return payload

    def get_calib_config(self, camera_namespace, camera_name):
        """
        Send a request to get a calib config and get response.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        self.send_get_calib_config_request(camera_namespace, camera_name)
        return self.receive_get_calib_config_response()


    def send_set_calib_config_request(self, camera_namespace, camera_name, calib_config):
        """
        Send a request to set a calib config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            calib_config (str): The calib config.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'calib_config': calib_config,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'set_calib_config_request')

    def receive_set_calib_config_response(self):
        """
        Get response to a request to set a calib config.

        Args:
        """
        msg = self.get_message()
        assert msg.topic == "set_calib_config_response", "Unexpected topic: set_calib_config_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "set_calib_config_response failed:" + payload["error_msg"]
        return payload

    def set_calib_config(self, camera_namespace, camera_name, calib_config):
        """
        Send a request to set a calib config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            calib_config (str): The calib config.
        """
        self.send_set_calib_config_request(camera_namespace, camera_name, calib_config)
        return self.receive_set_calib_config_response()
 

    def send_get_application_config_request(self, camera_namespace, camera_name):
        """
        Send a request to get an application config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'get_application_config_request')

    def receive_get_application_config_response(self):
        """
        Send a request to get an application config.

        Args:
        """
        msg = self.get_message()
        assert msg.topic == "get_application_config_response", "Unexpected topic: get_application_config_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "get_application_config_response failed:" + payload["error_msg"]
        return payload

    def get_application_config(self, camera_namespace, camera_name):
        """
        Send a request to get an application config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        self.send_get_application_config_request(camera_namespace, camera_name)
        return self.receive_get_application_config_response()



    def send_set_application_config_request(self, camera_namespace, camera_name, application_config):
        """
        Send a request to set an application config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            application_config (str): The application config.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'application_config': application_config,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'set_application_config_request')


    def receive_set_application_config_response(self):
        """
        Send a request to set an application config.

        Args:
        """
        msg = self.get_message()
        assert msg.topic == "set_application_config_response", "Unexpected topic: set_application_config_response expected, received " + msg.topic
        payload = json.loads(msg.payload)
        assert payload["success"] == True, "set_application_config_response failed:" + payload["error_msg"]
        return payload

    def set_application_config(self, camera_namespace, camera_name, application_config):
        """
        Send a request to set an application config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            application_config (str): The application config.
        """
        self.send_set_application_config_request(camera_namespace, camera_name, application_config)
        return self.receive_set_application_config_response()


    def send_get_device_info_request(self, camera_namespace, camera_name):
        """
        Send a request to get the device info.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'get_device_info_request')

    def receive_get_device_info_response(self):
        """
        Send a request to get the device info.

        Args:
        """
        msg = self.get_message()
        assert msg.topic == "get_device_info_response", "Unexpected topic: get_device_info_response expected, received " + msg.topic
        #payload = json.loads(msg.payload)
        #LOGGER.info(msg.topic)
        payload = json.loads(msg.payload)
        '''
        LOGGER.warning("payload[success] is not a string in get_device_info_response")
        #assert payload["success"] == True, "get_device_info_response failed:" + payload["error_msg"]
        #assert payload["success"] == "true", "get_device_info_response failed:" + payload["error_msg"]
        '''
        return payload

    def get_device_info(self, camera_namespace, camera_name):
        """
        Send a request to get an application config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        self.send_get_device_info_request(camera_namespace, camera_name)
        return self.receive_get_device_info_response()




    def send_triggered_calibration_request(self, camera_namespace, camera_name, dryrun=False):
        """
        Send request to run triggered calibration action.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'json':'calib run',
        }
        if dryrun == True:
            request_dict['json'] = 'calib dry run'
        LOGGER.debug(f"triggered calib request:{request_dict}")
        j = json.dumps(request_dict)
        self.msg = None
        self.locked = True
        self.publish(j, 'triggered_calibration_request')
    
    def receive_triggered_calibration_response(self):
        """
        Receive rseponse to run triggered calibration action.

        Args:
        """
        msg  = self.get_message()
        assert msg is not None, "Invalid message received"
        LOGGER.info(f'response received: {msg.topic}')
        assert msg.topic == "triggered_calibration_response", "Unexpected topic: triggered_calibration_response expected, received " + msg.topic
        #multple responses expected
        payload = json.loads(msg.payload)
        LOGGER.debug(msg.payload)
        self.locked = True
        return payload
    
    def abort_triggered_calibration_request(self, camera_namespace, camera_name, dryrun=False):
        """
        Send request to run triggered calibration action.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'json':'calib abort',
        }
        LOGGER.debug(f"triggered calib request:{request_dict}")
        j = json.dumps(request_dict)
        self.msg = None
        self.locked = True
        self.publish(j, 'triggered_calibration_request')

    def set_safety_mode(self, camera_namespace, camera_name, mode):
        self.set_integer_param(camera_namespace, camera_name, 'safety_camera.safety_mode',mode)
        time.sleep(0.5)
        LOGGER.info("Param safety_camera.safety_mode: %s", str(self.get_param_msg(camera_namespace, camera_name, 'safety_camera.safety_mode')))

    def prepare_for_calibration(self, camera_namespace, camera_name):
        self.set_safety_mode(camera_namespace, camera_name, 2)
        # switch to visual preset #1
        self.set_integer_param(camera_namespace, camera_name, 'depth_module.visual_preset',1)
        time.sleep(0.5)
        LOGGER.info("Param depth_module.visual_preset: %s", self.get_param_msg(camera_namespace, camera_name, 'depth_module.visual_preset'))

        # enable emitter
        self.set_bool_param(camera_namespace, camera_name, 'depth_module.emitter_enabled',True)
        time.sleep(0.5)
        LOGGER.info("Param depth_module.emitter_enabled: %s", self.get_param_msg(camera_namespace, camera_name, 'depth_module.emitter_enabled'))

        # enable auto exposuretc_done   CAMERA_NAME,
        self.set_bool_param(camera_namespace, camera_name,  'depth_module.enable_auto_exposure',True)
        time.sleep(0.5)
        LOGGER.info("Param depth_module.enable_auto_exposure: %s", self.get_param_msg(camera_namespace, camera_name, 'depth_module.enable_auto_exposure'))

        # turn off depth streaming
        self.set_bool_param(camera_namespace, camera_name, 'enable_depth',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_depth: %s", self.get_param_msg(camera_namespace, camera_name, 'enable_depth'))
        
        # turn off infra1 streaming
        self.set_bool_param(camera_namespace, camera_name, 'enable_infra1',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_infra1: %s", self.get_param_msg(camera_namespace, camera_name, 'enable_infra1'))
        # turn off infra2 streaming
        self.set_bool_param(camera_namespace, camera_name, 'enable_infra2',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_infra2: %s", self.get_param_msg(camera_namespace, camera_name, 'enable_infra2'))
        # turn off safety streaming
        self.set_bool_param(camera_namespace, camera_name, 'enable_safety',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_safety: %s", self.get_param_msg(camera_namespace, camera_name, 'enable_safety'))
        # turn off occupancy streaming
        self.set_bool_param(camera_namespace, camera_name, 'enable_occupancy',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_occupancy: %s", self.get_param_msg(camera_namespace, camera_name, 'enable_occupancy'))

        self.set_bool_param(camera_namespace, camera_name, 'enable_color',False)
        time.sleep(0.5)
        LOGGER.info("Param enable_color: %s", self.get_param_msg(camera_namespace, camera_name, 'enable_color'))

        # turn off lpcl streaming
        self.set_bool_param(camera_namespace, camera_name, 'enable_labeled_point_cloud',False)
        time.sleep(1.5)
        LOGGER.info("Param enable_labeled_point_cloud: %s", self.get_param_msg(camera_namespace, camera_name, 'enable_labeled_point_cloud'))

    def start_stop_stream(self, camera_namespace, camera_name, stream_name, start_stream=True):
        self.set_integer_param(camera_namespace, camera_name, 'safety_camera.safety_mode',0)
        time.sleep(0.5)
        self.set_bool_param(camera_namespace, camera_name, stream_name,'true' if start_stream else 'false')
        time.sleep(0.5)
        is_stream_enabled = self.get_param(camera_namespace, camera_name, stream_name) 
        assert is_stream_enabled != start_stream, f"Param {stream_name} is not set: expected: {start_stream} received: {is_stream_enabled}"

    def start_stop_color_stream(self, camera_namespace, camera_name, start_stream=True):
        self.start_stop_stream(camera_namespace, camera_name, 'enable_color', start_stream)

    def start_stop_depth_stream(self, camera_namespace, camera_name, start_stream=True):
        self.start_stop_stream(camera_namespace, camera_name, 'enable_depth', start_stream)

    def start_stop_infra1_stream(self, camera_namespace, camera_name, start_stream=True):
        self.start_stop_stream(camera_namespace, camera_name, 'enable_infra1', start_stream)
        
    def start_stop_infra2_stream(self, camera_namespace, camera_name, start_stream=True):
        self.start_stop_stream(camera_namespace, camera_name, 'enable_infra2', start_stream)

    def start_stop_safety_stream(self, camera_namespace, camera_name, start_stream=True):
        self.start_stop_stream(camera_namespace, camera_name, 'enable_safety', start_stream)
