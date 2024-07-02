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
"""Main module for interacting with the MQTT client."""
import json
import random
import time

from paho.mqtt import client as paho_mqtt_client


class DemoMQTTClient:
    """
    Class that acts as Demo MQTT Client.

    This class provides methods for initializing the MQTT client, handling
    MQTT connections, publishing messages, and interacting with MQTT topics.

    Attributes:
        mqtt_broker_ip: The IP address of the MQTT broker.
        mqtt_port: The port number of the MQTT broker.
        mqtt_client: The MQTT client instance.
    """

    def __init__(self, mqtt_broker_ip, mqtt_port):
        """
        Initialize the Demo MQTT client.

        Args:
            mqtt_broker_ip: The IP address of the MQTT broker.
            mqtt_port: The port number of the MQTT broker.
        """
        self.mqtt_broker_ip = mqtt_broker_ip
        self.mqtt_port = mqtt_port

        # Generate a Client ID with a random user-id suffix.
        mqtt_client_id = f'mqtt-client-user-{random.randint(0, 1000)}'

        self.mqtt_client = paho_mqtt_client.Client(
            paho_mqtt_client.CallbackAPIVersion.VERSION1, mqtt_client_id)
        # client.username_pw_set(username, password)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.connect(mqtt_broker_ip, mqtt_port)

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
            print(f'Connected to MQTT Broker on'
                  f' ip:{ self.mqtt_broker_ip} port:{self.mqtt_port}')
        else:
            print(f'Could not Connect to MQTT Broker on'
                  f' ip:{ self.mqtt_broker_ip} port:{self.mqtt_port}'
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
            time.sleep(1)
            result = self.mqtt_client.publish(topic, msg, qos=2)
            # result: [0, 1]
            status = result[0]
            if status == 0:
                print(f'Send {msg} to topic {topic}')
            else:
                print(f'Failed to send message to topic {topic}')
            msg_count += 1
            if msg_count > 1:
                break

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
        if msg.topic == 'get_frame_response':
            print('got image... \
                not printing it since its raw data is huge...')
        else:
            print(f'Received {content} to topic {msg.topic}')

    def start_client(self):
        """Start the MQTT client."""
        self.mqtt_client.loop_start()
        self.mqtt_client.subscribe('enumerate_devices_response')
        self.mqtt_client.subscribe('get_parameter_response')
        self.mqtt_client.subscribe('set_parameter_response')
        self.mqtt_client.subscribe('get_frame_response')
        self.mqtt_client.subscribe('get_safety_preset_response')
        self.mqtt_client.subscribe('set_safety_preset_response')
        self.mqtt_client.subscribe('get_safety_interface_config_response')
        self.mqtt_client.subscribe('set_safety_interface_config_response')
        self.mqtt_client.subscribe('get_calib_config_response')
        self.mqtt_client.subscribe('set_calib_config_response')

        self.mqtt_client.on_message = self.on_message

    def stop_client(self):
        """Stop the MQTT client."""
        self.mqtt_client.loop_stop()

    def enumerate_devices(self, camera_namespace_prefix, camera_name_prefix):
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
        self.publish(j, 'enumerate_devices_request')

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
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'parameter_name': parameter_name,
            'parameter_value': parameter_value,
            'parameter_type': parameter_type
        }
        j = json.dumps(request_dict)
        self.publish(j, 'set_param_request')

    def get_param(self, camera_namespace, camera_name, parameter_name):
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
        self.publish(j, 'get_param_request')

    def get_frame(self, camera_namespace, camera_name, stream_name):
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
        self.publish(j, 'get_frame_request')

    def get_safety_preset(self, camera_namespace, camera_name, index):
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
        self.publish(j, 'get_safety_preset_request')

    def set_safety_preset(self, camera_namespace, camera_name, sp, index):
        """
        Send a request to set a safety preset.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            sp (str): The safety preset.
            index (int): The index of the safety preset.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'preset': sp,
            'index': str(index),
        }
        j = json.dumps(request_dict)
        self.publish(j, 'set_safety_preset_request')

    def get_safety_interface_config(self, camera_namespace, camera_name):
        """
        Send a request to get a safety inteface config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
        }
        j = json.dumps(request_dict)
        self.publish(j, 'get_safety_interface_config_request')

    def set_safety_interface_config(self, camera_namespace, camera_name, sic):
        """
        Send a request to set a safety interface config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
            sic (str): The safety interface config.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'safety_interface_config': sic,
        }
        j = json.dumps(request_dict)
        self.publish(j, 'set_safety_interface_config_request')

    def get_calib_config(self, camera_namespace, camera_name):
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
        self.publish(j, 'get_calib_config_request')

    def set_calib_config(self, camera_namespace, camera_name, sic):
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
            'calib_config': sic,
        }
        j = json.dumps(request_dict)
        self.publish(j, 'set_calib_config_request')


if __name__ == '__main__':
    MQTT_BROKER_IP = 'localhost'
    MQTT_PORT = 1883

    demo_mqtt_client = DemoMQTTClient(MQTT_BROKER_IP, MQTT_PORT)
    demo_mqtt_client.start_client()

    CAMERA_NAMESPACE_PREFIX = 'robot'
    CAMERA_NAME_PREFIX = 'c_'

    demo_mqtt_client.enumerate_devices(CAMERA_NAMESPACE_PREFIX,
                                       CAMERA_NAME_PREFIX)

    CAMERA_NAMESPACE = 'robot1'
    CAMERA_NAME = 'c_353322320702'

    # switch to service mode
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'safety_camera.safety_mode',
                               '2',
                               'int')

    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'rgb_camera.exposure',
                               '6012',
                               'int')

    demo_mqtt_client.get_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'rgb_camera.exposure')

    demo_mqtt_client.get_frame(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'color')

    ###################################################################
    ################ SAFETY PRESET GET/SET EXAMPLE ####################

    demo_mqtt_client.get_safety_preset(CAMERA_NAMESPACE,
                                       CAMERA_NAME,
                                       1)

    safety_preset_file = open('../../examples/config/safety_preset_example.json',
                              mode='r',
                              encoding='utf-8')
    safety_preset_json = json.load(safety_preset_file)
    SP_ESCAPED = str(safety_preset_json).replace('"', '\"')
    SP_ESCAPED = str(safety_preset_json).replace("'", '\"')

    demo_mqtt_client.set_safety_preset(CAMERA_NAMESPACE,
                                       CAMERA_NAME,
                                       SP_ESCAPED,
                                       61)


    ###################################################################
    ########### SAFETY INTERFACE CONFIG GET/SET EXAMPLE ###############

    demo_mqtt_client.get_safety_interface_config(CAMERA_NAMESPACE, 
                                                 CAMERA_NAME)

    safety_interface_config_file = open('../../examples/config/safety_interface_config_example.json',
                                        mode='r',
                                        encoding='utf-8')

    safety_interface_config_json = json.load(safety_interface_config_file)
    SIC_ESCAPED = str(safety_interface_config_json).replace('"', '\"')
    SIC_ESCAPED = str(safety_interface_config_json).replace("'", '\"')

    demo_mqtt_client.set_safety_interface_config(CAMERA_NAMESPACE,
                                                 CAMERA_NAME,
                                                 SIC_ESCAPED)


    ###################################################################
    ############## CALIB CONFIG GET/SET EXAMPLE #######################

    demo_mqtt_client.get_calib_config(CAMERA_NAMESPACE,
                                      CAMERA_NAME)

    calib_config_file = open('../../examples/config/calib_config_example.json',
                             mode='r',
                             encoding='utf-8')
    calib_config_json = json.load(calib_config_file)
    CALIB_CONFIG_ESCAPED = str(calib_config_json).replace('"', '\"')
    CALIB_CONFIG_ESCAPED = str(calib_config_json).replace("'", '\"')

    demo_mqtt_client.set_calib_config(CAMERA_NAMESPACE,
                                      CAMERA_NAME,
                                      CALIB_CONFIG_ESCAPED)
    


    time.sleep(3)
    demo_mqtt_client.stop_client()
