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
"""Main module for interacting with the MQTT client."""
import json
import random
import time
import numpy as np

from paho.mqtt import client as paho_mqtt_client


class DemoMQTTClient:
    """
    Class that acts as Demo MQTT Client.

    This class provides methods for initializing the MQTT client, handling
    MQTT connections, publishing messages, and interacting with MQTT topics.

    Attributes:
        mqtt_broker_ip: The IP address of the MQTT broker.
        mqtt_broker_port: The port number of the MQTT broker.
        mqtt_client: The MQTT client instance.
    """

    def __init__(self, mqtt_broker_ip, mqtt_broker_port):
        """
        Initialize the Demo MQTT client.

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
                  f' ip:{ self.mqtt_broker_ip} port:{self.mqtt_broker_port}')
        else:
            print(f'Could not Connect to MQTT Broker on'
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
            if msg.topic == 'triggered_calibration_response':
                tc_json = json.loads(content)
                if tc_json['progress'] == 100.0:
                    self.tc_done = True
            print(f'Received {content} to topic {msg.topic}')
        self.locked = False;

    def start_client(self):
        """Start the MQTT client."""
        self.mqtt_client.loop_start()
        self.mqtt_client.subscribe('enumerate_devices_response')
        self.mqtt_client.subscribe('get_transformation_response')
        self.mqtt_client.subscribe('send_hw_reset_response')
        self.mqtt_client.subscribe('send_hwm_command_response')
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

    def get_transformation(self, source, destination):
        """
        Send a request to find the ROS2 transformation from source frame to destination frame

        Args:
            source: source frame id.
            destination: destination frame id
        """
        request_dict = {
            'source': source,
            'destination': destination
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'get_transformation_request')
        while self.locked:
            pass

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
        while self.locked:
            pass

    def send_hwm_command(self, camera_namespace, camera_name, opcode,
                         param1 = 0, param2 = 0, param3 = 0, param4 = 0,
                         data = []):
        """
        Send a hwm command request
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'opcode' : opcode,
            'param1' : param1, 
            'param2' : param2,
            'param3' : param3,
            'param4' : param4,
            'data': np.array(data).tolist()
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'send_hwm_command_request')
        while self.locked:
            pass

    def get_device_info(self, camera_namespace, camera_name):
        """
        Send a request to get device info.

        Args:
            camera_namespace: The namespace of the camera.
            camera_name: The name of the camera.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'get_device_info_request')
        while self.locked:
            pass

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
        self.locked = True
        self.publish(j, 'set_param_request')
        while self.locked:
            pass

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
        self.locked = True
        self.publish(j, 'get_param_request')
        while self.locked:
            pass

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
        self.locked = True
        self.publish(j, 'get_frame_request')
        while self.locked:
            pass

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
        self.locked = True
        self.publish(j, 'get_safety_preset_request')
        while self.locked:
            pass

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
            'safety_preset': sp,
            'index': str(index),
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'set_safety_preset_request')
        while self.locked:
            pass

    def get_safety_interface_config(self, camera_namespace, camera_name, index=2):
        """
        Send a request to get a safety inteface config.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'calib_location': index
        }
        j = json.dumps(request_dict)
        self.locked = True
        self.publish(j, 'get_safety_interface_config_request')
        while self.locked:
            pass

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
        self.locked = True
        self.publish(j, 'set_safety_interface_config_request')
        while self.locked:
            pass

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
        self.locked = True
        self.publish(j, 'get_calib_config_request')
        while self.locked:
            pass

    def set_calib_config(self, camera_namespace, camera_name, calib_config):
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
        while self.locked:
            pass

    def get_application_config(self, camera_namespace, camera_name):
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
        while self.locked:
            pass

    def set_application_config(self, camera_namespace, camera_name, application_config):
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
        while self.locked:
            pass


    def triggered_calibration(self, camera_namespace, camera_name):
        """
        Run triggered calibration action.

        Args:
            camera_namespace (str): The namespace of the camera.
            camera_name (str): The name of the camera.
        """
        self.tc_done = False
        request_dict = {
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'json': 'calib run'
        }
        j = json.dumps(request_dict)
        self.publish(j, 'triggered_calibration_request')


if __name__ == '__main__':
    MQTT_BROKER_IP = 'localhost'
    MQTT_BROKER_PORT = 1883

    jsons_dir = '../../realsense2_camera/examples/d500_tables/'

    demo_mqtt_client = DemoMQTTClient(MQTT_BROKER_IP, MQTT_BROKER_PORT)
    demo_mqtt_client.start_client()

    CAMERA_NAMESPACE_PREFIX = 'robot'
    CAMERA_NAME_PREFIX = 'c_'

    # enumerate devices
    demo_mqtt_client.enumerate_devices(CAMERA_NAMESPACE_PREFIX,
                                       CAMERA_NAME_PREFIX)

    # choose specific camera
    CAMERA_NAMESPACE = 'robot1'
    CAMERA_NAME = 'c_353322320702'

    # reset the device
    demo_mqtt_client.send_hw_reset_request(CAMERA_NAMESPACE,
                                     CAMERA_NAME)
    #needs time to reset
    import time
    time.sleep(8)

    # get device info
    demo_mqtt_client.get_device_info(CAMERA_NAMESPACE,
                                     CAMERA_NAME)
    
    # get ROS2 transformation from 'c_353322320702_link' to 'c_353322320702_color_frame'
    demo_mqtt_client.get_transformation('c_353322320702_link', 'c_353322320702_color_frame')


    # send raw HWM command like GVD
    demo_mqtt_client.send_hwm_command(camera_namespace= CAMERA_NAMESPACE,
                                      camera_name = CAMERA_NAME,
                                      opcode = 0x10)

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

    safety_preset_file = open(jsons_dir + 'safety_preset_example.json',
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

    safety_interface_config_file = open(jsons_dir + 'safety_interface_config_example.json',
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

    calib_config_file = open(jsons_dir + 'calib_config_example.json',
                             mode='r',
                             encoding='utf-8')
    calib_config_json = json.load(calib_config_file)
    CALIB_CONFIG_ESCAPED = str(calib_config_json).replace('"', '\"')
    CALIB_CONFIG_ESCAPED = str(calib_config_json).replace("'", '\"')

    demo_mqtt_client.set_calib_config(CAMERA_NAMESPACE,
                                      CAMERA_NAME,
                                      CALIB_CONFIG_ESCAPED)


    ##################################################################
    ########## APPLICATION CONFIG GET/SET EXAMPLE ####################

    demo_mqtt_client.get_application_config(CAMERA_NAMESPACE,
                                            CAMERA_NAME)

    application_config_file = open(jsons_dir + 'application_config_example.json',
                                   mode='r',
                                   encoding='utf-8')
    application_config_json = json.load(application_config_file)
    APPLICATION_CONFIG_ESCAPED = str(application_config_json).replace('"', '\"')
    APPLICATION_CONFIG_ESCAPED = str(application_config_json).replace("'", '\"')

    demo_mqtt_client.set_application_config(CAMERA_NAMESPACE,
                                            CAMERA_NAME,
                                            APPLICATION_CONFIG_ESCAPED)


    ###################################################################
    ############## TRIGGERED CALIBRATION EXAMPLE ######################

    # setup params for triggered calibration

    # we are already in service mode, no need to switch

    # switch to visual preset #1
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'depth_module.visual_preset',
                               '1',
                               'int')

    # enable emitter
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'depth_module.emitter_enabled',
                               'true',
                               'bool')

    # enable auto exposure
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'depth_module.enable_auto_exposure',
                               'true',
                               'bool')

    # turn off depth streaming
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'enable_depth',
                               'false',
                               'bool')

    # turn off infra1 streaming
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'enable_infra1',
                               'false',
                               'bool')

    # turn off infra2 streaming
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'enable_infra2',
                               'false',
                               'bool')

    # turn off safety streaming
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'enable_safety',
                               'false',
                               'bool')

    # turn off occupancy streaming
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'enable_occupancy',
                               'false',
                               'bool')

    # turn off lpcl streaming
    demo_mqtt_client.set_param(CAMERA_NAMESPACE,
                               CAMERA_NAME,
                               'enable_labeled_point_cloud',
                               'false',
                               'bool')

    # call the triggered calibration method
    demo_mqtt_client.triggered_calibration(CAMERA_NAMESPACE,
                                           CAMERA_NAME)

    # check if TC is done, otherwise sleep for 2 seconds
    while not demo_mqtt_client.tc_done:
          time.sleep(2)

    ###################################################################
    demo_mqtt_client.stop_client()
    print(f'MQTT example completed, stopping the client and exiting')
