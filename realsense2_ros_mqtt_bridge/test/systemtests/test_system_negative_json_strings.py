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

import sys
import os
import json
import numpy as np

import rclpy 

import pytest 
sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))

from camera_n_mqtt_nodes import CameraNMqttNodes as RSCameraSimulator
from camera_n_mqtt_nodes import launch_descr_with_parameters
import camera_n_mqtt_nodes
from mqtt_client_simulator import MQTTClientSimulator
import logging

#LOGGER = logging.getLogger(__name__)
LOGGER = logging.getLogger()

test_params_d585s = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    }

#@pytest.mark.timeout(20)
@pytest.mark.parametrize("launch_descr_with_parameters",[
    pytest.param(test_params_d585s, marks=pytest.mark.d585s),
    ]
    ,indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
def test_system_negative_json_strings(launch_descr_with_parameters):
    #initialization starts....
    rclpy.init()
    params = launch_descr_with_parameters[1]
    namespace = 'camera'
    name = params['camera_name']
    camera = RSCameraSimulator(namespace, name)
    sds = MQTTClientSimulator("localhost", 1883)
    sds.start_client()
    camera.start()
    if LOGGER.getEffectiveLevel() <= logging.DEBUG:
        os.system("ros2 node list")
    #initialization ends....
    LOGGER.info("Testing enumerate_devices")

    '''
    request_dict = {
        'camera_name_prefix': name
    }
    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'enumerate_devices_request')
    msg = sds.get_message()
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name_prefix in mqtt_request"


    request_dict = {
        'camera_namespace_prefix': namespace
    }
    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'enumerate_devices_request')
    msg = sds.get_message()
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace_prefix in mqtt_request"

    '''
    request_dict = {
        'camera_namespace_prefix': namespace,
        'camera_name_prefix': name
    }
    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'enumerate_devices_request')
    msg = sds.get_message()
    response = json.loads(msg.payload)
    assert response["success"] == True, "Expected a success message for a proper mqtt_request"


    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'parameter_name': 'enable_safety',
    }
    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_param_request')
    msg = sds.get_message()
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'parameter_name': 'enable_safety',
    }
    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_param_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"


    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        #'parameter_name': 'enable_safety',
    }
    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_param_request')
    msg = sds.get_message()
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of parameter_name in mqtt_request"


    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'parameter_name': 'enable_safety',
        'parameter_value': False,
        'parameter_type': 'bool'
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_param_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"

    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'parameter_name': 'enable_safety',
        'parameter_value': False,
        'parameter_type': 'bool'
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_param_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"


    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        #'parameter_name': 'enable_safety',
        'parameter_value': False,
        'parameter_type': 'bool'
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_param_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of parameter_name in mqtt_request"


    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        'parameter_name': 'enable_safety',
        #'parameter_value': False,
        'parameter_type': 'bool'
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_param_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of parameter_value in mqtt_request"



    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        'parameter_name': 'enable_safety',
        'parameter_value': False,
        #'parameter_type': 'bool'
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_param_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of parameter_type in mqtt_request"

    #get_frame request
    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'stream_name': 'color',
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_frame_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'stream_name': 'color',
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_frame_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"
    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        #'stream_name': 'color',
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_frame_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of stream_name in mqtt_request"

    #get_safety_preset
    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'index': 0,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_safety_preset_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'index': 0,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_safety_preset_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"


    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        #'index': 0,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_safety_preset_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of index in mqtt_request"

    #set_safety_preset
    preset_response = sds.get_safety_preset(namespace, name, 1)

    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'index': 1,
        'safety_preset': preset_response['safety_preset']
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_safety_preset_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"


    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'index': 1,
        'safety_preset': preset_response['safety_preset']
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_safety_preset_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        #'index': 1,
        'safety_preset': preset_response['safety_preset']
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_safety_preset_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of index in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        'index': 1,
        #'safety_preset': preset_response['safety_preset']
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_safety_preset_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of preset in mqtt_request"

    #get_safety_interface_config
    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'calib_location': 2,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_safety_interface_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'calib_location': 2,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_safety_interface_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"
    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        #'calib_location': 2,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_safety_interface_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] == True, "Expected a success message for the absence of calib_location in mqtt_request, it should use the default value 2"

    #set_safety_interface_config
    si_cfg = response['safety_interface_config']
    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'safety_interface_config':si_cfg
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_safety_interface_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'safety_interface_config':si_cfg
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_safety_interface_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"


    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        #'safety_interface_config':si_cfg
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_safety_interface_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of safety_interface_config in mqtt_request"


    #get_calib_config
    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_calib_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_calib_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"

    sds.set_safety_mode(namespace, name, 2)

    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_calib_config_request')
    msg = sds.get_message()
  
    response = json.loads(msg.payload)
    assert response["success"] == True, "Expected a success message for calib_config:" + response["error_msg"]
    original_data = response['calib_config'].replace("null",str(np.finfo(np.float32).max))
    
    #set_calib_config_request
    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'calib_config':original_data
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_calib_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'calib_config':original_data
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_calib_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        #'calib_config':original_data
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_calib_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of calib_config in mqtt_request"


    #get_application_config
    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_application_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_application_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'get_application_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] == True, "Expected a a successful application_config read"
    original_data = response['application_config']
    #set_application_config
    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'application_config':original_data
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_application_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'application_config':original_data
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_application_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"


    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
    }

    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'set_application_config_request')
    msg = sds.get_message()
    
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of application_config in mqtt_request"


    #triggered calibration
    sds.prepare_for_calibration(namespace, name)

    request_dict = {
        #'camera_namespace': namespace,
        'camera_name': name,
        'json':'calib run',
    }
    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'triggered_calibration_request')
    msg = sds.get_message()
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_namespace in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        #'camera_name': name,
        'json':'calib run',
    }
    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'triggered_calibration_request')
    msg = sds.get_message()
    response = json.loads(msg.payload)
    assert response["success"] != True, "Expected a failure message for the absence of camera_name in mqtt_request"

    request_dict = {
        'camera_namespace': namespace,
        'camera_name': name,
        #'json':'calib run',
    }
    j = json.dumps(request_dict)
    sds.msg = None
    sds.locked = True
    sds.publish(j, 'triggered_calibration_request')
    msg = sds.get_message()
    sds.locked = True
    response = json.loads(msg.payload)
    assert response["error_msg"] == "", "default triggered calibration request is calib run, but failed with error:"+response["error_msg"]

    while True:
        response = sds.receive_triggered_calibration_response()
        LOGGER.info(f"Response: {response}")
        if response['success'] == True or response['error_msg'] != '':
            if response['success'] == True:
                LOGGER.info('Triggered calibraton was successful')
                assert (response['calibration'] != "{}"), "The calibration data received is empty"
                assert type(response['calibration']) != list, "The calibration data received is not a list"
                LOGGER.info('Triggered calibraton data:' + response['calibration'])
            elif response['progress'] == 100.0 and 'Calibration completed but algorithm failed' in response['error_msg']:
                #since it's an issue with the camera field of view, treating it as a success with warning. 
                #Manual adjustment of camera is needed to pass the test.
                LOGGER.warning('Triggered calibraton completed, but algorithm failed. This is treated as a successful completion of the test')
            else:
                assert False, 'Triggered calibration failed with unexpected response:'+str(response)
            break

    #cleanup starts....
    camera.stop()
    rclpy.shutdown()
    LOGGER.info("Test completed")
    #cleanup ends....
