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
sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))

from camera_n_mqtt_nodes import CameraNMqttNodes as RSCameraSimulator
from camera_n_mqtt_nodes import launch_descr_with_parameters
import camera_n_mqtt_nodes
from mqtt_client_simulator import MQTTClientSimulator
import logging
import pytest
import rclpy

#LOGGER = logging.getLogger(__name__)
LOGGER = logging.getLogger()

test_params_hmc_d585s = {
    'camera_name': 'D585S',
    'device_type': 'D585S',
    }

@pytest.mark.parametrize("launch_descr_with_parameters",[
    pytest.param(test_params_hmc_d585s, marks=pytest.mark.d585s),
    ]
    ,indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
def test_system_hmc_commands(launch_descr_with_parameters):
    #initialization starts....
    rclpy.init()
    params = launch_descr_with_parameters[1]
    namespace = 'camera'
    name = params['camera_name']
    camera = RSCameraSimulator(namespace, name)
    camera.start()
    sds = MQTTClientSimulator("localhost", 1883)
    sds.start_client()

    if LOGGER.getEffectiveLevel() <= logging.DEBUG:
        os.system("ros2 node list")
    #initialization ends....

    LOGGER.info("Testing enumerate_devices")
    sds.send_enumerate_devices_request(namespace, name)
    response = sds.get_enumerate_devices_response()
    links = [
        {"opcode":0x76, "datasize_expected":8}, #GETRGBAEROI
        {"opcode":0x2A, "param1": 0, "datasize_expected":20}, #GTEMP
        {"opcode":0x2A, "param1": 1, "datasize_expected":20}, #GTEMP
        {"opcode":0x2A, "param1": 2, "datasize_expected":20}, #GTEMP
        {"opcode":0xAC, "datasize_expected":240}, #HEALTH Status
        {"opcode":0x76,"datasize_expected":8},#GETRGBAEROI
        {"opcode":0x75, "param1": 0x1c4, "param2": 0x2a0, "param3": 0x36d, "param4": 0x0ad},#SETRGBAEROI
        {"opcode":0x76,"datasize_expected":8},#GETRGBAEROI
    ]
    for cmd in links:
        param1 = cmd.get("param1", None)
        param2 = cmd.get("param2", None)
        param3 = cmd.get("param3", None)
        param4 = cmd.get("param4", None)
        data = cmd.get("data", None)
        val = sds.send_hwm_command(namespace,name, cmd['opcode'], param1=param1, param2=param2, param3=param3, param4=param4, data=data)
        LOGGER.info(f"Data received {val['result']}")
        size = cmd.get("datasize_expected", 0)
        if size:
            in_data = val['result'][4:]
            assert val['result'] is not None, f"Expected {size} of data, but didn't get any"
            assert len(in_data) == size, f"Expected {size} of data, but got {len(in_data)}" #1st 4 bytes contain the opcode
        else:
            assert len(val['result']) == 4, f"Expected {size} of data, but got {len(val['result'])}" #1st 4 bytes contain the opcode
        header = int.from_bytes(val['result'][0:3], byteorder='little', signed=False)
        assert header == cmd['opcode'], f"Didn't get the opcode back, got {header} instead of {cmd['opcode']}"

    val = sds.send_hwm_command(namespace,name, 0xBC, param1=0)
    LOGGER.debug(val)
    data = val['result'][4:]
    data[3] =1 
    sds.set_safety_mode(namespace, name, 2)
    val = sds.send_hwm_command(namespace,name, 0xBC, param1=1,data=data)
    val = sds.send_hwm_command(namespace,name, 0xBC, param1=0)
    data = val['result'][4:]
    LOGGER.debug(val)
    assert data[3] ==1, "Error: couldn't read back the written data"
    
    val = sds.send_hwm_command(namespace,name,0x76)#GETRGBAEROI
    val = sds.send_hwm_command(namespace,name,0x75, param1=0x1c2, param2= 0x2a0, param3 = 0x36d, param4 = 0x0ad)#SETRGBAEROI
    val = sds.send_hwm_command(namespace,name,0x76)#GETRGBAEROI
    param1l = val['result'][4:6]
    param1 = int.from_bytes(param1l, byteorder='little', signed=False)
    assert param1 == 0x1c2, "Error: written data is not matching with the read one"
    ''' #taking too much of time and inconsistent
    val = sds.send_hwm_command(namespace,name,32, param1=1) #SoC reset
    import time
    time.sleep(15)
    mode = sds.get_integer_param(namespace, name, 'safety_camera.safety_mode')
    print("Mode:", mode)
    time.sleep(2)
    '''
    #cleanup starts....
    camera.stop()
    rclpy.shutdown()
    LOGGER.info("Test completed")
    #cleanup ends....
