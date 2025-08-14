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

import os
import sys
import time
import threading

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch import LaunchDescription
from launch import LaunchService

import launch_pytest

import rclpy
from rclpy.node import Node
import logging
LOGGER = logging.getLogger()

import rs_launch

class CameraNMqttNodes(Node):#, threading.Thread):
    def __init__(self, namespace="camera", name='camera', device_type_="D6585S"):
        super().__init__(name)
        self.dummy = True
        return
    def run(self):
        return

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

    def start(self):
        if self.dummy == True:
            self.wait_for_node("realsense_ros_mqtt_bridge_node")
            return
    
        

    #start of dummy functions
    def stop(self):
        return
    def create_device_info_service(self):
        return
    def add_parameters(self,params):
        return
    def start_publish_color_frame(self):
        return
    def create_application_config_service(self):
        return
    def create_calib_config_service(self):
        return
    def create_safety_interface_config_service(self):
        return

    #end of dummy functions 

def get_camera_device_info(device_type, serial_no):
    short_data = os.popen("rs-enumerate-devices -S").read().splitlines()
    print(serial_no)
    line_found = 0
    found = False
    for line in short_data:
        print(line)
        if device_type in line:
            if serial_no in line:
                found = True
                break
        line_found += 1
    if found == True:
        '''
        rs-enumerate-devices -S
        Device Name                   Serial Number       Firmware Version
        Intel RealSense D585S         416422320067        8.17.17151.218
        Device info: 
            Name                          : Intel RealSense D585S
            Serial Number                 : 416422320067
            Firmware Version              : 8.17.17151.218
            Physical Port                 : /sys/devices/pci0000:00/0000:00:14.0/usb2/2-5/2-5:1.0/video4linux/video0
            Debug Op Code                 : 180
            Advanced Mode                 : NO
            Product Id                    : 0B6B
            Camera Locked                 : YES
            Usb Type Descriptor           : 3.2
            Product Line                  : D500
            Firmware Update Id            : 416422320067
            Smcu Fw Version               : 2.0.6.76
        '''
        #if this order changes, test will fail
        length = len(short_data)
        LOGGER.debug(device_type + " with serial_no " + serial_no +" found in " + short_data[line_found])
        for i in range(line_found, line_found+9):
            LOGGER.debug(short_data[i])
        device_info = {}
        #3rd line contains serial no
        line_no = line_found+3
        if not "Serial Number" in short_data[line_no]:
            return None
        split_line = short_data[line_no].split()
        device_info['serial_number'] = split_line[3]
        #4th line contains Firmware Version
        line_no = line_found+4
        if not "Firmware Version" in short_data[line_no]:
            return None
        split_line = short_data[line_no].split()
        device_info['firmware_version'] = split_line[3]
        #5th line contains physical port
        line_no = line_found+5
        if not "Physical Port" in short_data[line_no]:
            return None
        split_line = short_data[line_no].split()
        device_info['physical_port'] = split_line[3]
        #10th line contains Usb Type Descriptor 
        line_no = line_found+10
        if not "Usb Type Descriptor" in short_data[line_no]:
            return None
        split_line = short_data[line_no].split()
        device_info['usb_type_descriptor'] = split_line[4]
        #12th line contains Usb Type Descriptor 
        line_no = line_found+12
        if not "Firmware Update Id" in short_data[line_no]:
            return None
        split_line = short_data[line_no].split()
        device_info['firmware_update_id'] = split_line[4]
        return device_info
    return None

'''
get the default parameters from the launch script so that the test doesn't have to
get updated for each change to the parameter or default values 
'''
def get_default_params():
    params = {}
    for param in rs_launch.configurable_parameters:
        params[param['name']] = param['default']
    return params

''' 
The format used by rs_launch.py and the LuanchConfiguration yaml files are different,
so the params reused from the rs_launch has to be reformated to be added to yaml file.
'''
def convert_params(params):
    cparams = {}
    def strtobool (val):
        val = val.lower()
        if val == 'true':
            return True
        elif val == 'false':
            return False
        else:
            raise ValueError("invalid truth value %r" % (val,))
    for key, value in params.items():
        try:
            cparams[key] = int(value)
        except ValueError:
            try:
                cparams[key] = float(value)
            except ValueError:
                try:
                    cparams[key] = strtobool(value)
                except ValueError:
                    cparams[key] = value.replace("'","")
    return cparams

def get_rs_node_description(params):
    import tempfile
    import yaml
    tmp_yaml = tempfile.NamedTemporaryFile(prefix='launch_rs_',delete=False)
    params = convert_params(params)
    ros_params = {"ros__parameters":params}

    camera_params = {params['camera_namespace'] +"/"+params['camera_name']: ros_params}
    with open(tmp_yaml.name, 'w') as f:
        yaml.dump(camera_params, f)

    '''
    comment out the '#prefix' line, if you like gdb and want to debug the code, you may have to do more
    if you have more than one rs node.
    '''
    loglevel = "info"
    if LOGGER.getEffectiveLevel() <= logging.DEBUG:
        loglevel = "debug"
    print("loglevel", loglevel)
    return launch_ros.actions.Node(
        package='realsense2_camera',
        namespace=params["camera_namespace"],
        name=params["camera_name"],
        #prefix=['xterm -e gdb -ex=run --args'],
        executable='realsense2_camera_node',
        parameters=[tmp_yaml.name],
        output='screen',
        arguments=['--ros-args', '--log-level', loglevel],
        emulate_tty=True,
    )

@launch_pytest.fixture
def launch_descr_with_parameters(request):
    params = request.param
    changed_params = request.param
    params = get_default_params()
    for key, value in changed_params.items():
        params[key] = value
    if  'camera_name' not in changed_params:
        params['camera_name'] = 'camera_with_params'
    device_type = LaunchConfiguration('device_type', default=params['device_type'])
    loglevel = "info"
    if LOGGER.getEffectiveLevel() <= logging.DEBUG:
        loglevel = "debug"
    ld = LaunchDescription([
            launch_ros.actions.Node(
                package='realsense2_ros_mqtt_bridge',
                executable='realsense2_ros_mqtt_bridge',
                name='realsense2_ros_mqtt_bridge',
                arguments=['--ros-args', '--log-level', loglevel],
                output='screen',
                respawn=True,
                respawn_delay=1,
            ),
            get_rs_node_description(params),
            DeclareLaunchArgument(
                'device_type',
                default_value=device_type,
                description='Specifying the device type'),
            launch_pytest.actions.ReadyToTest(),
        ]),params
    return ld
