# Copyright 2023 Intel Corporation. All Rights Reserved.
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
import ctypes
import struct
import requests
import json

from pytest_rs_utils import debug_print


def get_profile_config(camera_name):
    config = {
        "Color":{"profile":"rgb_camera.color_profile", "format":'rgb_camera.color_format', "param":"enable_color", "topic":camera_name+'/color/image_raw',},
        "Depth":{"profile":"depth_module.depth_profile", "format":'depth_module.depth_format', "param":"enable_depth", 'topic':camera_name+'/depth/image_rect_raw'},
        "Infrared":{"profile":"depth_module.infra_profile", "format":'depth_module.infra_format', "param":"enable_infra", 'topic':camera_name+'/infra/image_rect_raw'},
        "Infrared1":{"profile":"depth_module.infra_profile", "format":'depth_module.infra1_format',"param":"enable_infra1", 'topic':camera_name+'/infra1/image_rect_raw'},
        "Infrared2":{"profile":"depth_module.infra_profile", "format":'depth_module.infra2_format',"param":"enable_infra2", 'topic':camera_name+'/infra2/image_rect_raw'},
    }
    return config


def get_default_profiles(cap, profile):
    profile1 = "" 
    profile2 = ""
    for profiles in cap:
        if profiles[0] == profile and int(profiles[1].split('x')[0]) != 640:
            profile1 = profiles[1]
            break
    for profiles in cap:
        if profiles[0] == profile and int(profiles[1].split('x')[0]) != int(profile1.split('x')[0]):
            profile2 = profiles[1]
            break
    debug_print(profile + " default profile1:" +  profile1)
    debug_print(profile + " default profile2:" +  profile2)
    return profile1,profile2



device_info_string = "Device info:"

def get_device_info_location(long_data, index=0):
    for line_no in range(index, len(long_data)):
        if device_info_string in long_data[line_no]:
            return line_no
    return len(long_data)

stream_profile_string = "Stream Profiles supported by"
def get_stream_profile_location(long_data, start_index, end_index, profile_string):
    for line_no in range(start_index, end_index):
        if stream_profile_string in long_data[line_no]:
            if profile_string in long_data[line_no]:
                return line_no
    return None

def get_depth_profiles(long_data, start_index, end_index):
    cap = []
    for line_no in range(start_index, end_index):
        if len(long_data[line_no]) == 0:
            break
        debug_print("depth profile processing:" + long_data[line_no])
        enumerate_devices_line_splitted = long_data[line_no].split()
        if len(enumerate_devices_line_splitted) == 7:
            stream0_idx = 1
            stream1_idx = 0
            resolution_idx = 2
            frequency_idx = 5
            format_idx = 3
        elif len(enumerate_devices_line_splitted) == 8:
            stream0_idx = 1
            stream1_idx = 2
            resolution_idx = 3
            frequency_idx = 6
            format_idx = 4
        else:
            assert false, "Seems that the depth profile info format printed by rs-enumerate-devices changed"
        if stream1_idx != 0:
            depth_camera_stream = enumerate_devices_line_splitted[stream0_idx]+enumerate_devices_line_splitted[stream1_idx]
        else:
            depth_camera_stream = enumerate_devices_line_splitted[stream0_idx]
        depth_profile_param = enumerate_devices_line_splitted[resolution_idx]+"x"+enumerate_devices_line_splitted[frequency_idx]
        depth_format_param = enumerate_devices_line_splitted[format_idx]
        debug_print("depth profile added: " + depth_camera_stream, depth_profile_param, depth_format_param)
        cap.append([depth_camera_stream, depth_profile_param, depth_format_param])
    debug_print(cap)
    return cap

 
def get_color_profiles(long_data, start_index, end_index):
    cap = []
    for line_no in range(start_index, end_index):
        if len(long_data[line_no]) == 0:
            break
        debug_print("color profile processing:" + long_data[line_no])
        enumerate_devices_line_splitted = long_data[line_no].split()
        if len(enumerate_devices_line_splitted) == 7:
            stream_idx = 1
            resolution_idx = 2
            frequency_idx = 5
            format_idx = 3
        else:
            assert false, "Seems that the color profile info format printed by rs-enumerate-devices changed"
        color_camera_stream = enumerate_devices_line_splitted[stream_idx]
        color_profile_param = enumerate_devices_line_splitted[resolution_idx]+"x"+enumerate_devices_line_splitted[frequency_idx]
        color_format_param = enumerate_devices_line_splitted[format_idx]
        debug_print("color profile added: " + color_camera_stream, color_profile_param, color_format_param)
        cap.append([color_camera_stream, color_profile_param, color_format_param])
    debug_print(cap)
    return cap

NAME_LINE_INDEX = 1
NAME_LINE_NAME_OFFSET = 4
SERIAL_NO_LINE_INDEX = 2
SERIAL_NO_VALUE_OFFSET = 3
def parse_device_info(long_data, start_index, end_index, device_type, serial_no):
    #after device_info, the next line should have the name and device type
    capability = {}
    debug_print("Searching for data between lines ", str(start_index) + " and " + str(end_index))
    name_line = long_data[start_index+NAME_LINE_INDEX].split()
    if name_line[0] != "Name":
        assert False, "rs-enumerate-devices output format changed"
    if name_line[4] != device_type:
        debug_print("device not matching:" + name_line[NAME_LINE_NAME_OFFSET])
        return None
    debug_print("device matched:" + name_line[NAME_LINE_NAME_OFFSET])
    if serial_no != None:
        #next line after nameline should have the serial_no
        serial_no_line = long_data[start_index+SERIAL_NO_LINE_INDEX].split()
        if serial_no_line[0] != "Serial":
            assert False, "rs-enumerate-devices output format changed"
        if serial_no_line[SERIAL_NO_VALUE_OFFSET] != serial_no:
            debug_print("serial_no not matching:" + serial_no_line[SERIAL_NO_VALUE_OFFSET])
            return None
        debug_print("serial_no matched:" + serial_no_line[SERIAL_NO_VALUE_OFFSET])
    else:
        serial_no = long_data[start_index+SERIAL_NO_LINE_INDEX].split()[SERIAL_NO_VALUE_OFFSET]
 
    capability["device_type"] = device_type
    capability["serial_no"] = serial_no
    depth_profile_index = get_stream_profile_location(long_data, start_index, end_index, "Stereo Module")
    if depth_profile_index != None:
        capability["depth_profile"] = get_depth_profiles(long_data, depth_profile_index+3, end_index)
    rgb_profile_index = get_stream_profile_location(long_data, start_index, end_index, "RGB Camera")
    if rgb_profile_index != None:
        capability["color_profile"] = get_color_profiles(long_data, rgb_profile_index+3, end_index)
    return capability

def get_camera_capabilities(device_type, serial_no=None):
    long_data = os.popen("rs-enumerate-devices -v").read().splitlines()
    debug_print(serial_no)
    index = 0
    while index < len(long_data):
        index = get_device_info_location(long_data, index)
        if index == len(long_data):
            return
        else:
            debug_print("DeviceInfo found at: " + str(index))
        start_index = index
        index += 1
        index = get_device_info_location(long_data, index)
        capability = parse_device_info(long_data, start_index, index, device_type, serial_no)
        if capability != None:
            return capability
    return None    

def get_camera_capabilities_short(device_type, serial_no=None):
    short_data = os.popen("rs-enumerate-devices -s").read().splitlines()
    print(serial_no)
    for line in short_data:
        print(line)
        if device_type in line:
            if serial_no is None or  serial_no == "" :
                print(device_type+ " found in " + line)
                return
            if serial_no in line:
                print(device_type + " with serial_no " + serial_no +" found in " + line)
                return
    print(device_type + " not found")

def check_if_camera_connected(device_type, serial_no=None):
    long_data = os.popen("rs-enumerate-devices -s").read().splitlines()
    debug_print(serial_no)
    index = 0
    for index in range(len(long_data)):
        name_line = long_data[index].split()
        if name_line[0] != "Intel":
            continue
        if name_line[2] != device_type:
            continue
        if serial_no == None:
            return True
        if serial_no == name_line[3]:
            return True

    return False

if __name__ == '__main__':
    device_type = 'D455'
    serial_no = None
    if len(sys.argv) > 1:
        device_type = sys.argv[1]
    if len(sys.argv) > 2:
        serial_no = sys.argv[2]
    cap = get_camera_capabilities(device_type, serial_no)
    print("Capabilities:")
    print(cap)