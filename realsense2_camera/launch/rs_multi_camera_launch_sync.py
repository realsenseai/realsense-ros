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

# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices and enable the hardware synchronization. 
# As describe in https://dev.intelrealsense.com/docs/multiple-depth-cameras-configuration both devices
# have to be connected using a sync cable. The devices will by default stream asynchronously. 
# Using this launch file one device will operate as master and the other as slave. As a result they will
# capture at exactly the same time and rate. 
# command line example: (to be adapted with the serial numbers or the used devices)
# ros2 launch realsense2_camera rs_multi_camera_launch_sync.py camera_name1:=cam_1 camera_name2:=cam_2 camera_namespace1:=camera camera_namespace2:=camera serial_no1:="'_031422250097'" serial_no2:="'_336222301921'"
# The value of the param can be checked using ros2 param get /camera/cam_1 depth_module.inter_cam_sync_mode and ros2 param get /camera/cam_2 depth_module.inter_cam_sync_mode

"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription, LaunchContext
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [{'name': 'camera_name1', 'default': 'camera1', 'description': 'camera1 unique name'},
                    {'name': 'camera_name2', 'default': 'camera2', 'description': 'camera2 unique name'},
                    {'name': 'camera_namespace1', 'default': 'camera1', 'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2', 'default': 'camera2', 'description': 'camera2 namespace'},
                    {'name': 'depth_module.inter_cam_sync_mode1', 'default': '1', 'description': 'master'},
                    {'name': 'depth_module.inter_cam_sync_mode2', 'default': '2', 'description': 'slave'},
                    ]


def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def launch_static_transform_publisher_node(context : LaunchContext):
    # dummy static transformation from camera1 to camera2
    node = launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0",
                          context.launch_configurations['camera_name1'] + "_link",
                          context.launch_configurations['camera_name2'] + "_link"]
    )
    return [node]

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params1),
                                 'param_name_suffix': '1'}),
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params2),
                                 'param_name_suffix': '2'}),
        OpaqueFunction(function=launch_static_transform_publisher_node)
    ])
