# Copyright 2023 RealSense, Inc. All Rights Reserved.
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
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_dual_camera_launch.py serial_no1:=<serial number of 1st camera> serial_no2:=<serial number of 2nd camera>

"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription, LaunchContext
import launch_ros.actions
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [{'name': 'camera_name1',            'default': 'camera1', 'description': 'camera unique name'},
                    {'name': 'camera_name2',            'default': 'camera2', 'description': 'camera unique name'},
                    {'name': 'camera_namespace1',       'default': 'camera1', 'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2',       'default': 'camera2', 'description': 'camera2 namespace'},
                    {'name': 'enable_color1',           'default': 'true', 'description': 'enable color stream'},
                    {'name': 'enable_color2',           'default': 'true', 'description': 'enable color stream'},
                    {'name': 'enable_depth1',           'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'enable_depth2',           'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'pointcloud.enable1',      'default': 'true', 'description': 'enable pointcloud'},
                    {'name': 'pointcloud.enable2',      'default': 'true', 'description': 'enable pointcloud'},
                    {'name': 'spatial_filter.enable1',  'default': 'true', 'description': 'enable_spatial_filter'},
                    {'name': 'spatial_filter.enable2',  'default': 'true', 'description': 'enable_spatial_filter'},
                    {'name': 'temporal_filter.enable1', 'default': 'true', 'description': 'enable_temporal_filter'},
                    {'name': 'temporal_filter.enable2', 'default': 'true', 'description': 'enable_temporal_filter'},
                    {'name': 'tf.translation.x',        'default': '0.0', 'description': 'x'},
                    {'name': 'tf.translation.y',        'default': '0.0', 'description': 'y'},
                    {'name': 'tf.translation.z',        'default': '0.0', 'description': 'z'},
                    {'name': 'tf.rotation.yaw',         'default': '0.0', 'description': 'yaw'},
                    {'name': 'tf.rotation.pitch',       'default': '0.0', 'description': 'pitch'},
                    {'name': 'tf.rotation.roll',        'default': '0.0', 'description': 'roll'},
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
    # Static transformation from camera1 to camera2
    node = launch_ros.actions.Node(
            name = "my_static_transform_publisher",
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = [context.launch_configurations['tf.translation.x'],
                         context.launch_configurations['tf.translation.y'],
                         context.launch_configurations['tf.translation.z'],
                         context.launch_configurations['tf.rotation.yaw'],
                         context.launch_configurations['tf.rotation.pitch'],
                         context.launch_configurations['tf.rotation.roll'],
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
        OpaqueFunction(function=launch_static_transform_publisher_node),
        launch_ros.actions.Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [ThisLaunchFileDir(), '/rviz/dual_camera_pointcloud.rviz']]
        )
    ])
