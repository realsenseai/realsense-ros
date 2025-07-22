# Copyright 2025 Intel Corporation. All Rights Reserved.
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
# Use this launch file to launch a device and align depth to infrared 2.
# The Parameters available for definition in the command line for the camera are described in rs_launch.configurable_parameters
# command line example:
# ros2 launch realsense2_camera rs_align_depth_to infra_launch.py

"""Launch realsense2_camera node."""
from launch import LaunchDescription
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

local_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                    {'name': 'camera_namespace',             'default': 'camera', 'description': 'camera namespace'},
                    {'name': 'enable_color',                 'default': 'false', 'description': 'enable color stream'},
                    {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'enable_infra2',                'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'enable_sync',                  'default': 'true', 'description': 'enable sync mode'},
                    {'name': 'pointcloud.enable',            'default': 'true', 'description': 'enable pointcloud'},
                    {'name': 'pointcloud.stream_filter',     'default': '3', 'description': 'pointcloud with infrared'},
                   ]

def set_configurable_parameters(local_params):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])


def generate_launch_description():
    params = rs_launch.configurable_parameters
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) + 
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                kwargs = {'params' : set_configurable_parameters(params)}
        )
    ])
