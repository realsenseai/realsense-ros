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
# Use this launch file to launch a device and visualize pointcloud along with the camera model D405.
# The Parameters available for definition in the command line for the camera are described in rs_launch.configurable_parameters
# command line example:
# ros2 launch realsense2_camera rs_d405_pointcloud_launch.py

"""Launch realsense2_camera node."""
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os
import sys
import pathlib
import xacro
import tempfile
from ament_index_python.packages import get_package_share_directory
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                    {'name': 'camera_namespace',             'default': 'camera', 'description': 'camera namespace'},
                    {'name': 'device_type',                  'default': "d405", 'description': 'choose device by type'},
                    {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                    {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'pointcloud.enable',            'default': 'true', 'description': 'enable pointcloud'},
                   ]

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path

def set_configurable_parameters(local_params):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])


def generate_launch_description():
    params = rs_launch.configurable_parameters
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_d405_camera.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) + 
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                kwargs = {'params' : set_configurable_parameters(params)}
        ),
        launch_ros.actions.Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [ThisLaunchFileDir(), '/rviz/urdf_pointcloud.rviz']],
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        launch_ros.actions.Node(
            name='model_node',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='',
            output='screen',
            arguments=[urdf]
        )
    ])
