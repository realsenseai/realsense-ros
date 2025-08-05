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

'''
Launch realsense2_camera node & a frame latency printer node, 
This tool allow the user the evaluate the reduction of the frame latency when intra-process communication is used.
Run syntax: ros2 launch realsense2_camera rs_intra_process_demo_launch.py intra_process_comms:=true
Note: 
*   Running this tool require building with build tools flag on (colcon build --cmake-args '-DBUILD_TOOLS=ON')
*   Currently default for color stream only
'''
import sys
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Make sure required packages can be found
process = subprocess.run(['ros2','component', 'types'],
                         stdout=subprocess.PIPE, 
                         universal_newlines=True)

rs_node_class=  'RealSenseNodeFactory'
rs_latency_tool_class = 'FrameLatencyNode'

if process.stdout.find(rs_node_class) == -1 or process.stdout.find(rs_latency_tool_class) == -1 :
    sys.exit('Cannot locate all required node components (' + rs_node_class + ',' + rs_latency_tool_class + ') on the available component list\n' + process.stdout + \
    '\nplease make sure you run "colcon build --cmake-args \'-DBUILD_TOOLS=ON\'" command before running this launch file')


realsense_node_params = [{'name': 'serial_no',              'default': "''", 'description': 'choose device by serial number'},
                         {'name': 'usb_port_id',            'default': "''", 'description': 'choose device by usb port id'},
                         {'name': 'device_type',            'default': "''", 'description': 'choose device by type'},
                         {'name': 'log_level',              'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                         {'name': 'rgb_camera.profile',     'default': '0,0,0', 'description': 'color image width'},
                         {'name': 'enable_color',           'default': 'true', 'description': 'enable color stream'},
                         {'name': 'enable_depth',           'default': 'true', 'description': 'enable depth stream'},
                         {'name': 'enable_infra',           'default': 'false', 'description': 'enable infra stream'},
                         {'name': 'enable_infra1',          'default': 'true', 'description': 'enable infra1 stream'},
                         {'name': 'enable_infra2',          'default': 'true', 'description': 'enable infra2 stream'},
                         {'name': 'enable_gyro',            'default': 'true', 'description': "enable gyro stream"},
                         {'name': 'enable_accel',           'default': 'true', 'description': "enable accel stream"},
                         {'name': 'unite_imu_method',       'default': "1", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                         {'name': 'intra_process_comms',    'default': 'true', 'description': "enable intra-process communication"},
                         {'name': 'enable_sync',            'default': 'true', 'description': "'enable sync mode'"},
                         {'name': 'pointcloud.enable',      'default': 'true', 'description': ''},
                         {'name': 'enable_rgbd',            'default': 'true', 'description': "'enable rgbd topic'"},
                         {'name': 'align_depth.enable',     'default': 'true', 'description': "'enable align depth filter'"},
                         {'name': 'publish_tf',             'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                         {'name': 'tf_publish_rate',        'default': '1.0', 'description': '[double] rate in HZ for publishing dynamic TF'},
                        ]

frame_latency_node_params = [{'name': 'topic_name', 'default': '/camera/color/image_raw', 'description': 'topic to which latency calculated'},
                             {'name': 'topic_type', 'default': 'image', 'description': 'topic type [image|points|imu|metadata|camera_info|rgbd|imu_info|tf]'},
                            ]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():


    return LaunchDescription(declare_configurable_parameters(realsense_node_params) + 
                             declare_configurable_parameters(frame_latency_node_params) +[
        ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense2_camera',
                    namespace='',
                    plugin='realsense2_camera::' + rs_node_class,
                    name="camera",
                    parameters=[set_configurable_parameters(realsense_node_params)],
                    extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}]) ,
                ComposableNode(
                    package='realsense2_camera',
                    namespace='',
                    plugin='rs2_ros::tools::frame_latency::' + rs_latency_tool_class,
                    name='frame_latency',
                    parameters=[set_configurable_parameters(frame_latency_node_params)],
                    extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}]) ,
                ],
            output='screen',
            emulate_tty=True, # needed for display of logs
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )])


