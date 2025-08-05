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
import os
import sys
import subprocess
import time

import launch

import launch_pytest
from launch_pytest.tools import process as process_tools

import pytest
from setuptools import find_packages

sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import get_rosbag_file_path


'''
This is a pytest fixture used by the lauch_pytest

This function triggers the launch of a process. This mimics the 
behavior of the customer usecases where they use the launch file directly.

So this test can be used as a template to create customer scenarios and also to 
test the ra_launch.py script 

This function uses default parameters from the launch file and overwrites the required
parameters.
'''

@pytest.fixture
def start_camera():
    params = pytest_rs_utils.get_default_params() 
    rosfile = get_rosbag_file_path("outdoors_1color.bag")
    params['camera_name'] = 'camera'
    params['rosbag_filename'] = rosfile
    params['color_width'] = '0'
    params['color_height'] = '0'
    params['depth_width'] = '0'
    params['depth_height'] = '0'
    params['infra_width'] = '0'
    params['infra_height'] = '0'
    params_str = pytest_rs_utils.get_params_string_for_launch(params)
    cmd_params=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py'] + params_str.split(' ')
    time.sleep(1)
    return launch.actions.ExecuteProcess(
        cmd=cmd_params,
        shell=True,
        cached_output=True,
    )
'''
This function specifies the processes to be run during the test.
The tester can add more such functions to start specific processes
or do specific actions.
'''
@launch_pytest.fixture
def launch_description(start_camera):
    """Launch a simple process to start the camera"""
    return launch.LaunchDescription([
        start_camera,
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])


'''
This is a test that can be used as a reference for other tests where it checks if
the realsense node has come up or not.
In this test, once the camera is detected, the camera itself is killed. But the user
can do different operations based on the testcase requirements
'''
#test skipped as it fails in industrial_ci

@pytest.mark.skip
@pytest.mark.launch(fixture=launch_description)
def test_start_camera(start_camera, launch_context):
    def validate_output(output):
        # by now, the camera would have started.
        service_list = subprocess.check_output(['ros2', 'node', 'list']).decode("utf-8")
        is_node_up = '/camera/camera' in service_list
        print(service_list)
        assert is_node_up, 'Node is NOT UP'
        print ('Node is UP')
        print ('*'*8 + ' Killing ROS ' + '*'*9)
        pytest_rs_utils.kill_realsense2_camera_node()

    process_tools.assert_output_sync(
        launch_context, start_camera, validate_output, timeout=5)
    yield
    '''
    the next code checks the return value of the start_camera. 
    Since it was killed, -2 is expected to indicate shutdown
    '''
    assert start_camera.return_code == -2 
