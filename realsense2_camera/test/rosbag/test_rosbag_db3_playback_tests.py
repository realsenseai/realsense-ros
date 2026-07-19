# Copyright 2026 RealSense, Inc. All Rights Reserved.
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

# .db3 counterparts of the legacy .bag rosbag tests. Each drives the
# realsense2_camera node with a ROS2 .db3 handed to its `rosbag_filename`
# device-playback path (node -> librealsense load_device(.db3) -> pipeline),
# proving the node accepts a .db3 the same way it accepts a .bag. This is
# distinct from `ros2 bag play` (test_rosbag_db3_play_test.py): here the frames
# - including the node-pipeline-produced aligned_depth_to_color - come out of
# the node, not out of a stored topic.
#
# The .db3 is rs-convert'ed from the original .bag; ground truth is still read
# from that .bag (importRosbag cannot read .db3). When the installed
# librealsense predates .db3 playback (rosdistro release without BUILD_ROSBAG2),
# the whole module skips - matching the existing .db3 tests.

import os
import sys

import pytest

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu

sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_parameters
from pytest_rs_utils import delayed_launch_descr_with_parameters
from pytest_rs_utils import launch_descr_with_yaml
from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_db3_file_path
from pytest_rs_utils import db3_playback_supported
from pytest_rs_utils import get_node_heirarchy

_DB3_OK = db3_playback_supported()
pytestmark = pytest.mark.skipif(
    not _DB3_OK,
    reason="installed librealsense cannot play a .db3 (needs BUILD_ROSBAG2 / rs-convert --output-db3); rebuild from source")

_COLOR_BAG = "outdoors_1color.bag"
_IMU_BAG = "D435i_Depth_and_IMU_Stands_still.bag"

def _db3(bag_filename):
    # Deferred conversion: only convert when .db3 playback is supported. When it
    # is not, the module is skipped and this placeholder path is never launched.
    return get_db3_file_path(bag_filename) if _DB3_OK else "unsupported.db3"


test_params_color = {"rosbag_filename": _db3(_COLOR_BAG),
    'camera_name': 'Vis2_Cam_db3',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_color],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestVis2Db3(pytest_rs_utils.RsTestBaseClass):
    def test_vis_2_db3(self,delayed_launch_descr_with_parameters):
        params = delayed_launch_descr_with_parameters[1]
        data = pytest_rs_utils.ImageColorGetData(get_rosbag_file_path(_COLOR_BAG))
        themes = [
        {'topic':get_node_heirarchy(params)+'/color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'data':data
        }
        ]
        try:
            '''
            initialize, run and check the data
            '''
            self.init_test("RsTest"+params['camera_name'])
            self.wait_for_node(params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)


test_params_depth = {"rosbag_filename": _db3(_COLOR_BAG),
    'camera_name': 'Depth_Avg_db3',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }
@pytest.mark.rosbag
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_depth],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestDepthAvgDb3(pytest_rs_utils.RsTestBaseClass):
    def test_depth_avg_db3(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        data = pytest_rs_utils.ImageDepthGetData(get_rosbag_file_path(_COLOR_BAG))
        themes = [
        {'topic':get_node_heirarchy(params)+'/depth/image_rect_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'data':data
        }
        ]
        try:
            '''
            initialize, run and check the data
            '''
            self.init_test("RsTest"+params['camera_name'])
            self.wait_for_node(params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)


test_params_align_depth = {
    "rosbag_filename": _db3(_COLOR_BAG),
    'camera_name': 'align_db3',
    'enable_color': 'true',
    'enable_depth': 'true',
    'depth_module.profile': '1280x720x30',
    'rgb_camera.profile': '640x480x30',
    'align_depth.enable': 'true',
    }
@pytest.mark.rosbag
@pytest.mark.launch(fixture=launch_descr_with_yaml)
@pytest.mark.parametrize("launch_descr_with_yaml", [test_params_align_depth],indirect=True)
class TestAlignDepthDb3(pytest_rs_utils.RsTestBaseClass):
    def test_align_depth_db3(self, launch_descr_with_yaml):
        params = launch_descr_with_yaml[1]
        themes = [
            {'topic': get_node_heirarchy(params)+'/color/image_raw', 'msg_type': msg_Image,
             'expected_data_chunks': 1, 'width': 640, 'height': 480},
            {'topic': get_node_heirarchy(params)+'/depth/image_rect_raw', 'msg_type': msg_Image,
             'expected_data_chunks': 1, 'width': 1280, 'height': 720},
            {'topic': get_node_heirarchy(params)+'/aligned_depth_to_color/image_raw', 'msg_type': msg_Image,
             'expected_data_chunks': 1, 'width': 640, 'height': 480}
        ]
        try:
            '''
            initialize, run and check the data
            '''
            self.init_test('RsTest'+params['camera_name'])
            self.wait_for_node(params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()


test_params_accel = {"rosbag_filename": _db3(_IMU_BAG),
    'camera_name': 'Accel_Cam_db3',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    'enable_accel': 'true',
    'accel_fps': '0.0'
    }
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_accel],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestAccelUpDb3(pytest_rs_utils.RsTestBaseClass):
    def test_accel_up_db3(self,delayed_launch_descr_with_parameters):
        params = delayed_launch_descr_with_parameters[1]
        data = pytest_rs_utils.AccelGetDataDeviceStandStraight(get_rosbag_file_path(_IMU_BAG))
        themes = [
        {'topic':get_node_heirarchy(params)+'/accel/sample',
         'msg_type':msg_Imu,
         'expected_data_chunks':1,
         'data':data
        }
        ]
        try:
            '''
            initialize, run and check the data
            '''
            self.init_test("RsTest"+params['camera_name'])
            self.wait_for_node(params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)
