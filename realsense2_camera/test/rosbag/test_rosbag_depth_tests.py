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
import itertools


import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
from tf2_msgs.msg import TFMessage as msg_TFMessage

import numpy as np

sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_parameters
from pytest_rs_utils import delayed_launch_descr_with_parameters
from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_node_heirarchy


test_params_depth_points_cloud_1 = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'Points_cloud_1',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    'pointcloud.enable': 'true'
    }

'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py depth_w_cloud1 points_cloud_1"
This rs2_test command fails once in a while because of the delay in bringing up of the test node misses
some of the points cloud data. This test adds a delay in bringing up the RS node.

Even then, the test fails sometimes due to the avg and epsilon value of points cloud that was set for 
a different rosbag file (or so its seems.)
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_depth_points_cloud_1],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestDepthPointsCloud1(pytest_rs_utils.RsTestBaseClass):
    def test_depth_points_cloud_1(self,delayed_launch_descr_with_parameters):
        ''' 
        Using the delayed launch of the ROS node so that the below data can be extracted.
        This can be done after also as in the case of test_points_cloud_1, but even with that
        since there are two callbacks, the initial few frames/data gets lost.
        '''
        params = delayed_launch_descr_with_parameters[1]
        self.rosbag = params["rosbag_filename"]
        data2 = pytest_rs_utils.ImageDepthGetData(params["rosbag_filename"])
        data1 = {'width': [660353, 3300], 
                'height': [1], 
                'avg': [np.array([ 1.28251814, -0.15839984, 4.82235184, 80, 160, 240])], 
                'epsilon': [0.04, 5]}
        themes = [
        {'topic':get_node_heirarchy(params)+'/depth/color/points',
         'msg_type':msg_PointCloud2,
         'expected_data_chunks':1,
         'data':data1
        },
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)


test_params_static_tf_1 = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'Static_tf1',
    'color_width': '0',
    'color_height': '0',
    "static_tf":True,
    'infra_height': '0',
    'enable_infra1':'true', 
    'enable_infra2':'true'
    }
'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py static_tf_1"
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_static_tf_1],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestStaticTf1(pytest_rs_utils.RsTestBaseClass):
    def test_static_tf_1(self,delayed_launch_descr_with_parameters):
        self.params = delayed_launch_descr_with_parameters[1]
        self.rosbag = self.params["rosbag_filename"]
        themes = [
        {'topic':get_node_heirarchy(self.params)+'/color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'static_tf':True,
        },
        {'topic':'/tf_static',
         'msg_type':msg_TFMessage,
         'expected_data_chunks':1,
         'qos' :  QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST,)#static
        }
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+self.params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        expected_data = {(self.params['camera_name']+'_link', self.params['camera_name']+'_color_frame'): ([-0.00010158783697988838, 0.014841210097074509, -0.00022671300393994898], [-0.0008337442995980382, 0.0010442184284329414, -0.0009920650627464056, 0.9999986290931702]), 
                                                          (self.params['camera_name']+'_link', self.params['camera_name']+'_depth_frame'): ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]), 
                                                          (self.params['camera_name']+'_link', self.params['camera_name']+'_infra1_frame'): ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]), 
                                                          (self.params['camera_name']+'_depth_frame', self.params['camera_name']+'_infra1_frame'): ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]), 
                                                          (self.params['camera_name']+'_depth_frame', self.params['camera_name']+'_color_frame'): ([-0.00010158783697988838, 0.014841210097074509, -0.00022671300393994898], [-0.0008337442995980382, 0.0010442184284329414, -0.0009920650627464056, 0.9999986290931702]), 
                                                          (self.params['camera_name']+'_infra1_frame', self.params['camera_name']+'_color_frame'): ([-0.00010158783697988838, 0.014841210097074509, -0.00022671300393994898], [-0.0008337442995980382, 0.0010442184284329414, -0.0009920650627464056, 0.9999986290931702])}
        frame_ids = [self.params['camera_name']+'_link', self.params['camera_name']+'_depth_frame', self.params['camera_name']+'_infra1_frame', self.params['camera_name']+'_infra2_frame', self.params['camera_name']+'_color_frame', self.params['camera_name']+'_fisheye_frame', self.params['camera_name']+'_pose']
        coupled_frame_ids = [xx for xx in itertools.combinations(frame_ids, 2)]
        data = self.node.pop_first_chunk('/tf_static')
        coupled_frame_ids = [xx for xx in itertools.combinations(frame_ids, 2)]
        tfs_data = self.get_transform_data(data, coupled_frame_ids, is_static=True)
        ret = pytest_rs_utils.staticTFTest(tfs_data, expected_data)
        assert ret[0], ret[1]
        return ret[0]


test_params_non_existing_rosbag = {"rosbag_filename":"non_existent.bag",
    'camera_name': 'non_existing_rosbag',
    }
'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py static_tf_1"
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_non_existing_rosbag],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestNonExistingRosbag(pytest_rs_utils.RsTestBaseClass):
    def test_non_existing_rosbag(self,delayed_launch_descr_with_parameters):
        params = delayed_launch_descr_with_parameters[1]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.node.wait_for_node(params['camera_name'],timeout=2.0)
            assert not ret[0], ret[1]
        finally:
            self.shutdown()



test_params_align_depth_color_1 = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'Align_Depth_Color_1',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    'align_depth.enable':'true'
    }
'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py align_depth_color_1"
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_align_depth_color_1],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestAlignDepthColor(pytest_rs_utils.RsTestBaseClass):
    def test_align_depth_color_1(self,delayed_launch_descr_with_parameters):
        params = delayed_launch_descr_with_parameters[1]
        data = pytest_rs_utils.ImageDepthInColorShapeGetData(params["rosbag_filename"])
        themes = [
        {'topic':get_node_heirarchy(params)+'/aligned_depth_to_color/image_raw',
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
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)


test_params_align_depth_infra_1 = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'Align_Depth_Infra_1',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    'enable_infra1':'true',
    'enable_infra2':'true',
    'align_depth.enable':'true',
    }
'''
This test was ported from rs2_test.py
the command used to run is "python3 realsense2_camera/scripts/rs2_test.py align_depth_ir_1"
'''
''' 
The following testcase is skipped due to a possible issue. It can be re-enabled once fixed.
    The ROS2 node doesn't publish the aligned depth data in /aligned_depth_to_infra1/image_raw.
    It seems that the aligned depth data is generated only for color, not for infrared.

    Hardwired the generation to Infra1 instead of color and test passed. 
    (Line nos 195 and 531 in realsense2_camera/src/base_realsense_node.cpp)

    However a proper fix needs to have all the alignments, such as color, infra1, infra2, if needed.
'''
@pytest.mark.skip
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_align_depth_infra_1],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestAlignDepthInfra1(pytest_rs_utils.RsTestBaseClass):
    def test_align_depth_infra_1(self,delayed_launch_descr_with_parameters):
        params = delayed_launch_descr_with_parameters[1]
        self.rosbag = params["rosbag_filename"]
        #data = pytest_rs_utils.ImageDepthInColorShapeGetData(params["rosbag_filename"])
        themes = [
        {'topic':get_node_heirarchy(params)+'/aligned_depth_to_infra1/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         #'data':data
        }
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        data = pytest_rs_utils.ImageDepthInInfra1ShapeGetData(self.rosbag)
        themes[0]["data"] = data
        return super().process_data(themes)
