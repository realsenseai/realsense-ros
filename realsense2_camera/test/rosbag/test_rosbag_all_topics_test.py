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

import pytest

import numpy as np
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
from realsense2_camera_msgs.msg import Extrinsics as msg_Extrinsics
from realsense2_camera_msgs.msg import Metadata as msg_Metadata

from array import array
from builtin_interfaces.msg import Time
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
import time


sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_parameters

from pytest_rs_utils import delayed_launch_descr_with_parameters
from pytest_rs_utils import get_rosbag_file_path
from pytest_rs_utils import get_node_heirarchy


test_params_all_topics = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'AllTopics',
    'enable_infra1':'true',
    'enable_infra2':'true',
    'align_depth.enable':'true',
    }
'''
To test all topics published
'''
'''
To test all topics published

The test doesn't work in CI due to sync. The unlike the live camera, rosbag node publishes the extrinsics only once,
not every time the subscription is created. The CI due to limited resource can start the ros node much earlier than 
the testcase, hence missing the published data by the test 
'''
@pytest.mark.rosbag
@pytest.mark.skipif (os.getenv('RS_ROS_REGRESSION', "not found") == "not found",reason="The test doesn't work in CI")
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_all_topics],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestAllTopics(pytest_rs_utils.RsTestBaseClass):
    def test_all_topics(self,delayed_launch_descr_with_parameters):
 
        params = delayed_launch_descr_with_parameters[1]
        self.rosbag = params["rosbag_filename"]

        depth_to_infra_extrinsics_data = msg_Extrinsics()
        depth_to_infra_extrinsics_data.rotation = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        depth_to_infra_extrinsics_data.translation =[-0., -0., -0.]

        depth_to_color_extrinsics_data = msg_Extrinsics()
        depth_to_color_extrinsics_data.rotation=array('f',[ 0.99999666,  0.00166541,  0.00198587, -0.00166956,  0.99999642,
                        0.00208678, -0.00198239, -0.00209009,  0.99999583])
        depth_to_color_extrinsics_data.translation=array('f',[ 0.01484134, -0.00020221,  0.00013059])
        data = pytest_rs_utils.ImageColorGetData(params["rosbag_filename"])
        themes = [
        {
         'topic':get_node_heirarchy(params)+'/extrinsics/depth_to_color',
         'msg_type':msg_Extrinsics,
         'expected_data_chunks':1,
         'data':depth_to_color_extrinsics_data
        },
        {
         'topic':get_node_heirarchy(params)+'/extrinsics/depth_to_infra1',
         'msg_type':msg_Extrinsics,
         'expected_data_chunks':1,
         'data':depth_to_infra_extrinsics_data
        },
        {'topic':get_node_heirarchy(params)+'/color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         'data':data
        },
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            time.sleep(0.5)
            assert self.process_data(themes), "Data check failed, probably the rosbag file changed?"
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)

test_params_metadata_topics = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'MetadataTopics',
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
Meta data tests are not consistent, values are different every time.
Need a better way to check, so skipping the data checks
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_metadata_topics],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestMetaDataTopics(pytest_rs_utils.RsTestBaseClass):
    def test_metadata_topics(self,delayed_launch_descr_with_parameters):
        ''' 
        current rosbag file doesn't have color data 
        '''
        params = delayed_launch_descr_with_parameters[1]
        self.rosbag = params["rosbag_filename"]

        color_metadata = msg_Metadata()
        color_metadata.json_data = '{"frame_number":39,"clock_domain":"system_time","frame_timestamp":1508282881033.132324,"frame_counter":-8134432827560165376,"time_of_arrival":1508282881033}'

        depth_metadata = msg_Metadata()
        depth_metadata.json_data ='{"frame_number":13048,"clock_domain":"system_time","frame_timestamp":1508282880968.727295,"frame_counter":-327065418902536192,"time_of_arrival":1508282880968}'
        infra1_metadata = msg_Metadata()
        infra1_metadata.json_data ='{"frame_number":10938,"clock_domain":"system_time","frame_timestamp":1508282880964.985352,"frame_counter":0,"time_of_arrival":1508282880964}'

        themes = [
        {
         'topic':get_node_heirarchy(params)+'/color/metadata',
         'msg_type':msg_Metadata,
         'expected_data_chunks':1,
         #'data':color_metadata
        },
        {
         'topic':get_node_heirarchy(params)+'/depth/metadata',
         'msg_type':msg_Metadata,
         'expected_data_chunks':1,
         #'data':depth_metadata
        },
        {
         'topic':get_node_heirarchy(params)+'/infra1/metadata',
         'msg_type':msg_Metadata,
         'expected_data_chunks':1,
         #'data':infra1_metadata
        },
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes), "Data check failed, probably the rosbag file changed?"
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)

test_params_camera_info_topics = {"rosbag_filename":get_rosbag_file_path("outdoors_1color.bag"),
    'camera_name': 'CameraInfoTopics',
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
To test all topics published
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_camera_info_topics],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestCamerInfoTopics(pytest_rs_utils.RsTestBaseClass):
    def test_camera_info_topics(self,delayed_launch_descr_with_parameters):
        ''' 
        current rosbag file doesn't have color data 
        '''
        params = delayed_launch_descr_with_parameters[1]
        self.rosbag = params["rosbag_filename"]
        '''
        The test is hardwired to ensure the rosbag file is not changed.
        The function CameraInfoColorGetData requires changes to adapt to the changes
        made by the rosbag reader on extrincsic
        color_data = pytest_rs_utils.CameraInfoColorGetData(self.rosbag)
        '''
        color_data = CameraInfo(header=Header(stamp=Time(sec=1508282881, nanosec=33132324),
                                        frame_id=params['camera_name']+"_color_optical_frame"),
                                        width=640,
                                        height=480,
                                        distortion_model='plumb_bob', 
                                        d=[0.0, 0.0, 0.0, 0.0, 0.0], 
                                        k=[616.0769043,0.0,311.48977661,0.0,615.79931641,241.15310669,0.0,0.0,1.0], 
                                        r=[1.0,.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0], 
                                        p=[616.0769043,0.0,311.48977661,0.0,0.0,615.79931641,241.15310669,0.0,0.0,0.0,1.0,0.0],
                                        binning_x=0,
                                        binning_y=0,
                                        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
        depth_data = CameraInfo(header=Header(stamp=Time(sec=1508282880, nanosec=968727295), 
                                        frame_id=params['camera_name']+'_depth_optical_frame'), 
                                        height=720, 
                                        width=1280, 
                                        distortion_model='plumb_bob', 
                                        d=[0.0, 0.0, 0.0, 0.0, 0.0], 
                                        k=[976.7364502,0.0,636.62762451,0.0,976.7364502,373.01535034,0.0,0.0,1.0],
                                        r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], 
                                        p=[976.7364502, 0., 636.62762451, 0., 0., 976.7364502, 373.01535034, 0., 0., 0., 1., 0.], 
                                        binning_x=0, 
                                        binning_y=0, 
                                        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
        
        infra1_data =CameraInfo(header=Header(stamp=Time(sec=1508282880, nanosec=964985352), 
                                              frame_id=params['camera_name']+'_infra1_optical_frame'), 
                                              height=720, 
                                              width=1280, 
                                              distortion_model='plumb_bob', 
                                              d=[0.0, 0.0, 0.0, 0.0, 0.0], 
                                              k=[976.7364502, 0., 636.62762451, 0., 976.7364502 , 373.01535034, 0., 0., 1.], 
                                              r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], 
                                              p=[976.7364502, 0., 636.62762451, 0., 0., 976.7364502, 373.01535034, 0., 0., 0., 1., 0.], 
                                              binning_x=0, 
                                              binning_y=0, 
                                              roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
        
        depth_to_color_data = CameraInfo(header=Header(stamp=Time(sec=1508282880, nanosec=968727295), 
                                                frame_id=params['camera_name']+'_color_optical_frame'), 
                                                height=480, 
                                                width=640, 
                                                distortion_model='plumb_bob', 
                                                d=[0.0, 0.0, 0.0, 0.0, 0.0], 
                                                k=[616.0769043, 0., 311.48977661, 0., 615.79931641, 241.15310669, 0., 0., 1.], 
                                                r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], 
                                                p=[616.0769043, 0., 311.48977661, 0., 0., 615.79931641, 241.15310669, 0., 0., 0., 1., 0.], 
                                                binning_x=0, 
                                                binning_y=0, 
                                                roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))

        themes = [
         {
         'topic':get_node_heirarchy(params)+'/color/camera_info',
         'msg_type':CameraInfo,
         'expected_data_chunks':1,
         'data':color_data
        },
        {
         'topic':get_node_heirarchy(params)+'/depth/camera_info',
         'msg_type':CameraInfo,
         'expected_data_chunks':1,
         'data':depth_data
        },
        {
         'topic':get_node_heirarchy(params)+'/infra1/camera_info',
         'msg_type':CameraInfo,
         'expected_data_chunks':1,
         'data':infra1_data
        },
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            ret = self.run_test(themes)
            assert ret[0], ret[1]
            assert self.process_data(themes), "Data check failed, probably the rosbag file changed?"
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)
    
