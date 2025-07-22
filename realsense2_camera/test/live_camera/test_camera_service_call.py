# Copyright 2024 Intel Corporation. All Rights Reserved.
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

import os, sys
import pytest

sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
import pytest_live_camera_utils
from pytest_rs_utils import launch_descr_with_parameters
from pytest_rs_utils import get_node_heirarchy

test_params_test_srv_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    }
test_params_test_srv_d435i = {
    'camera_name': 'D435i',
    'device_type': 'D435i',
    }
test_params_test_srv_d415 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    }

'''
The test checks service call device_info with type DeviceInfo
device info includes - device name, FW version, serial number, etc
'''
@pytest.mark.parametrize("launch_descr_with_parameters", [    
    pytest.param(test_params_test_srv_d455, marks=pytest.mark.d455),
    pytest.param(test_params_test_srv_d435i, marks=pytest.mark.d435i),
    pytest.param(test_params_test_srv_d415, marks=pytest.mark.d415),    
    ],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestCamera_ServiceCall_DeviceInfo(pytest_rs_utils.RsTestBaseClass):
    def test_camera_service_call_device_info(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]

        if pytest_live_camera_utils.check_if_camera_connected(params['device_type']) == False:
            print("Device not found? : " + params['device_type'])
            assert False
        
        try:
            print("Starting camera test...")
            self.init_test("RsTest"+params['camera_name'])
            self.wait_for_node(params['camera_name'])
            self.create_service_client_ifs(get_node_heirarchy(params))
            #No need to call run_test() as no frame integritiy check required
            response = self.get_deviceinfo()
            if response is not None:
                print(f"device_info service response:\n{response}\n")
                assert params['device_type'].casefold() in response.device_name.casefold().split('_')
            else:
                assert False, "No device_info response received"
        except Exception as e:
            print(e)
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()
            
