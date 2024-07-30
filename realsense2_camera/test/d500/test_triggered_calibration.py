'''
from realsense2_camera_msgs.action import TriggeredCalibration
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
#from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
'''

'''
humble doesn't have the SetParametersResult and SetParameters_Response imported using 
__init__.py. The below two lines can be used for iron and hopefully succeeding ROS2 versions
'''


'''

#from rcl_interfaces.msg import SetParametersResult
#from rcl_interfaces.srv import SetParameters_Response
from rcl_interfaces.msg._set_parameters_result import SetParametersResult
from rcl_interfaces.srv._set_parameters  import SetParameters_Response

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
'''

import os
import sys
import logging
#LOGGER = logging.getLogger(__name__)
LOGGER = logging.getLogger()
import pytest


import rclpy
sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
from safety_camera_client import CameraClient

#@pytest.mark.skip(reason="under development")
def test_triggered_calibration():

    tester = CameraClient()
    try:
        tester.preapare_for_calibration()
        tester.send_goal()
        
        while tester.tc_done == False:
            rclpy.spin_once(tester)
            time.sleep(1)

        calib_result = tester.calibration_result

        assert calib_result.success == True, "First calibration run failed"
        time.sleep(0.5)

        tester.send_goal()
        while tester.tc_done == False:
            rclpy.spin_once(tester)
            time.sleep(1)
        assert tester.calibration_result.success == True, "Second calibration run failed"

        assert calib_result.calibration != tester.calibration_result.calibration, "Two calibrations gave the same data"

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        LOGGER.warning("Test failed")
        LOGGER.error(exc_type, fname, exc_tb.tb_lineno)
    finally:
        LOGGER.warning("Test completed")
        tester.destroy_node()
        rclpy.shutdown()

import time
if __name__ == '__main__':
    test_triggered_calibration()