import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))

from camera_node_simulator import RSCameraSimulator
from sds_simulator import SDSSimulator

#@pytest.mark.parametrize("launch_descr_with_yaml", [test_params],indirect=True)
def test_bridge_instatiation():
    namespace = 'camera'
    name = 'camera'
    camera = RSCameraSimulator(namespace, name)
    sds = SDSSimulator("localhost", 1883)
    sds.start_client()
    camera.start()
    os.system("ros2 node list")
    sds.enumerate_devices(namespace, name)

    param = sds.get_param(namespace,
        name,
        'safety_camera.safety_mode')
    print("safety_camera.safety_mode Param received: " + str(param))    

    sds.set_param(namespace,
            name,
            'safety_camera.safety_mode',
            '2',
            'int')
    param = sds.get_param(namespace,
        name,
        'safety_camera.safety_mode')
    print("safety_camera.safety_mode Param received: " + str(param))  
    frame = sds.get_frame(namespace, name, "color")
    print(frame)
    camera.stop()
    camera.join()
    print("Test completed")
