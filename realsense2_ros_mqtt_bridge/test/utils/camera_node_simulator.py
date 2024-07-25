import numpy as np
from rclpy.node import Node
import rclpy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
import threading

''' 
This is that holds the test node that listens to a subscription created by a test.  
'''
class RSCameraSimulator(Node, threading.Thread):
    def __init__(self, namespace="camera", name='RSCameraSimulator'):
        print('\nCreating node... /' + namespace + '/' + name)
        queue = 1
        if not rclpy.ok():
            rclpy.init()
        Node.__init__(self,namespace=namespace, node_name=name)
        threading.Thread.__init__(self)
        self._stop_event = threading.Event()
        self.color_frame = self.create_publisher(Image, '/' + namespace + '/' + name + '/color/image_raw', queue)
        self.declare_all_parameters()
    def run(self):
        print("\nThread started...")
        loop_count = 0
        while(self._stop_event.is_set() == False):
            if loop_count > 10:
                print("\nSpinning...")
                loop_count = 0
            self.publish_frame()
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop(self):
        print("\nSetting the stop event...")
        self._stop_event.set()
    def publish_frame(self):
        msg = Image()
        frame = np.zeros((4,4,0), dtype=int)
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'test'
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding = "rgb"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] * np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()
        # publishes message
        #print("\nPublishing color frame...")
        self.color_frame.publish(msg)
    def declare_all_parameters(self):
        try:
            self.declare_parameter('safety_camera.safety_mode', 0)
            self.safety_camera_safety_mode = self.get_parameter(
                'safety_camera.safety_mode').get_parameter_value().integer_value
            print("safety mode is " + str(self.safety_camera_safety_mode))
        except Exception as e:
            print(f'An unexpected error occurred: {e}')
        self.add_on_set_parameters_callback(self.parameter_callback)
        pass
    def parameter_callback(self, params):
        print("Params changed:")
        print(params)
        for param in params:
            if param.name == 'safety_camera.safety_mode':
                if param.type_ == rclpy.Parameter.Type.INTEGER:
                    self.safety_camera_safety_mode = param.value
                    print("Safety mode changed to " + str(self.safety_camera_safety_mode))
                else:
                    return SetParametersResult(successful=False)                    
        return SetParametersResult(successful=True)


if __name__ == '__main__':
    rclpy.init()
    import os
    namespace = 'camera'
    name = 'camera'
    camera = RSCameraSimulator(namespace, name)
    camera.start()
    os.system("ros2 node list")
    import time
    rclpy.spin(node)
    time.sleep(10)
    rclpy.shutdown()
    camera.stop()
    camera.join()
    print("Test completed")
