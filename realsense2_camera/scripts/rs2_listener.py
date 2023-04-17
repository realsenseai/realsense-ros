import sys
import time
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CompressedImage as msg_CompressedImage
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Imu as msg_Imu
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import inspect
import ctypes
import struct
import tf
try:
    from theora_image_transport.msg import Packet as msg_theora
except Exception:
    pass


def pc2_to_xyzrgb(point):
	# Thanks to Panos for his code used in this function.
    x, y, z = point[:3]
    rgb = point[3]

    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f', rgb)
    i = struct.unpack('>l', s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
    return x, y, z, r, g, b


class CWaitForMessage:
    def __init__(self, params={}):
        self.result = None

        self.break_timeout = False
        self.timeout = params.get('timeout_secs', -1) * 1e-3
        self.seq = params.get('seq', -1)
        self.time = params.get('time', None)
        self.node_name = params.get('node_name', 'rs2_listener')
        self.bridge = CvBridge()
        self.listener = None
        self.prev_msg_time = 0
        self.fout = None
        

        self.themes = {'depthStream': {'topic': '/camera/depth/image_rect_raw', 'callback': self.imageColorCallback, 'msg_type': msg_Image},
                       'colorStream': {'topic': '/camera/color/image_raw', 'callback': self.imageColorCallback, 'msg_type': msg_Image},
                       'pointscloud': {'topic': '/camera/depth/color/points', 'callback': self.pointscloudCallback, 'msg_type': msg_PointCloud2},
                       'alignedDepthInfra1': {'topic': '/camera/aligned_depth_to_infra1/image_raw', 'callback': self.imageColorCallback, 'msg_type': msg_Image},
                       'alignedDepthColor': {'topic': '/camera/aligned_depth_to_color/image_raw', 'callback': self.imageColorCallback, 'msg_type': msg_Image},
                       'static_tf': {'topic': '/camera/color/image_raw', 'callback': self.imageColorCallback, 'msg_type': msg_Image},
                       'accelStream': {'topic': '/camera/accel/sample', 'callback': self.imuCallback, 'msg_type': msg_Imu},
                       }

        self.func_data = dict()

    def imuCallback(self, theme_name):
        def _imuCallback(data):
            if self.listener is None:
                self.listener = tf.TransformListener()
            self.prev_time = time.time()
            self.func_data[theme_name].setdefault('value', [])
            self.func_data[theme_name].setdefault('ros_value', [])
            try:
                frame_id = data.header.frame_id
                value = data.linear_acceleration

                (trans,rot) = self.listener.lookupTransform('/camera_link', frame_id, rospy.Time(0))
                quat = tf.transformations.quaternion_matrix(rot)
                point = np.matrix([value.x, value.y, value.z, 1], dtype='float32')
                point.resize((4, 1))
                rotated = quat*point
                rotated.resize(1,4)
                rotated = np.array(rotated)[0][:3]
            except Exception as e:
                print(e)
                return
            self.func_data[theme_name]['value'].append(value)
            self.func_data[theme_name]['ros_value'].append(rotated)
        return _imuCallback            

    def imageColorCallback(self, theme_name):
        def _imageColorCallback(data):
            self.prev_time = time.time()
            self.func_data[theme_name].setdefault('avg', [])
            self.func_data[theme_name].setdefault('ok_percent', [])
            self.func_data[theme_name].setdefault('num_channels', [])
            self.func_data[theme_name].setdefault('shape', [])
            self.func_data[theme_name].setdefault('reported_size', [])

            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            except CvBridgeError as e:
                print(e)
                return
            channels = cv_image.shape[2] if len(cv_image.shape) > 2 else 1
            pyimg = np.asarray(cv_image)

            ok_number = (pyimg != 0).sum()

            self.func_data[theme_name]['avg'].append(pyimg.sum() / ok_number)
            self.func_data[theme_name]['ok_percent'].append(float(ok_number) / (pyimg.shape[0] * pyimg.shape[1]) / channels)
            self.func_data[theme_name]['num_channels'].append(channels)
            self.func_data[theme_name]['shape'].append(cv_image.shape)
            self.func_data[theme_name]['reported_size'].append((data.width, data.height, data.step))
        return _imageColorCallback

    def imageDepthCallback(self, data):
        pass

    def pointscloudCallback(self, theme_name):
        def _pointscloudCallback(data):
            self.prev_time = time.time()
            print ('Got pointcloud: %d, %d' % (data.width, data.height))

            self.func_data[theme_name].setdefault('frame_counter', 0)
            self.func_data[theme_name].setdefault('avg', [])
            self.func_data[theme_name].setdefault('size', [])
            self.func_data[theme_name].setdefault('width', [])
            self.func_data[theme_name].setdefault('height', [])
            # until parsing pointcloud is done in real time, I'll use only the first frame.
            self.func_data[theme_name]['frame_counter'] += 1

            if self.func_data[theme_name]['frame_counter'] == 1:
                # Known issue - 1st pointcloud published has invalid texture. Skip 1st frame.
                return

            try:
                points = np.array([pc2_to_xyzrgb(pp) for pp in pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgb")) if pp[0] > 0])
            except Exception as e:
                print(e)
                return
            self.func_data[theme_name]['avg'].append(points.mean(0))
            self.func_data[theme_name]['size'].append(len(points))
            self.func_data[theme_name]['width'].append(data.width)
            self.func_data[theme_name]['height'].append(data.height)
        return _pointscloudCallback

    def wait_for_message(self, params, msg_type=msg_Image):
        topic = params['topic']
        print ('connect to ROS with name: %s' % self.node_name)
        rospy.init_node(self.node_name, anonymous=True)

        out_filename = params.get('filename', None)
        if (out_filename):
            self.fout = open(out_filename, 'w')
            if msg_type is msg_Imu:
                col_w = 20
                print ('Writing to file: %s' % out_filename)
                columns = ['frame_number', 'frame_time(sec)', 'accel.x', 'accel.y', 'accel.z', 'gyro.x', 'gyro.y', 'gyro.z']
                line = ('{:<%d}'*len(columns) % (col_w, col_w, col_w, col_w, col_w, col_w, col_w, col_w)).format(*columns) + '\n'
                sys.stdout.write(line)
                self.fout.write(line)

        rospy.loginfo('Subscribing on topic: %s' % topic)
        self.sub = rospy.Subscriber(topic, msg_type, self.callback)

        self.prev_time = time.time()
        break_timeout = False
        while not any([rospy.core.is_shutdown(), break_timeout, self.result]):
            rospy.rostime.wallsleep(0.5)
            if self.timeout > 0 and time.time() - self.prev_time > self.timeout:
                break_timeout = True
                self.sub.unregister()

        return self.result

    @staticmethod
    def unregister_all(registers):
        for test_name in registers:
            rospy.loginfo('Un-Subscribing test %s' % test_name)
            registers[test_name]['sub'].unregister()

    def wait_for_messages(self, themes):
        # tests_params = {<name>: {'callback', 'topic', 'msg_type', 'internal_params'}}
        self.func_data = dict([[theme_name, {}] for theme_name in themes])

        print ('connect to ROS with name: %s' % self.node_name)
        rospy.init_node(self.node_name, anonymous=True)
        for theme_name in themes:
            theme = self.themes[theme_name]
            rospy.loginfo('Subscribing %s on topic: %s' % (theme_name, theme['topic']))
            self.func_data[theme_name]['sub'] = rospy.Subscriber(theme['topic'], theme['msg_type'], theme['callback'](theme_name))

        self.prev_time = time.time()
        break_timeout = False
        while not any([rospy.core.is_shutdown(), break_timeout]):
            rospy.rostime.wallsleep(0.5)
            if self.timeout > 0 and time.time() - self.prev_time > self.timeout:
                break_timeout = True
                self.unregister_all(self.func_data)

        return self.func_data

    def callback(self, data):
        msg_time = data.header.stamp.secs + 1e-9 * data.header.stamp.nsecs

        if (self.prev_msg_time > msg_time):
            rospy.loginfo('Out of order: %.9f > %.9f' % (self.prev_msg_time, msg_time))
        if type(data) == msg_Imu:
            col_w = 20
            frame_number = data.header.seq
            accel = data.linear_acceleration
            gyro = data.angular_velocity
            line = ('\n{:<%d}{:<%d.6f}{:<%d.4f}{:<%d.4f}{:<%d.4f}{:<%d.4f}{:<%d.4f}{:<%d.4f}' % (col_w, col_w, col_w, col_w, col_w, col_w, col_w, col_w)).format(frame_number, msg_time, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z)
            sys.stdout.write(line)
            if self.fout:
                self.fout.write(line)

        self.prev_msg_time = msg_time
        self.prev_msg_data = data

        self.prev_time = time.time()
        if any([self.seq < 0 and self.time is None, 
                self.seq > 0 and data.header.seq >= self.seq,
                self.time and data.header.stamp.secs == self.time['secs'] and data.header.stamp.nsecs == self.time['nsecs']]):
            self.result = data
            self.sub.unregister()



def main():
    if len(sys.argv) < 2 or '--help' in sys.argv or '/?' in sys.argv:
        print ('USAGE:')
        print ('------')
        print ('rs2_listener.py <topic | theme> [Options]')
        print ('example: rs2_listener.py /camera/color/image_raw --time 1532423022.044515610 --timeout 3')
        print ('example: rs2_listener.py pointscloud')
        print ('')
        print ('Application subscribes on <topic>, wait for the first message matching [Options].')
        print ('When found, prints the timestamp.')
        print
        print ('[Options:]')
        print ('-s <sequential number>')
        print ('--time <secs.nsecs>')
        print ('--timeout <secs>')
        print ('--filename <filename> : write output to file')
        exit(-1)

    # wanted_topic = '/device_0/sensor_0/Depth_0/image/data'
    # wanted_seq = 58250

    wanted_topic = sys.argv[1]
    msg_params = {}
    if 'points' in wanted_topic:
        msg_type = msg_PointCloud2
    elif ('imu' in wanted_topic) or ('gyro' in wanted_topic) or ('accel' in wanted_topic):
        msg_type = msg_Imu
    elif 'theora' in wanted_topic:
        try:
            msg_type = msg_theora
        except NameError as e:
            print ('theora_image_transport is not installed. \nType "sudo apt-get install ros-kinetic-theora-image-transport" to enable registering on messages of type theora.')
            raise
    elif 'compressed' in wanted_topic:
        msg_type = msg_CompressedImage
    else:
        msg_type = msg_Image

    for idx in range(2, len(sys.argv)):
        if sys.argv[idx] == '-s':
            msg_params['seq'] = int(sys.argv[idx + 1])
        if sys.argv[idx] == '--time':
            msg_params['time'] = dict(zip(['secs', 'nsecs'], [int(part) for part in sys.argv[idx + 1].split('.')]))
        if sys.argv[idx] == '--timeout':
            msg_params['timeout_secs'] = int(sys.argv[idx + 1])
        if sys.argv[idx] == '--filename':
            msg_params['filename'] = sys.argv[idx+1]

    msg_retriever = CWaitForMessage(msg_params)
    if '/' in wanted_topic:
        msg_params.setdefault('topic', wanted_topic)
        res = msg_retriever.wait_for_message(msg_params, msg_type)
        rospy.loginfo('Got message: %s' % res.header)
        if (hasattr(res, 'encoding')):
            print ('res.encoding:', res.encoding)
        if (hasattr(res, 'format')):
            print ('res.format:', res.format)
    else:
        themes = [wanted_topic]
        res = msg_retriever.wait_for_messages(themes)
        print (res)


if __name__ == '__main__':
    main()

