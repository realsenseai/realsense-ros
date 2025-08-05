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

import rclpy
from rclpy.node import Node
import sys
import geometry_msgs.msg

import termios
import tty
import os
import math
import json
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    return


def print_status(status):
    sys.stdout.write('%-8s%-8s%-8s%-40s\r' % (status['mode'], status[status['mode']]['value'], status[status['mode']]['step'], status['message']))


def publish_status(node, broadcaster, status):
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = node.get_clock().now().to_msg()
    static_transformStamped.header.frame_id = from_cam

    static_transformStamped.child_frame_id = to_cam
    static_transformStamped.transform.translation.x = status['x']['value']
    static_transformStamped.transform.translation.y = status['y']['value']
    static_transformStamped.transform.translation.z = status['z']['value']

    quat = quaternion_from_euler(math.radians(status['roll']['value']),
                                                    math.radians(status['pitch']['value']),
                                                    math.radians(status['azimuth']['value']))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    broadcaster.sendTransform(static_transformStamped)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print ('USAGE:')
        print ('set_cams_transforms.py from_cam to_cam x y z azimuth pitch roll')
        print ('x, y, z: in meters')
        print ('azimuth, pitch, roll: in degrees')
        print ('If parameters are not given read last used parameters.')
        print ('[OPTIONS]')
        print ('--file <file name> : if given, default values are loaded from file')
        sys.exit(-1)

    from_cam, to_cam = sys.argv[1:3]
    try:
        filename = sys.argv[sys.argv.index('--file')+1]
        print ('Using file %s' % os.path.abspath(filename))
    except:
        filename = os.path.join(os.path.dirname(__file__), '_set_cams_info_file.txt')
        print ('Using default file %s' % os.path.abspath(filename))

    if len(sys.argv) >= 9:
        x, y, z, yaw, pitch, roll = [float(arg) for arg in sys.argv[3:10]]
        status = {'mode': 'pitch',
                  'x': {'value': x, 'step': 0.1},
                  'y': {'value': y, 'step': 0.1},
                  'z': {'value': z, 'step': 0.1},
                  'azimuth': {'value': yaw, 'step': 1},
                  'pitch': {'value': pitch, 'step': 1},
                  'roll': {'value': roll, 'step': 1},
                  'message': ''}
        print ('Use given initial values.')
    else:
        try:
            status = json.load(open(filename, 'r'))
            print ('Read initial values from file.')
        except IOError as e:
            print ('Failed reading initial parameters from file %s' % filename)
            print ('Initial parameters must be given for initial run or if an un-initialized file has been given.')
            sys.exit(-1)

    rclpy.init()
    node = Node('my_static_tf2_broadcaster')
    #rospy.init_node('my_static_tf2_broadcaster')
    broadcaster = StaticTransformBroadcaster(node)

    print
    print ('Press the following keys to change mode: x, y, z, (a)zimuth, (p)itch, (r)oll')
    print ('For each mode, press 6 to increase by step and 4 to decrease')
    print ('Press + to multiply step by 2 or - to divide')
    print ('Press Q to quit')

    status_keys = [key[0] for key in status.keys()]
    print ('%-8s%-8s%-8s%s' % ('Mode', 'value', 'step', 'message'))
    print_status(status)
    publish_status(node, broadcaster, status)
    while True:
        kk = getch()
        status['message'] = ''
        try:
            key_idx = status_keys.index(kk)
            status['mode'] = list(status.keys())[key_idx]
        except ValueError as e:
            if kk.upper() == 'Q':
                sys.stdout.write('\n')
                exit(0)
            elif kk == '4':
                status[status['mode']]['value'] -= status[status['mode']]['step']
            elif kk == '6':
                status[status['mode']]['value'] += status[status['mode']]['step']
            elif kk == '-':
                status[status['mode']]['step'] /= 2.0
            elif kk == '+':
                status[status['mode']]['step'] *= 2.0
            else:
                status['message'] = 'Invalid key:' + kk

        print_status(status)
        publish_status(node, broadcaster, status)
        json.dump(status, open(filename, 'w'), indent=4)

    #rospy.spin()
