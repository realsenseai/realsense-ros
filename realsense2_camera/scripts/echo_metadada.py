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

#!/usr/bin/env python
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy import qos
from realsense2_camera_msgs.msg import Metadata
import json

def metadata_cb(msg):
    aa = json.loads(msg.json_data)
    os.system('clear')
    print('header:\nstamp:\n  secs:', msg.header.stamp.sec, '\n  nsecs:',  msg.header.stamp.nanosec)
    print('\n'.join(['%10s:%-10s' % (key, str(value)) for key, value in aa.items()]))

def main():
    if len(sys.argv) < 2 or '--help' in sys.argv or '/?' in sys.argv:
        print ('USAGE:')
        print('echo_metadata.py <topic>')
        print('Demo for listening on given metadata topic.')
        print('App subscribes on given topic')
        print('App then prints metadata from messages')
        print('')
        print('Example: echo_metadata.py /camera/depth/metadata')
        print('')
        exit(-1)

    topic = sys.argv[1]

    rclpy.init()
    node = Node('metadata_tester')

    depth_sub = node.create_subscription(Metadata, topic, metadata_cb, qos.qos_profile_sensor_data)

    rclpy.spin(node)

if __name__ == '__main__':
    main()
