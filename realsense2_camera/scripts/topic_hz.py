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
import sensor_msgs.msg
from realsense2_camera_msgs.msg import RGBD
import sys
import time

class ImageListener(Node):
    def __init__(self, topic):
        self.topic = topic
        super().__init__('topic_hz')
        if 'points' in topic:
            self.sub = self.create_subscription(sensor_msgs.msg.PointCloud2, topic, self.imageDepthCallback, 1)
        elif 'image' in topic:
            self.sub = self.create_subscription(sensor_msgs.msg.Image, topic, self.imageDepthCallback, 1)
        elif 'rgb' in topic:
            self.sub = self.create_subscription(RGBD, topic, self.imageDepthCallback, 1)
        else:
            raise ('Unknown message type for topic ', topic)
        # self.sub # prevent unused variable warning
        self.message_times = []
        self.max_buffer_size = 100
        self.print_time = time.time()

    def imageDepthCallback(self, data):
        crnt_time = time.time()
        self.message_times.append(crnt_time)
        if len(self.message_times) > self.max_buffer_size:
            del self.message_times[0]
        if crnt_time - self.print_time > 1 and len(self.message_times) > 1:
            rate = len(self.message_times) / (self.message_times[-1] - self.message_times[0])
            print('Frame rate at time: %s: %.02f(Hz)' % (time.ctime(crnt_time), rate))
            # sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            # sys.stdout.flush()

def main():
    if len(sys.argv) < 2 or '-h' in sys.argv or '--help' in sys.argv:
        print ()
        print ('USAGE:')
        print ('------')
        print ('python3 ./topic_hz.py <topic>')
        print ('Application to act as ros2 topic hz : print the rate of which the messages on given topic arrive.')
        print ()
        sys.exit(0)

    topic = sys.argv[1]
    listener = ImageListener(topic)
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    main()
