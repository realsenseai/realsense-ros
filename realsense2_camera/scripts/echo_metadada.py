#!/usr/bin/env python
import os
import sys
import rospy
from realsense2_camera.msg import Metadata
import json

def metadata_cb(msg):
    aa = json.loads(msg.json_data)
    os.system('clear')
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

    rospy.init_node('metadata_tester', anonymous=True)

    depth_sub = rospy.Subscriber(topic, Metadata, metadata_cb)

    rospy.spin()

if __name__ == '__main__':
    main()
