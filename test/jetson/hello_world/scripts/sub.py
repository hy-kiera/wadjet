#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def print_hello_world():
    rospy.init_node('hello_world_sub', anonymous=False)
    rospy.Subscriber("/hello_world/msg", String, callback)
    rospy.spin()

def callback(received_msg):
    print("Received msg : %s", received_msg)

if __name__ == '__main__':
    print_hello_world()
