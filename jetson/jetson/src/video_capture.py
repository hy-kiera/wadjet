#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import sys
import cv2
import gi
import numpy as np
from cv_bridge import CvBridge

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class Camera:

    def __init__(self):
        self.pub = rospy.Publisher("/jetson/video_capture", Image, queue_size=1)
        self.cvb = CvBridge()

    def read_cam(self):
        cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
        if cap.isOpened():
            # cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
            while not rospy.core.is_shutdown():
                ret_val, img = cap.read() # img type is numpy.ndarray

                # cv2.imshow('demo', img)

                # send image to pub node using cvbridge
                self.pub.publish(self.cvb.cv2_to_imgmsg(img, 'bgr8')) 
        else:
            print("Cap failed")

if __name__=="__main__":
    cam = Camera()
    rospy.init_node('camera')
    try:
        cam.read_cam()
        rospy.spin()
        outcome = 'Test Completed'
    except rospy.ROSInterruptException:
        print("Exception")
        pass
    rospy.core.signal_shutdown(outcome)


