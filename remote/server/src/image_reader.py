#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image


class ImageReader:

    def __init__(self):
        self.sub = rospy.Subscriber('/jetson/video_capture', Image, self.show_image)
        self.cvb = CvBridge()

    def show_image(self, image):
        try:
            cv_image = self.cvb.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Image Window", cv_image)


if __name__ == '__main__':
    ir = ImageReader()
    rospy.init_node('image_reader', anonymous=False)
    
    try:
        rospy.spin()
        # send_hello_world()
    except rospy.ROSInterruptException:
        pass
