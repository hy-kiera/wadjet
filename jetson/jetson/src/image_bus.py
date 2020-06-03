#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import os
import time
import subprocess
import msgpack
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int64MultiArray
from sensor_msgs.msg import Image
from json_socket import Client

HOST_IP = "127.0.0.1"
HOST_PORT = 9879

class ImageBus:

    def __init__(self):
        self.cvb = CvBridge()
        self.rate = rospy.Rate(2)
        self.client = Client()
        self.client.connect(HOST_IP, HOST_PORT)
        self.sub_video_capture = rospy.Subscriber('/jetson/video_capture', Image, self.send_to_face_detector)
        self.pub_face_point = rospy.Publisher('/jetson/face_point', Int64MultiArray, queue_size=1)    
    def send_to_face_detector(self, image):
        try:
            cv_image = self.cvb.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
       
        print("\nSTART HERE!!\n")

        send_data = cv_image.tolist()

        self.client.send(send_data)
            
        recv_data = self.client.recv()
        recv_data = msgpack.unpackb(recv_data)

        print("recv_data [p1, p2] : ", recv_data)
        face_points = Int64MultiArray(data=recv_data[0]+recv_data[1])
        self.pub_face_point.publish(face_points)
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ImageBus', anonymous=False)
    filename = os.path.dirname(os.path.abspath(__file__)) + "/face_detector.py"

    p = subprocess.Popen(["python3", filename])
    rospy.sleep(15)
    ib = ImageBus()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    ib.client.close()
