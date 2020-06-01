#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import LaserScan

class PixhawkController:
    def __init__(self):
        self.sub_lidar = rospy.Subscriber("jetson/rplidar_node", LaserScan, self.listen)
        self.sub_face_point = rospy.Subscriber('jetson/face_point', Int64MultiArray, self.listen_face_point)
    
    def listen(self, lidar_data):
        #print(lidar_data.ranges)
        time.sleep(1)
    
    def listen_face_point(self, face_points):
        print("Pixhawk : ", face_points)


if __name__=="__main__":
    rospy.init_node("PixhawkController", anonymous=False)
    l = PixhawkController()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
