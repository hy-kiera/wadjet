#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def send_hello_world():
    rospy.init_node('hello_world_pub_remote', anonymous=False)
    pub = rospy.Publisher('/hello_world/msg_remote', String, queue_size=1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        send_msg = "hello_world! from remote PC"
        pub.publish(send_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_hello_world()
    except rospy.ROSInterruptException:
        pass
