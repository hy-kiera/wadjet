#!/usr/bin/env python
import numpy as np
import rospy
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State, HomePosition
from std_msgs.msg import Header
from six.moves import xrange
from tf.transformations import quaternion_from_euler
import mavros
from mavros import command

class DroneController:
    
    def __init__(self):
        self.current_state = State()
        # self.local_position = PoseStamped()
        # self.pose = PoseStamped()

        self.state_sub = rospy.Subscriber("/mavros/state", State, self.set_state)
        # self.local_pos_sub = rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, self.set_local_pos)
        # self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
       
        # Services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service("/mavros/cmd/arming", service_timeout)
            rospy.wait_for_service("/mavros/set_mode", service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")

        self.set_arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        
        #self.wn = rospy.get_param("pub_setpoints_traj/wn", 1.0)
        #self.r = rospy.get_param("pub_setpoints_traj/r", 1.0)

        # Position
        #self.pose.pose.position.x = 0
        #self.pose.pose.position.y = 0
        #self.pose.pose.position.z = 2


    def set_mode(self, mode, timeout):
        rospy.loginfo("setting FCU mode: {0}".format(mode))

        rate = rospy.Rate(1)
            
        #for i in range(5):
        #    self.local_pos_pub.publish(self.pose)
        #    rate.sleep()

        # for i in xrange(timeout * loop_freq):
        while not rospy.is_shutdown():
            rospy.loginfo(self.current_state.mode)
            rospy.loginfo(mode)
            if self.current_state.mode == mode:
                rospy.loginfo("set mode success")
                break
            else:
                try:
                    res = self.set_mode_client(0, mode)  # 0 is custom mode
                    rospy.loginfo(res)
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                    #else:
                    #    self.current_state.mode = mode
                    #    rospy.loginfo("mode send success")
                        #break
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            rospy.sleep(5)

    def set_arm_g(self, arm, timeout):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            res = armService(True)
            if not res.success:
                rospy.logerr("failed to send arm command")
            else:
                rospy.loginfo("set arm success")
        except rospy.ServiceException, e:
            print "Service arm call failed: %s"%e

    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.current_state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        
        # for i in xrange(timeout * loop_freq):
        while not rospy.is_shutdown():
            if self.current_state.armed == arm:
                # rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                # i / loop_freq, timeout))
                rospy.loginfo("set arm success")
                break
            else:
                try:
                    res = self.set_arming_client(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                    else:
                        self.current_state.armed = arm
                        rospy.loginfo("set arm success")
                        break
                except rospy.ServiceException as e:
                    rospy.logerr(e)

        rate.sleep()


    def set_state(self, new_state):
        self.current_state = new_state

    #def set_local_pos(self, local_pos):
    #    self.local_position = local_pos

    #def is_at_position(self, x, y, z, offset):
    #    """offset: meters"""
    #    rospy.logdebug(
    #        "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
    #            self.local_position.pose.position.x, self.local_position.pose.
    #            position.y, self.local_position.pose.position.z))

    #    desired = np.array((x, y, z))
    #    pos = np.array((self.local_position.pose.position.x,
    #                    self.local_position.pose.position.y,
    #                    self.local_position.pose.position.z))

    #    return np.linalg.norm(desired - pos) < offset


    #def position_control(self, x, y, z):
    #    rate = rospy.Rate(10)
    #    self.pose.header = Header()
    #    self.pose.header.frame_id = "base_footprint"

    #    start_time = rospy.Time.now()

    #    first_reach = True
    #    first_reach_time = rospy.Time.now()
    #    self.pose.pose.position.x = x
    #    self.pose.pose.position.y = y 
    #    self.pose.pose.position.z = z

    #    yaw_degrees = 0  # North
    #    yaw = math.radians(yaw_degrees)
    #    quaternion = quaternion_from_euler(0, 0, yaw)
    #    self.pose.pose.orientation = Quaternion(*quaternion)

    #    while not rospy.is_shutdown():
    #        self.pose.header.stamp = rospy.Time.now()
    #        self.local_pos_pub.publish(self.pose)
    #    
    #        if self.is_at_position(x, y, z, self.r):
    #            if first_reach:
    #                first_reach = False
    #                first_reach_time = rospy.Time.now()

    #            end_time = rospy.Time.now()
    #            rospy.loginfo("Position Control Success!")
                # rospy.loginfo("Move StartTime : {}, EndTime : {}, Total : {}".format(start_time, end_time, end_time - start_time))

    #            if (end_time - start_time) > rospy.rostime.Duration(5):
    #                rospy.loginfo("Let's landing!")
    #                break
            
    #        else:
    #            rospy.loginfo(
    #                "Attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
    #            format(x, y, z, self.local_position.pose.position.x,
    #                    self.local_position.pose.position.y,
    #                    self.local_position.pose.position.z))

    #        rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("offb_node")
    dc = DroneController()
    try:
        dc.set_mode("OFFBOARD", 5)
        dc.set_arm(True, 5)

        #target_pos = (0, 0, 2)

        # Takeoff
        #rospy.loginfo("Takeoff start")
        # dc.set_mode("AUTO.TAKEOFF", 3)
        #dc.position_control(target_pos[0], target_pos[1], target_pos[2])

        #rospy.loginfo("Landing start")
        #dc.set_mode("AUTO.LAND", 5)
        #dc.position_control(0, 0, 0)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
