#!/usr/bin/python

import rospy
import numpy as np
from sam_msgs.msg import ThrusterRPMs
from geometry_msgs.msg import TwistStamped
from uavcan_ros_bridge.msg import ESCStatus
from sbg_driver.msg import SbgEkfEuler
from nav_msgs.msg import Odometry
import message_filters
from sensor_msgs.msg import Imu
import tf

class SamMMDummy(object):

    def __init__(self):
        
        self.dr_thrust_topic = rospy.get_param('~thrust_dr', '/sam/dr/motion_dr')
        self.base_frame = rospy.get_param('~base_frame', 'sam/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')

        self.pub_odom = rospy.Publisher(self.dr_thrust_topic, Odometry, queue_size=10)

        while not rospy.is_shutdown():

            odom_msg = Odometry()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.child_frame_id = self.base_frame
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.covariance = [0.] * 36
            odom_msg.pose.covariance[0] = 1.
            odom_msg.pose.covariance[7] = 1.
            odom_msg.pose.covariance[16] = 40.
            odom_msg.pose.pose.orientation.w = 1.
            self.pub_odom.publish(odom_msg)
            rospy.sleep(0.02)


if __name__ == "__main__":

    rospy.init_node('sam_mm_dummy')
    try:
        SamMMDummy()
    except rospy.ROSInterruptException:
        pass
