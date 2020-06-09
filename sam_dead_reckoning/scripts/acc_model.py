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

class SamACC(object):

    def __init__(self):
        
        self.dr_thrust_topic = rospy.get_param('~thrust_acc', '/sam/dr/motion_dr')
        self.rpm_fb_topic = rospy.get_param('~thrust_fb', '/sam/core/rpm_fb')
        self.imu_topic = rospy.get_param('~sbg_imu', '/sam/core/sbg_imu')
        self.base_frame = rospy.get_param('~base_frame', 'sam/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')

        self.subs_thrust = rospy.Subscriber(self.rpm_fb_topic, ThrusterRPMs, self.thrustCB)

        self.control_pub = rospy.Publisher(self.dr_thrust_topic, TwistStamped, queue_size=10)

        self.coeff = 0.0005
        
        rospy.spin()

    def thrustCB(self, rpm_msg):

        rpm_0 = rpm_msg.thruster_1_rpm
        rpm_1 = rpm_msg.thruster_2_rpm

        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = self.base_frame
        twist_msg.twist.linear.x = (rpm_0 + rpm_1) * self.coeff
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        self.control_pub.publish(twist_msg)


if __name__ == "__main__":

    rospy.init_node('sam_am')
    try:
        SamACC()
    except rospy.ROSInterruptException:
        pass
