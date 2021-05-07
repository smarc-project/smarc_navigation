#!/usr/bin/env python

import rospy
import numpy as np
#  from sam_msgs.msg import ThrusterRPMs
from smarc_msgs.msg import DualThrusterFeedback, ThrusterFeedback
from geometry_msgs.msg import TwistStamped

class SamACC(object):

    def __init__(self):
        
        self.dr_thrust_topic = rospy.get_param('~thrust_acc', '/sam/dr/motion_dr')
        self.rpm1_fb_topic = rospy.get_param('~thrust1_fb', '/sam/core/thruster1_fb')
        self.rpm2_fb_topic = rospy.get_param('~thrust2_fb', '/sam/core/thruster2_fb')
        #self.imu_topic = rospy.get_param('~sbg_imu', '/sam/core/sbg_imu')
        self.base_frame = rospy.get_param('~base_frame', 'sam/base_link')
        #self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')

        self.sub1_thrust = rospy.Subscriber(self.rpm1_fb_topic, ThrusterFeedback, self.thrust1CB)
        self.sub0_thrust = rospy.Subscriber(self.rpm2_fb_topic, ThrusterFeedback, self.thrust0CB)

        self.control_pub = rospy.Publisher(self.dr_thrust_topic, TwistStamped, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCB)

        self.coeff = 0.0005
        self.rpm_0 = 0.
        self.rpm_1 = 0.
        
        rospy.spin()

    def thrust1CB(self, rpm_msg):
        self.rpm_1 = rpm_msg.rpm.rpm

    def thrust0CB(self, rpm_msg):
        self.rpm_0 = rpm_msg.rpm.rpm

    def timerCB(self, ev):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = self.base_frame
        twist_msg.twist.linear.x = (self.rpm_0 + self.rpm_1) * self.coeff
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        self.control_pub.publish(twist_msg)


if __name__ == "__main__":

    rospy.init_node('sam_am')
    try:
        SamACC()
    except rospy.ROSInterruptException:
        pass
