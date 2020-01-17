#!/usr/bin/python

import rospy
import numpy as np
#from sam_msgs.msg import ThrusterRPMs
from geometry_msgs.msg import TwistStamped
from uavcan_ros_bridge.msg import ESCStatus

class SamMM(object):

	def __init__(self):
            self.dr_thrust_topic = rospy.get_param(rospy.get_name() + '/thrust_dr', '/motion_dr')
            self.rpm_fb_topic = rospy.get_param(rospy.get_name() + '/thrust_fb', '/rpm_fb')
            self.subs_thrust = rospy.Subscriber(self.rpm_fb_topic, ESCStatus, self.thrustCB)	    
            self.control_pub = rospy.Publisher(self.dr_thrust_topic, TwistStamped, queue_size=10)
            
            self.prev_time = rospy.Time.now()
            self.coeff = 0.001
            self.first_it = True
            self.rpm_0 = 0.0
            self.rpm_1 = 0.0
            self.rpm_t = 0.

            rospy.spin()

 	def thrustCB(self, esc_msg):
            if esc_msg.esc_index == 0:
		self.rpm_0 = esc_msg.rpm
            if esc_msg.esc_index == 1:
		self.rpm_1 = esc_msg.rpm

            twist_msg = TwistStamped()
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = "sam/base_link"
            twist_msg.twist.linear.x = (self.rpm_0 + self.rpm_1) * self.coeff
#            msg_odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
#                                         0.0, 1, 0.0, 0.0, 0.0, 0.0,
#                                         0.0, 0.0, 1, 0.0, 0.0, 0.0,
#                                         0.0, 0.0, 0.0, 1, 0.0, 0.0,
#                                         0.0, 0.0, 0.0, 0.0, 1, 0.0,
#                                         0.0, 0.0, 0.0, 0.0, 0.0, 1]

            self.control_pub.publish(twist_msg)


if __name__ == "__main__":

	rospy.init_node('sam_mm')
	try:
		SamMM()
	except rospy.ROSInterruptException:
		pass

