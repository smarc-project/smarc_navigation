#!/usr/bin/python

import rospy
import numpy as np
from sam_msgs.msg import ThrusterRPMs
from nav_msgs.msg import Odometry

class SamDR(object):

	def __init__(self):
		
		self.dr_thrust_topic = rospy.get_param(rospy.get_name() + '/thrust_dr', '/motion_dr')
		self.rpm_fb_topic = rospy.get_param(rospy.get_name() + '/thrust_fb', '/rpm_fb')
		self.subs_thrust = rospy.Subscriber(self.rpm_fb_topic, ThrusterRPMs, self.thrustCB)	
 		self.odom_pub = rospy.Publisher(self.dr_thrust_topic, Odometry, queue_size=10)

 		self.prev_time = rospy.Time.now()
		self.coeff = 0.0001
 		self.first_it = True
 		self.rpm_1 = 0.0
 		self.rpm_2 = 0.0


 		rate = rospy.Rate(125.)
 		while not rospy.is_shutdown():

			msg_odom = Odometry()
			msg_odom.header.stamp = rospy.Time.now()
			msg_odom.header.frame_id = "sam_odom"
			msg_odom.child_frame_id = "sam/base_link"

			msg_odom.twist.twist.linear.x = (self.rpm_1 + self.rpm_2) * self.coeff 
	 		msg_odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
	 		0.0, 1, 0.0, 0.0, 0.0, 0.0,
	 		0.0, 0.0, 1, 0.0, 0.0, 0.0,
	 		0.0, 0.0, 0.0, 1, 0.0, 0.0,
	 		0.0, 0.0, 0.0, 0.0, 1, 0.0,
	 		0.0, 0.0, 0.0, 0.0, 0.0, 1]

			self.odom_pub.publish(msg_odom)	
			rate.sleep()

	def thrustCB(self, thrust_msg):
		self.rpm_1 = thrust_msg.thruster_1_rpm
		self.rpm_2 = thrust_msg.thruster_2_rpm


if __name__ == "__main__":

	rospy.init_node('sam_mm')
	try:
		SamDR()
	except rospy.ROSInterruptException:
		pass

