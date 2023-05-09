#!/usr/bin/env python

import rospy
import numpy as np
import tf
from sbg_driver.msg import SbgEkfEuler
from std_msgs.msg import Float64


class Yaw2Heading(object):

	def __init__(self):
		self.sbg_topic = rospy.get_param('/sam/sbg/ekf_euler', '/sam/sbg/ekf_euler')
		self.heading_topic = rospy.get_param('/sam/dr/heading', '/sam/dr/heading')

		self.subs = rospy.Subscriber(self.sbg_topic, SbgEkfEuler, self.headingCB)
		self.pub = rospy.Publisher(self.heading_topic, Float64, queue_size=10)

		rospy.spin()


	def headingCB(self, sbg_msg):
		heading_msg = Float64()
		heading_msg.data = sbg_msg.angle.z

		self.pub.publish(heading_msg)



if __name__ == "__main__":

	rospy.init_node('yaw_2_heading')
	try:
		pi = Yaw2Heading()
	except rospy.ROSInterruptException:
		pass
