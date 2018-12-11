#!/usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped

class Press2Depth(object):

	def __init__(self):
	
		self.subs = rospy.Subscriber("/uavcan_pressure2", FluidPressure, self.depthCB)
		self.pub = rospy.Publisher("/depth_dummy", PoseWithCovarianceStamped)



	def depthCB(self, press_msg):

		msg = PoseWithCovarianceStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "sam_odom"
		msg.pose.covariance = [1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
		msg.pose.pose.orientation.w = 1. # = [0., 0., 0., 1.]
		msg.pose.pose.position.z = - self.pascal_pressure_to_depth(press_msg.fluid_pressure) # = [0., 0., 2.]
		self.pub.publish(msg)

	def pascal_pressure_to_depth(self, pressure):

		return 10.*((pressure / 100000.) - 1.) # 117000 -> 1.7



if __name__ == "__main__":

	rospy.init_node('press_to_depth')
	#try:
	pi = Press2Depth()
	rospy.spin()
	#except rospy.ROSInterruptException:
	#	pass