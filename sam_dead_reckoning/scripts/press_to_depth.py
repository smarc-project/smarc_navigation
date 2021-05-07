#!/usr/bin/env python

import rospy
import numpy as np
import tf
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class Press2Depth(object):

	def __init__(self):
		self.odom_frame = rospy.get_param(rospy.get_name() + '/odom_frame', '/odom')
		self.base_frame = rospy.get_param(rospy.get_name() + '/base_frame', '/base_link')
		self.depth_frame = rospy.get_param(rospy.get_name() + '/depth_frame', '/depth_link')
		self.press_topic = rospy.get_param(rospy.get_name() + '/pressure_topic', '/pressure')
		self.press_frame = rospy.get_param(rospy.get_name() + '/pressure_frame', '/pressure_link')
		self.depth_topic = rospy.get_param(rospy.get_name() + '/depth_topic', '/depth')

		self.subs = rospy.Subscriber(self.press_topic, FluidPressure, self.depthCB)
		self.pub = rospy.Publisher(self.depth_topic, PoseWithCovarianceStamped, queue_size=10)

		self.depth_msg = PoseWithCovarianceStamped()
		self.depth_msg.header.frame_id = self.odom_frame
		self.depth_msg.pose.covariance = [100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1., 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

		self.depth_msg.pose.pose.orientation.w = 1.
		
		self.listener_odom = tf.TransformListener()
		self.listener_press = tf.TransformListener()
		self.x_base_depth = 0.580

		# try:
		# 	(trans,quaternion) = self.listener_press.lookupTransform(self.base_frame, self.depth_frame, rospy.Time(10))

		# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# 	print('Could not get tf base to depth.')

		rospy.spin()


	def depthCB(self, press_msg):
		try:
		
			# # depth_abs is positive, must be manually negated
			depth_abs = - self.pascal_pressure_to_depth(press_msg.fluid_pressure)
			# rospy.loginfo("Depth abs %s", depth_abs)
			# rospy.loginfo("Fluid press %s", press_msg.fluid_pressure)

			if press_msg.fluid_pressure > 90000. and press_msg.fluid_pressure < 500000.:
				self.depth_msg.header.stamp = rospy.Time.now()
				self.depth_msg.pose.pose.position.z = depth_abs # = [0., 0., 2.]
				self.pub.publish(self.depth_msg)
		
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logerr("Depth transform missing tf")

	def pascal_pressure_to_depth(self, pressure):

		return 10.*((pressure / 100000.) - 1.) # 117000 -> 1.7



if __name__ == "__main__":

	rospy.init_node('press_to_depth')
	try:
		pi = Press2Depth()
	except rospy.ROSInterruptException:
		pass
