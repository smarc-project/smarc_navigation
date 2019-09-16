#!/usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped

class SpoofDepth(object):

	def __init__(self):
		self.odom_frame = rospy.get_param(rospy.get_name() + '/odom_frame', '/odom')
		self.depth_topic = rospy.get_param(rospy.get_name() + '/depth_topic', '/depth')

		self.pub = rospy.Publisher(self.depth_topic, PoseWithCovarianceStamped)

		self.depth_msg = PoseWithCovarianceStamped()
		self.depth_msg.header.frame_id = self.odom_frame
		self.depth_msg.pose.covariance = [1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
		self.depth_msg.pose.pose.orientation.w = 1.
		
                while not rospy.is_shutdown():
                    self.depth_msg.header.stamp = rospy.Time.now()
                    self.depth_msg.pose.pose.position.z = 0.
                    self.pub.publish(self.depth_msg)
                    rospy.sleep(1./25.)

if __name__ == "__main__":

	rospy.init_node('spoof_depth')
	try:
		pi = SpoofDepth()
	except rospy.ROSInterruptException:
		pass
