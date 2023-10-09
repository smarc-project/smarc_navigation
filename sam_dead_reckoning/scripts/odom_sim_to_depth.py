#!/usr/bin/env python3
"""
Extract depth from simulated odom message and publish to depth topic.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class Odom2Depth(object):
	"""
	Simple class to extract depth from simulated odom message.
	"""

	def __init__(self):
		"""
		Init function to set up ros interface
		"""
		self.odom_frame = rospy.get_param('~odom_frame', '/odom')
		self.odom_sim_topic = rospy.get_param('~odom_sim_topic', '/pressure')
		self.depth_topic = rospy.get_param('~depth_topic', '/depth')

		self.odom_sim_sub = rospy.Subscriber(self.odom_sim_topic, Odometry, self.odom_cb)
		self.depth_pub = rospy.Publisher(self.depth_topic, PoseWithCovarianceStamped, queue_size=10)

		self.depth_msg = PoseWithCovarianceStamped()
		self.depth_msg.header.frame_id = self.odom_frame
		self.depth_msg.pose.covariance = [100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1., 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

		self.depth_msg.pose.pose.orientation.w = 1.

		rospy.spin()


	def odom_cb(self, odom_msg):
		"""
		Simple callback to convert ground truth from simulation to 
		depth message.
		"""
		depth = -odom_msg.pose.pose.position.z
		self.depth_msg.header.stamp = rospy.Time.now()
		self.depth_msg.pose.pose.position.z = depth

		self.depth_pub.publish(self.depth_msg)


if __name__ == "__main__":

	rospy.init_node('odom_to_depth')
	try:
		Odom2Depth()
	except rospy.ROSInterruptException:
		pass
