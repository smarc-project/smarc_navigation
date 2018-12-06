#!/usr/bin/python


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

rospy.init_node("odom_publisher")

rate = rospy.Rate(10)
#msg = Odometry()
#msg.header.frame_id = "/sam_auv_odom" 
#msg.child_frame_id = "/sam_auv/base_link"
#msg.pose.pose.position.z = -2. # = [0., 0., 2.]
#msg.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
#msg.pose.pose.orientation.w = 1. # = [0., 0., 0., 1.]

msg = PoseWithCovarianceStamped()
msg.header.frame_id = "sam_auv_odom" 
msg.pose.pose.position.z = -2. # = [0., 0., 2.]
msg.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
msg.pose.pose.orientation.w = 1. # = [0., 0., 0., 1.]

#pub = rospy.Publisher("/depth_dummy", Odometry)
pub = rospy.Publisher("/depth_dummy", PoseWithCovarianceStamped)

z = np.arange(-2, 2, 0.001)
print z
counter = 0
while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.z = z[counter] # = [0., 0., 2.]
    pub.publish(msg)
    rate.sleep()
    counter = counter + 1
