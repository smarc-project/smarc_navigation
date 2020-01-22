#!/usr/bin/python

import rospy
from rospy import ROSException
from std_msgs.msg import Header, Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sam_msgs.msg import GetGPSFixAction, GetGPSFixFeedback, GetGPSFixResult
from sam_msgs.msg import PercentStamped 
from nav_msgs.msg import Odometry
import actionlib
import tf_conversions
import tf
from tf.transformations import quaternion_from_euler, quaternion_multiply
from geodesy import utm
import math
import numpy as np

class PublishGPSPose(object):

    def __init__(self, name):

        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('gps_odom', Odometry)
	self.pose_pub = rospy.Publisher('gps_pose', PoseWithCovarianceStamped, queue_size=10)

        self.listener = tf.TransformListener()

        self.first_gps = True

	rospy.Subscriber("/sam/core/gps", NavSatFix, self.gps_callback)

    def gps_callback(self, gps_msg):

        if gps_msg.status.status == -1:
            return

        try:
            now = rospy.Time(0)
            (world_trans, world_rot) = self.listener.lookupTransform("world_utm", "world_local", now)
        except (tf.LookupException, tf.ConnectivityException):
            self._feedback.status = "Could not get transform between %s and %s" % ("world_utm", "world_local")
            rospy.loginfo("Could not get transform between %s and %s" % ("world_utm", "world_local"))
            self._as.publish_feedback(self._feedback)
            
        # easting, northing is in world_utm coordinate system,
        # we need to transform it to world or world_local
        utm_point = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude)
        easting = utm_point.easting
        northing = utm_point.northing
        utm_zone = utm_point.zone

        pos = np.array([easting-world_trans[0], northing-world_trans[1], 1.])
        rot = [0., 0., 0., 1.]

        header = Header()
        header.stamp = gps_msg.header.stamp

        if self.first_gps:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = header
            pose_msg.header.frame_id = "world"
            pose_msg.pose.pose.position = Point(*pos.tolist())
            pose_msg.pose.pose.orientation = Quaternion(*rot)
            pose_msg.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pose_msg.pose.covariance[0] = 5.
            pose_msg.pose.covariance[7] = 5.
            pose_msg.pose.covariance[14] = .1
            pose_msg.pose.covariance[21] = .01
            pose_msg.pose.covariance[28] = .01
            pose_msg.pose.covariance[35] = .5
            self.initial_pose_pub.publish(pose_msg)
            self.first_gps = False

        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "sam/gps_link"
        odom_msg.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        odom_msg.pose.covariance[0] = 0.01
        odom_msg.pose.covariance[7] = 0.01
        odom_msg.pose.covariance[14] = 0.003
        odom_msg.pose.covariance[21] = 0.1 # 10.
        odom_msg.pose.covariance[28] = 0.1 # 10.
        odom_msg.pose.covariance[35] = .5
        #self.odom_msg.pose.pose.orientation.w = 1.
        odom_msg.pose.pose.position = Point(*pos.tolist())
        odom_msg.pose.pose.orientation = Quaternion(*rot)
        self.odom_pub.publish(odom_msg)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "world"
        pose_msg.pose.pose.position = Point(*pos.tolist())
        pose_msg.pose.pose.orientation = Quaternion(*rot)
        pose_msg.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose_msg.pose.covariance[0] = 100.
        pose_msg.pose.covariance[7] = 100.
        pose_msg.pose.covariance[14] = .003
        pose_msg.pose.covariance[21] = .1
        pose_msg.pose.covariance[28] = .1
        pose_msg.pose.covariance[35] = .5
        self.pose_pub.publish(pose_msg)

if __name__ == "__main__":

    rospy.init_node('publish_gps_pose', anonymous=False) #True)

    check_server = PublishGPSPose(rospy.get_name())

    rospy.spin()
