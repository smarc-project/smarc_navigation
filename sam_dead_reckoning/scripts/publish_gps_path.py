#!/usr/bin/python

import rospy
import numpy as np
import math
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, Quaternion, Point
import tf_conversions
import tf
from geodesy import utm
from nav_msgs.msg import Odometry

class GPSOdomPublisher(object):

    def __init__(self):
        self.gps_topic = rospy.get_param(rospy.get_name() + '/gps_topic', '/fix')
        self.imu_topic = rospy.get_param(rospy.get_name() + '/imu_topic', '/heading_imu')

        self.odom_topic = "sam/utm_odom"
        self.odom_frame = "world_utm"
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        #self.odom_msg.pose.covariance = [.5, 0., 0., 0., .5, 0., 0., 0., .5]
        #self.odom_msg.pose.pose.orientation.w = 1.
        self.odom_msg.pose.pose.orientation = Quaternion(*[0., 0., 0., 1.])

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)

        rospy.spin()

    def imu_callback(self, imu_msg):

        print "Got IMU callback!"
        self.odom_msg.pose.pose.orientation = imu_msg.orientation

    def gps_callback(self, gps_msg):
        if gps_msg.status == -1:
            return
        try:
            utm_point = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude)
        except ValueError:
            return
        print utm_point.gridZone()
        easting = utm_point.easting
        northing = utm_point.northing

        self.odom_msg.pose.pose.position = Point(*[easting, northing, 0.])

        self.odom_pub.publish(self.odom_msg)

        self.listener.waitForTransform("sam_odom", "sam/base_link", rospy.Time(), rospy.Duration(4.0))
        try:
            now = rospy.Time(0)
            (odom_trans, odom_rot) = self.listener.lookupTransform("sam_odom", "sam/base_link", now)
            #(odom_trans, odom_rot) = self.listener.lookupTransform("sam/base_link", "sam_odom", now)
            odom_transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(odom_trans), tf.transformations.quaternion_matrix(odom_rot))
        except (tf.LookupException, tf.ConnectivityException):
            print "Could not get transform between ", "sam_odom", "and", "sam/base_link"
            return

        try:
            now = rospy.Time(0)
            (world_trans, world_rot) = self.listener.lookupTransform("world_utm", "world_local", now)
            #(world_trans, world_rot) = self.listener.lookupTransform("world_local", "world_utm", now)
            world_transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(world_trans), tf.transformations.quaternion_matrix(world_rot))
        except (tf.LookupException, tf.ConnectivityException):
            print "Could not get transform between ", "world_utm", "and", "world_local"
            return

        gps_trans = [easting, northing, 0.]
        gps_rot = [self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w]
        gps_transform = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(gps_trans), tf.transformations.quaternion_matrix(gps_rot))
        

        print "Got pose: ", odom_trans, odom_rot
        print "Got odom matrix: ", odom_transform
        print "Got world matrix: ", world_transform
        print "Got gps matrix: ", gps_transform

        correction = np.dot(np.dot(tf.transformations.inverse_matrix(world_transform), gps_transform), tf.transformations.inverse_matrix(odom_transform))
        print "got correction", correction

        correction_trans = tf.transformations.translation_from_matrix(correction)
        correction_rot = tf.transformations.quaternion_from_matrix(correction)

        self.br.sendTransform(correction_trans,
                     correction_rot,
                     rospy.Time.now(),
                     "world",
                     "world_local")

        print "Published transform!"


if __name__ == "__main__":

    rospy.init_node('spoof_gps_imu')
    try:
        node = GPSOdomPublisher()
    except rospy.ROSInterruptException:
        pass
