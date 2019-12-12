#!/usr/bin/python

import rospy
import numpy as np
import math
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, Quaternion
import tf_conversions
import tf
from geodesy import utm
from nav_msgs.msg import Odometry

class SpoofIMU(object):

    def __init__(self):
        self.imu_frame =  rospy.get_param(rospy.get_name() + '/imu_frame', 'sam/base_link')
        self.gps_topic = rospy.get_param(rospy.get_name() + '/gps_topic', '/fix')
        self.vel_topic = rospy.get_param(rospy.get_name() + '/vel_topic', '/vel')
        self.heading_imu_topic = rospy.get_param(rospy.get_name() + '/heading_imu_topic', '/heading_imu')
        self.odom_topic = '/odom'

        if True:
            self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback)
        else:
            self.vel_sub = rospy.Subscriber(self.vel_topic, TwistStamped, self.vel_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.pub = rospy.Publisher(self.heading_imu_topic, Imu)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.imu_frame
        self.imu_msg.orientation_covariance = [.5, 0., 0., 0., .5, 0., 0., 0., .5]
        #self.imu_msg.pose.pose.orientation.w = 1.

        self.window = 5
        self.positions = []
        self.vels = []
        self.roll = 0.
        self.pitch = 0.

        rospy.spin()

    def odom_callback(self, odom_msg):

        quaternion = (odom_msg.pose.pose.orientation.x,
                      odom_msg.pose.pose.orientation.y,
                      odom_msg.pose.pose.orientation.z,
                      odom_msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]

    def vel_callback(self, vel_msg):
        if math.isnan(vel_msg.twist.linear.x):
            self.vels = self.vels[1:]
            return
        if len(self.vels) >= self.window:
            self.vels = self.vels[1:] + [[vel_msg.twist.linear.x, vel_msg.twist.linear.y]]
        else:
            self.vels = self.vels + [[vel_msg.twist.linear.x, vel_msg.twist.linear.y]]

        print self.vels
        vels = np.array(self.vels)
        if vels.shape[0] < 2:
            return
        mean_vel = np.mean(vels, axis=0)
        mean_vel = 1./np.linalg.norm(mean_vel)*mean_vel

        yaw = math.pi/2.-math.atan2(mean_vel[0], mean_vel[1])

        print mean_vel
        print yaw
        self.imu_msg.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(self.roll, self.pitch, yaw))
        print self.imu_msg.orientation
        self.imu_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.imu_msg)

    def gps_callback(self, gps_msg):
        if gps_msg.status == -1:
            self.positions = self.positions[1:]
            return
        utm_point = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude)
        easting = utm_point.easting
        northing = utm_point.northing
        print "Easting, northing: ", easting, northing
        self.pub.publish(self.imu_msg)
        if len(self.positions) >= self.window:
            self.positions = self.positions[1:] + [[northing, easting]]
        else:
            self.positions = self.positions + [[northing, easting]]
        pos = np.array(self.positions)
        if pos.shape[0] < 2:
            return
        diffs = pos[1:, :] - pos[:-1, :]
        mean_diff = np.mean(diffs, axis=0)
        print diffs
        print mean_diff
        print self.positions
        yaw = math.atan2(mean_diff[0], mean_diff[1])
        print yaw
        roll = 0.
        pitch = 0.
        self.imu_msg.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(self.roll, self.pitch, yaw))
        print self.imu_msg.orientation
        self.imu_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.imu_msg)
 

if __name__ == "__main__":

    rospy.init_node('spoof_gps_imu')
    try:
        pi = SpoofIMU()
    except rospy.ROSInterruptException:
        pass
