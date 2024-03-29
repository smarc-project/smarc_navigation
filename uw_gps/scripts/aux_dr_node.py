#!/usr/bin/python3

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, Quaternion, PoseWithCovarianceStamped
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolRequest
# from sbg_driver.msg import SbgEkfEuler
from geodesy import utm


class ExternalDR(object):

    def __init__(self):

        self.stim_topic = rospy.get_param('~imu', '/sam/core/imu')
        self.sbg_topic = rospy.get_param('~sbg_topic', '/sam/core/imu')
        self.base_frame = rospy.get_param('~base_frame', 'sam/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.utm_frame = rospy.get_param('~utm_frame', 'utm')
        self.dr_period = rospy.get_param('~dr_period', 0.02)
        self.uwgps_frame = rospy.get_param('~uwgps_frame', 'uwgps_frame')
        self.odom_top = rospy.get_param('~odom_topic', '/sam/dr/dvl_dr')
        self.uw_gps_odom = rospy.get_param('~uw_gps_odom_topic', '/sam/core/gps')
        self.sam_gps_topic = rospy.get_param('~sam_gps', 'uw_gps_latlon')

        self.listener = tf.TransformListener()
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        self.br = tf.TransformBroadcaster()
        self.transformStamped = TransformStamped()

        self.init_heading = False
        self.init_m2o = False

        # Stim integration
        self.rot_t = [0.] * 3
        self.t_stim_prev = 0.
        self.init_stim = False
        self.vel_rot = [0.] * 3
        self.t_now = 0.

        # Connect
        self.pub_odom = rospy.Publisher(self.odom_top, Odometry, queue_size=100)
        self.sbg_sub = rospy.Subscriber(self.sbg_topic, Imu, self.sbg_cb)
        self.stim_sub = rospy.Subscriber(self.stim_topic, Imu, self.stim_cb)
        self.uw_gps_odom_sub = rospy.Subscriber(self.uw_gps_odom, Odometry, self.uw_gps_odom_cb)
        self.sam_gps_sub = rospy.Subscriber(self.sam_gps_topic, NavSatFix, self.sam_gps_cb)

        self.uwgps_odom = None

        rospy.Timer(rospy.Duration(self.dr_period), self.dr_timer)

        rospy.spin()

    # BC tf UTM --> map and map --> odom once
    def sam_gps_cb(self, sam_gps):

        if sam_gps.status.status != -1:

            try:
                (world_trans, world_rot) = self.listener.lookupTransform(self.utm_frame,
                                                                         self.map_frame,
                                                                         rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException):
                rospy.loginfo("Aux DR: broadcasting transform %s to %s" % (self.utm_frame, self.map_frame))

                utm_sam = utm.fromLatLong(sam_gps.latitude, sam_gps.longitude)
                
                transformStamped = TransformStamped()
                quat = tf.transformations.quaternion_from_euler(np.pi, -np.pi/2., 0., axes='rxzy')
                transformStamped.transform.translation.x = utm_sam.northing
                transformStamped.transform.translation.y = utm_sam.easting
                transformStamped.transform.translation.z = 0.
                transformStamped.transform.rotation = Quaternion(*quat)
                transformStamped.header.frame_id = self.utm_frame
                transformStamped.child_frame_id = self.map_frame
                transformStamped.header.stamp = rospy.Time.now()
                self.static_tf_bc.sendTransform(transformStamped)

            try:
                (world_trans, world_rot) = self.listener.lookupTransform(self.map_frame,
                                                                         self.odom_frame,
                                                                         rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               
                if self.init_heading:
                    rospy.loginfo("Aux DR node: broadcasting transform %s to %s" % (
                        self.map_frame, self.odom_frame))

                    euler = euler_from_quaternion(
                        [self.init_quat.x, self.init_quat.y, self.init_quat.z, self.init_quat.w])
                    quat = quaternion_from_euler(0., 0., euler[2])

                    self.transformStamped.transform.translation.x = 0.
                    self.transformStamped.transform.translation.y = 0.
                    self.transformStamped.transform.translation.z = 0.
                    self.transformStamped.transform.rotation = Quaternion(*quat)
                    self.transformStamped.header.frame_id = self.map_frame
                    self.transformStamped.child_frame_id = self.odom_frame
                    self.transformStamped.header.stamp = rospy.Time.now()
                    self.static_tf_bc.sendTransform(self.transformStamped)
                    self.init_m2o = True

                    self.sam_gps_sub.unregister()


    # BC tf map to odom
    def uw_gps_odom_cb(self, gps_odom_msg):

        if self.init_m2o and self.init_stim:

            try:
                
                uwgps_rel = PointStamped()
                uwgps_rel.header.frame_id = "master_link"
                uwgps_rel.header.stamp = rospy.Time(0)
                uwgps_rel.point.x = gps_odom_msg.pose.pose.position.x
                uwgps_rel.point.y = gps_odom_msg.pose.pose.position.y
                uwgps_rel.point.z = gps_odom_msg.pose.pose.position.z
                
                self.uwgps_odom = self.listener.transformPoint(
                    self.odom_frame, uwgps_rel)

                # rospy.loginfo(
                #     "Aux DR: UW GPS to odom successful")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn(
                    "Aux DR: Transform master to uwgps not available yet")


    def dr_timer(self, event):

        # If tf map to odom exists:
        if self.uwgps_odom:

            quat_t = tf.transformations.quaternion_from_euler(
                self.rot_t[0], self.rot_t[1], self.rot_t[2])
            odom_msg = Odometry()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.child_frame_id = "sam_test_wl"
            odom_msg.pose.pose.position.x = self.uwgps_odom.point.x
            odom_msg.pose.pose.position.y = self.uwgps_odom.point.y
            odom_msg.pose.pose.position.z = self.uwgps_odom.point.z
            # odom_msg.twist.twist.linear.x = lin_vel_t[0]
            # odom_msg.twist.twist.linear.y = lin_vel_t[1]
            # odom_msg.twist.twist.linear.z = lin_vel_t[2]
            # odom_msg.twist.twist.angular.x = rot_vel_t[0]
            # odom_msg.twist.twist.angular.y = rot_vel_t[1]
            # odom_msg.twist.twist.angular.z = rot_vel_t[2]
            odom_msg.pose.covariance = [0.] * 36
            odom_msg.pose.pose.orientation = Quaternion(*quat_t)
            self.pub_odom.publish(odom_msg)

            self.br.sendTransform([self.uwgps_odom.point.x, self.uwgps_odom.point.y, self.uwgps_odom.point.z],
                                quat_t,
                                rospy.Time.now(),
                                "sam_test_wl",
                                self.odom_frame)


    def sbg_cb(self, sbg_msg):
        self.init_quat = sbg_msg.orientation
        self.init_heading = True
        # self.euler_sub.unregister()

    def fullRotation(self, roll, pitch, yaw):
        rot_z = np.array([[np.cos(yaw), -np.sin(yaw), 0.0],
                          [np.sin(yaw), np.cos(yaw), 0.0],
                          [0., 0., 1]])
        rot_y = np.array([[np.cos(pitch), 0.0, np.sin(pitch)],
                          [0., 1., 0.],
                          [-np.sin(pitch), np.cos(pitch), 0.0]])
        rot_x = np.array([[1., 0., 0.],
                          [0., np.cos(roll), -np.sin(roll)],
                          [0., np.sin(roll), np.cos(roll)]])

        return np.matmul(rot_z, np.matmul(rot_y, rot_x))

    def stim_cb(self, imu_msg):
        if self.init_stim and self.init_m2o:
            self.rot_stim = np.array([imu_msg.orientation.x,
                                      imu_msg.orientation.y,
                                      imu_msg.orientation.z,
                                      imu_msg.orientation.w])
            euler_t = tf.transformations.euler_from_quaternion(self.rot_stim)

            # Integrate yaw velocities
            self.vel_rot = np.array([imu_msg.angular_velocity.x,
                                     imu_msg.angular_velocity.y,
                                     imu_msg.angular_velocity.z])

            dt = imu_msg.header.stamp.to_sec() - self.t_stim_prev
            self.rot_t = np.array(self.rot_t) + self.vel_rot * dt
            self.t_stim_prev = imu_msg.header.stamp.to_sec()

            for rot in self.rot_t:
                rot = (rot + np.pi) % (2 * np.pi) - np.pi

            # Measure roll and pitch directly
            self.rot_t[0] = euler_t[0]
            self.rot_t[1] = euler_t[1]

        else:
            # rospy.loginfo("Stim data coming in")
            self.t_stim_prev = imu_msg.header.stamp.to_sec()
            self.init_stim = True


if __name__ == "__main__":
    rospy.init_node('aux_dr_node')
    try:
        ExternalDR()
    except rospy.ROSInterruptException:
        pass
