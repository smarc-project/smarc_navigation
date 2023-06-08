#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf
import tf2_ros
from geodesy import utm
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped

from uwgps_interface import UWGPSInterface
import time

class UWGPSNode():

    # BC tf UTM --> master (NED)
    def wl_gps(self, master_gps):

        if master_gps.status.status != -1:

            utm_master = utm.fromLatLong(
                master_gps.latitude, master_gps.longitude)
            
            # try:
            #     (world_trans, world_rot) = self.listener.lookupTransform(self.utm_frame,
            #                                                              "map",
            #                                                              rospy.Time(0))

            # except (tf.LookupException, tf.ConnectivityException):
            #     rospy.loginfo("Aux DR: broadcasting transform %s to %s" % (self.utm_frame, "map"))

            #     utm_sam = utm.fromLatLong(master_gps.latitude, master_gps.longitude)
                
            #     transformStamped = TransformStamped()
            #     quat = tf.transformations.quaternion_from_euler(np.pi, -np.pi/2., 0., axes='rxzy')
            #     transformStamped.transform.translation.x = utm_sam.northing
            #     transformStamped.transform.translation.y = utm_sam.easting
            #     transformStamped.transform.translation.z = 0.
            #     transformStamped.transform.rotation = Quaternion(*quat)
            #     transformStamped.header.frame_id = self.utm_frame
            #     transformStamped.child_frame_id = "map"
            #     transformStamped.header.stamp = rospy.Time.now()
            #     self.static_tf_bc.sendTransform(transformStamped)
            
            master_imu = self.uwgps_int.get_master_imu(self.base_url)
            quat_transf = tf.transformations.quaternion_from_euler(
                np.pi, -np.pi/2., 0., axes='rxzy')
            # if master_imu:
            print("Master imu roll {}, pitch {}, yaw {}".format(master_imu["roll"], master_imu["pitch"], master_imu["yaw"]))
            euler_rad = np.deg2rad([master_imu["roll"], master_imu["pitch"], master_imu["yaw"]])
            euler_rad = [(angle + np.pi) % (2 * np.pi) - np.pi for angle in  euler_rad]
            # Rotate to go from NED to ENU
            quat_ned = tf.transformations.quaternion_from_euler(
                0.,0.,euler_rad[2], axes='sxyz')
                # euler_rad[0],euler_rad[1],euler_rad[2], axes='sxyz')
            quat_enu = quat_ned * quat_transf
            mag = np.linalg.norm(quat_enu)
            quat_enu /= mag
        
            quat_b = [0.,0.,0.,1.]
            transformStamped = TransformStamped()
            transformStamped.header.stamp = rospy.Time.now()
            transformStamped.header.frame_id = self.utm_frame
            transformStamped.child_frame_id = self.master_frame
            transformStamped.transform.translation.x = utm_master.northing
            transformStamped.transform.translation.y = utm_master.easting
            transformStamped.transform.translation.z = 0.
            transformStamped.transform.rotation = Quaternion(*quat_transf)
            self.tf_bc.sendTransform(transformStamped)
            
            rospy.loginfo("UW GPS node: broadcasting transform %s to %s" % (self.utm_frame, self.master_frame))

        else:
            rospy.logwarn("WL GPS status -1")
            

    def __init__(self):

        # TODO: use uwgps_link instead of base_link
        self.utm_frame = rospy.get_param('~utm_frame', 'utm')
        self.uwgps_frame = rospy.get_param('~uwgps_frame', 'sam/uwgps_link')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.master_frame = rospy.get_param('~master_frame', 'master_link')
        self.antenna = rospy.get_param('~bool_antenna', False)
        self.uw_gps_odom = rospy.get_param('~uw_gps_odom', '/sam/external/uw_gps_odom')
        self.uw_gps_latlon = rospy.get_param('~uw_gps_latlon', '/sam/external/uw_gps_latlon')
        self.external_gps = rospy.get_param('~wl_gps', '/sam/external/uw_gps_latlon')
        # self.base_url = rospy.get_param('~uwgps_server_ip', "http://192.168.2.94")
        self.base_url = rospy.get_param('~uwgps_server_ip', "https://demo.waterlinked.com")

        self.uwgps_int = UWGPSInterface()

        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        self.tf_bc = tf2_ros.TransformBroadcaster()
        self.master_pos_ext = None
        self.listener = tf.TransformListener()

        self.gps_global_pub = rospy.Publisher(self.uw_gps_latlon, NavSatFix, queue_size=10)
        self.uwgps_odom_pub = rospy.Publisher(self.uw_gps_odom, Odometry, queue_size=10)
        self.wl_gps_sub = rospy.Subscriber(self.external_gps, NavSatFix, self.wl_gps)


        print("Using base_url: %s" % self.base_url)

        while not rospy.is_shutdown():

            print("------------------------")
           
            # Acoustic position: x,y,z wrt to master cage
            acoustic_position = self.uwgps_int.get_acoustic_position(
                self.base_url)
            if acoustic_position:
                print("Current acoustic position. X: {}, Y: {}, Z: {}".format(
                    acoustic_position["x"],
                    acoustic_position["y"],
                    acoustic_position["z"]))
                depth = acoustic_position["z"]

                # UW GPS 
                t_now = rospy.Time.now()
                rot = [0., 0., 0., 1.]
                               
                odom_msg = Odometry()
                odom_msg.header.stamp = t_now
                odom_msg.header.frame_id = self.master_frame
                odom_msg.child_frame_id = self.uwgps_frame
                odom_msg.pose.covariance = [0.] * 36
                odom_msg.pose.pose.position.x = acoustic_position["y"]
                odom_msg.pose.pose.position.y = acoustic_position["x"]
                odom_msg.pose.pose.position.z = -acoustic_position["z"]
                odom_msg.pose.pose.orientation = Quaternion(*rot)
                self.uwgps_odom_pub.publish(odom_msg)

                transformStamped = TransformStamped()
                transformStamped.header.stamp = t_now
                transformStamped.header.frame_id = self.master_frame
                transformStamped.child_frame_id = self.uwgps_frame
                transformStamped.transform.translation.x = acoustic_position["y"]
                transformStamped.transform.translation.y = acoustic_position["x"]
                transformStamped.transform.translation.z = -acoustic_position["z"]
                transformStamped.transform.rotation = Quaternion(*rot)
                self.tf_bc.sendTransform(transformStamped)

            else:
                rospy.logwarn("UW GPS: relative position not received")
            
            # Locator global position (lat/lon)
            global_position = self.uwgps_int.get_global_position(self.base_url)
            if global_position:

                print("Current global position. Latitude: {}, Longitude: {}".format(
                    global_position["lat"],
                    global_position["lon"]))
                
                t_now = rospy.Time.now()

                gps_msg = NavSatFix()
                gps_msg.header.frame_id = self.uwgps_frame
                gps_msg.header.stamp = t_now
                gps_msg.status.status = 0
                gps_msg.latitude = global_position["lat"]
                gps_msg.longitude = global_position["lon"]
                gps_msg.altitude = 0.
                self.gps_global_pub.publish(gps_msg)

            else:
                rospy.logwarn("UW GPS: global position not received")
            
            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("uw_gps_node")
    try:
        UWGPSNode()
    except rospy.ROSInterruptException:
        pass
