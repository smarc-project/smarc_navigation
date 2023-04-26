#!/usr/bin/env python3

"""
Get position from Water Linked Underwater GPS
"""
import argparse
import json
import requests
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf
import tf2_ros
from geodesy import utm
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped

class UWGPSNode():

    def get_data(self, url):
        try:
            r = requests.get(url)
        except requests.exceptions.RequestException as exc:
            print("Exception occured {}".format(exc))
            return None

        if r.status_code != requests.codes.ok:
            print("Got error {}: {}".format(r.status_code, r.text))
            return None

        return r.json()

    def get_antenna_position(self, base_url):
        return self.get_data("{}/api/v1/config/antenna".format(base_url))

    def get_acoustic_position(self, base_url):
        return self.get_data("{}/api/v1/position/acoustic/filtered".format(base_url))

    def get_global_position(self, base_url, acoustic_depth = None):
        return self.get_data("{}/api/v1/position/global".format(base_url))
    
    def get_master_position(self, base_url):
        return self.get_data("{}/api/v1/position/master".format(base_url))
    
    def get_master_imu(self, base_url):
        return self.get_data("{}/api/v1/imu/calibrate".format(base_url))
    
    def set_position_master(self, url, latitude, longitude, orientation):
        payload = dict(lat=latitude, lon=longitude, orientation=orientation)
        # Keep loop running even if for some reason there is no connection.
        try:
            requests.put(url, json=payload, timeout=1)
        except requests.exceptions.RequestException as err:
            print("Serial connection error: {}".format(err))

    # BC tf UTM --> master (NED)
    def wl_gps(self, master_gps):

        if master_gps.status.status != -1:

            utm_master = utm.fromLatLong(
                master_gps.latitude, master_gps.longitude)
            
            master_imu = self.get_master_imu(self.base_url)
            
            rospy.loginfo("UW GPS node: broadcasting transform %s to %s" % (self.utm_frame, self.master_frame))
            
            # Rotate to go from NED to ENU
            quat_ned = tf.transformations.quaternion_from_euler(
                master_imu["roll"], master_imu["pitch"], master_imu["yaw"], axes='sxyz')
            quat_transf = tf.transformations.quaternion_from_euler(
                np.pi, -np.pi/2., 0., axes='rxzy')
            quat_enu = quat_ned * quat_transf
            
            transformStamped = TransformStamped()
            transformStamped.transform.translation.x = utm_master.northing
            transformStamped.transform.translation.y = utm_master.easting
            transformStamped.transform.translation.z = 0.
            transformStamped.transform.rotation = Quaternion(*quat_enu)
            transformStamped.header.frame_id = self.utm_frame
            transformStamped.child_frame_id = self.master_frame
            transformStamped.header.stamp = rospy.Time.now()
            self.tf_bc.sendTransform(transformStamped)
            

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

        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        self.tf_bc = tf2_ros.TransformBroadcaster()
        self.master_pos_ext = None

        self.gps_global_pub = rospy.Publisher(self.uw_gps_latlon, NavSatFix, queue_size=10)
        self.uwgps_odom_pub = rospy.Publisher(self.uw_gps_odom, Odometry, queue_size=10)
        self.wl_gps_sub = rospy.Subscriber(self.external_gps, NavSatFix, self.wl_gps)


        print("Using base_url: %s" % self.base_url)

        while not rospy.is_shutdown():

            print("------------------------")

            # if self.master_pos_ext:
            #     self.set_position_master(
            #         '{}/api/v1/external/master'.format(self.base_url), self.master_pos_ext.latitude, self.master_pos_ext.longitde, -1)
                
                # Check that the master's position has been updated
                # master_position = self.get_master_position(self.base_url)
                # print("Current master position. Latitude: {}, Longitude: {}".format(
                #     master_position["lat"],
                #     master_position["lon"]))
            
            # antenna_position = None
            # if self.antenna:
            #     # Antenna position: x,y,z manually set wrt master cage
            #     antenna_position = self.get_antenna_position(self.base_url)
            # depth = None
            
            # Acoustic position: x,y,z wrt to master cage
            acoustic_position = self.get_acoustic_position(self.base_url)
            if acoustic_position:
                # if antenna_position:
                #     print("Current acoustic position relative to antenna. X: {}, Y: {}, Z: {}".format(
                #         acoustic_position["x"] - antenna_position["x"],
                #         acoustic_position["y"] - antenna_position["y"],
                #         acoustic_position["z"] - antenna_position["depth"]))
                # else:
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

            else:
                rospy.logwarn("UW GPS: relative position not received")
            
            # Locator global position (lat/lon)
            global_position = self.get_global_position(self.base_url)
            depth = acoustic_position["z"]
            print("Current global position. Latitude: {}, Longitude: {}, Depth: {}".format(
                global_position["lat"],
                global_position["lon"],
                depth))
            
            if global_position:
                t_now = rospy.Time.now()

                gps_msg = NavSatFix()
                gps_msg.header.frame_id = self.uwgps_frame
                gps_msg.header.stamp = t_now
                gps_msg.status.status = 0
                gps_msg.latitude = global_position["lat"]
                gps_msg.longitude = global_position["lon"]
                gps_msg.altitude = depth
                self.gps_global_pub.publish(gps_msg)

            else:
                rospy.logwarn("UW GPS: global position not received")
            
            # else:
            #     rospy.logwarn("UW GPS node: No external GPS fix received yet")

            rospy.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node("uw_gps_node")
    try:
        UWGPSNode()
    except rospy.ROSInterruptException:
        pass
