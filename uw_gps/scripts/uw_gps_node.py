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

    def __init__(self):

        # TODO: use uwgps_link instead of base_link
        self.utm_frame = rospy.get_param('~utm_frame', 'utm')
        self.uwgps_frame = rospy.get_param('~uwgps_frame', 'sam/uwgps_link')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.antenna = rospy.get_param('~bool_antenna', True)
        self.uw_gps_odom = rospy.get_param('~uw_gps_odom', '/sam/external/uw_gps_odom')
        self.uw_gps_latlon = rospy.get_param('~uw_gps_latlon', '/sam/external/uw_gps_latlon')
        self.base_url = rospy.get_param('~uwgps_server_ip', "http://192.168.2.94")
        #self.base_url = rospy.get_param('~uwgps_server_ip', "https://demo.waterlinked.com")

        self.gps_global_pub = rospy.Publisher(self.uw_gps_latlon, NavSatFix, queue_size=10)
        self.uwgps_odom_pub = rospy.Publisher(self.uw_gps_odom, Odometry, queue_size=10)

        print("Using base_url: %s" % self.base_url)

        while not rospy.is_shutdown():

            acoustic_position = self.get_acoustic_position(self.base_url)
            antenna_position = None
            if self.antenna:
                antenna_position = self.get_antenna_position(self.base_url)
            depth = None
            if acoustic_position:
                if antenna_position:
                    print(acoustic_position)
                    print(antenna_position)
                    print("Current acoustic position relative to antenna. X: {}, Y: {}, Z: {}".format(
                        acoustic_position["x"] - antenna_position["x"],
                        acoustic_position["y"] - antenna_position["y"],
                        acoustic_position["z"] - antenna_position["depth"]))
                else:
                    print("Current acoustic position. X: {}, Y: {}, Z: {}".format(
                        acoustic_position["x"],
                        acoustic_position["y"],
                        acoustic_position["z"]))
                depth = acoustic_position["z"]

            # Locator global position
            global_position = self.get_global_position(self.base_url)
            if global_position:
                print("Current global position. Latitude: {}, Longitude: {}, Depth: {}".format(
                    global_position["lat"],
                    global_position["lon"],
                    depth))
                
                utm_uwgps = utm.fromLatLong(
                    global_position["lat"], global_position["lon"])

                # UW GPS 
                t_now = rospy.Time.now()
                rot = [0., 0., 0., 1.]
                odom_msg = Odometry()
                odom_msg.header.stamp = t_now
                odom_msg.header.frame_id = self.utm_frame
                odom_msg.child_frame_id = self.base_frame
                odom_msg.pose.covariance = [0.] * 36
                odom_msg.pose.pose.position.x = utm_uwgps.northing
                odom_msg.pose.pose.position.y = utm_uwgps.easting
                odom_msg.pose.pose.position.z = depth
                odom_msg.pose.pose.orientation = Quaternion(*rot)
                self.uwgps_odom_pub.publish(odom_msg)

                gps_msg = NavSatFix()
                gps_msg.header.frame_id = self.base_frame
                gps_msg.header.stamp = t_now
                gps_msg.status.status = 0
                gps_msg.latitude = global_position["lat"]
                gps_msg.longitude = global_position["lon"]
                gps_msg.altitude = -depth
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
