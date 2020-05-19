#!/usr/bin/python

"""
Create tracklog from Water Linked Underwater GPS
"""
from __future__ import print_function
import requests
import argparse
import time
import logging
import datetime
import gpxpy
import gpxpy.gpx
import rospy
import utm 
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class UWGps():

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

    def get_acoustic_position(self, base_url):
        return self.get_data("{}/api/v1/position/acoustic/filtered".format(base_url))


    def get_global_position(self, base_url):
        return self.get_data("{}/api/v1/position/global".format(base_url))

    def get_master_position(self, base_url):
        return self.get_data("{}/api/v1/position/master".format(base_url))


    def __init__(self):
        log = logging.getLogger()
        logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO)

        gps_global_pub = rospy.Publisher('/sam/dr/uw_gps_global', NavSatFix, queue_size=10)
        odom_pub = rospy.Publisher('/sam/dr/uw_gps_odom', Odometry, queue_size=10)
        gps_master_pub = rospy.Publisher('/sam/dr/uw_gps_master', NavSatFix, queue_size=10)
        pose_pub = rospy.Publisher('/sam/dr/uw_gps_pose', PoseStamped, queue_size=10)
        
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument('-u', '--url', help='Base URL to use', type=str, default='http://demo.waterlinked.com')
        parser.add_argument('-o', '--output', help='Output filename', type=str, default='tracklog.gpx')

        args = parser.parse_args()

        base_url = args.url
        filename = args.output
        log.info("Creating tracklog from: {} into file {}. Press Ctrl-C to stop logging".format(base_url, filename))

        gpx = gpxpy.gpx.GPX()

        # Create track
        gpx_track = gpxpy.gpx.GPXTrack()
        gpx.tracks.append(gpx_track)

        # Create segment
        gpx_segment_master = gpxpy.gpx.GPXTrackSegment()
        gpx_segment_global = gpxpy.gpx.GPXTrackSegment()

        gpx_track.segments.append(gpx_segment_master)
        gpx_track.segments.append(gpx_segment_global)

        # Open file for writing so we don't get an access denied error
        # after a full log session is completed
        f = open(filename, "w")

        try:
            while True:
                pos_global = self.get_global_position(base_url)
                if not pos_global:
                    log.warning("Got no global position")
                    continue

                lat_global = pos_global["lat"]
                lon_global = pos_global["lon"]

                pos_master = self.get_master_position(base_url)
                if not pos_master:
                    log.warning("Got no master position")
                    continue

                lat_master = pos_master["lat"]
                lon_master = pos_master["lon"]

                acoustic = self.get_acoustic_position(base_url)
                if not acoustic:
                    log.warning("Got no acoustic position")
                    continue

                depth = acoustic["z"]
                altitude = -depth

                log.info("Global: Lat: {} Lon: {} Alt: {}".format(lat_global, lon_global, altitude))
                gpx_segment_global.points.append(gpxpy.gpx.GPXTrackPoint(lat_global, lon_global, elevation=-altitude, time=datetime.datetime.utcnow()))

                log.info("Master: Lat: {} Lon: {}".format(lat_master, lon_master))
                gpx_segment_master.points.append(gpxpy.gpx.GPXTrackPoint(lat_master, lon_master, time=datetime.datetime.utcnow()))

                gps_msg = NavSatFix()
                gps_msg.header.frame_id = 'world'
                gps_msg.header.stamp = rospy.Time.now()
                gps_msg.latitude = lat_global
                gps_msg.longitude = lon_global
                gps_msg.altitude = altitude 
                gps_global_pub.publish(gps_msg)

                gps_msg.header.frame_id = 'uw_master'
                gps_msg.header.stamp = rospy.Time.now()
                gps_msg.latitude = lat_master 
                gps_msg.longitude = lon_master 
                gps_msg.altitude = 0.0 
                gps_master_pub.publish(gps_msg)

                pose_msg = PoseStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.pose.position.x = lat_global
                pose_msg.pose.position.y = lon_global
                pose_msg.pose.position.z = -depth 

                pose_pub.publish(pose_msg)

                #  (utm_x, utm_y, utm_zone, utm_letter) = utm.from_latlon(lat_global, lon_global)
                #  odom_msg = Odometry()
                #  odom_msg.header.frame_id = 'world'
                #  odom_msg.child_frame_id = 'base_link'
                #  odom_msg.pose.pose.position.x = lat_global
                #  odom_msg.pose.pose.position.y = lon_global
                #  odom_msg.pose.pose.position.z = depth

                #  odom_pub.publish(odom_msg)

                time.sleep(1)

        except KeyboardInterrupt:
            pass
    
        print("Saving data to file: {}".format(filename))
        f.write(gpx.to_xml())
        f.close()

if __name__ == "__main__":
    rospy.init_node('uw_gps_node')
    try:
        UWGps()
    except rospy.ROSInterruptException:
        pass
