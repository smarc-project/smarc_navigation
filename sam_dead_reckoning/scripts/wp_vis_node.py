#!/usr/bin/python

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf
from geodesy import utm
import numpy as np
import tf2_ros
from smarc_msgs.msg import GotoWaypoint
import message_filters

class PublishWPsVis(object):

    def __init__(self, name):

        self.wp_topic = rospy.get_param('~wp_latest_topic', '/sam/smarc_bt/last_wp')
        self.utm_frame = rospy.get_param('~utm_frame', 'utm')

        # GPS odom in UTM frame
        self.gps_sam_sub = rospy.Subscriber(self.wp_topic, GotoWaypoint, self.wp_vis_cb)
        
        # Broadcast UTM to map frame
        self.listener = tf.TransformListener()        
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        


    def wp_vis_cb(self, wp_msg):

        if wp_msg.name != '':
            
            try:
                (world_trans, world_rot) = self.listener.lookupTransform(self.utm_frame, 
                                                                        wp_msg.name,
                                                                        rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException):

                utm_wp = utm.fromLatLong(wp_msg.lat, wp_msg.lon)
                rot = [0., 0., 0., 1.]

                rospy.loginfo("WP vis node: broadcasting transform %s to %s" % (self.utm_frame, wp_msg.name))            
                transformStamped = TransformStamped()
                transformStamped.transform.translation.x = utm_wp.easting
                transformStamped.transform.translation.y = utm_wp.northing
                transformStamped.transform.translation.z = 0.
                transformStamped.transform.rotation = Quaternion(*rot)               
                transformStamped.header.frame_id = self.utm_frame
                transformStamped.child_frame_id = wp_msg.name
                transformStamped.header.stamp = rospy.Time.now()
                self.static_tf_bc.sendTransform(transformStamped)

                return
   

if __name__ == "__main__":

    rospy.init_node('wp_vis_node', anonymous=False) #True)

    check_server = PublishWPsVis(rospy.get_name())

    rospy.spin()
