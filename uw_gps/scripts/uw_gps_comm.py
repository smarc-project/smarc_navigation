#!/usr/bin/env python3

import rospy
import numpy as np
import actionlib
from smarc_bt.msg import GotoWaypointActionGoal
from geometry_msgs.msg import PointStamped
from smarc_acomms_msgs.msg import UnderwaterGPS

import tf

class UWCommGPS(object):

    def wp_cb(self, wp_goal):

        self.nav_goal = wp_goal

        # Check if the goal has been reached
        goal_point = PointStamped()
        goal_point.header.frame_id = self.nav_goal.goal.waypoint.pose.header.frame_id
        goal_point.header.stamp = self.nav_goal.goal.waypoint.pose.header.stamp
        goal_point.point.x = self.nav_goal.goal.waypoint.pose.pose.position.x
        goal_point.point.y = self.nav_goal.goal.waypoint.pose.pose.position.y
        goal_point.point.z = self.nav_goal.goal.waypoint.pose.pose.position.z

        try:

            goal_utm = self.listener.transformPoint(self.utm_frame, goal_point)

            # Msg def
            # float64 easting
            # float64 northing
            # float64 altitude
            # float64 cov_xx
            # float64 cov_yy
            # float64 cov_xy
            # string frame
            # uint32 timestamp
            # uint32 topic_id
            # uint32 sender_id

            ucomm_gps = UnderwaterGPS()
            # ucomm_gps.frame = self.utm_frame
            ucomm_gps.timestamp = goal_utm.header.stamp
            
            ucomm_gps.easting = goal_utm.point.x
            ucomm_gps.northing = goal_utm.point.y
            ucomm_gps.altitude = 0.
            ucomm_gps.cov_xx = 1.
            ucomm_gps.cov_yy = 1.
            ucomm_gps.cov_xy = 0.

            self.uw_comm_out.publish(ucomm_gps)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("UW Comm GPS: Could not transform WP to utm")
                pass


    def __init__(self):

        self.utm_frame = rospy.get_param('~utm_frame', 'utm')
        self.listener = tf.TransformListener()
        
        goto_wp_as = rospy.get_param("~goto_wp_server_goal", "/goto_waypoint")
        uw_comm_out = rospy.get_param("~uw_comm_out_top", "")
        self.subs = rospy.Subscriber(goto_wp_as, GotoWaypointActionGoal, self.wp_cb)
        self.uw_comm_out = rospy.Publisher(uw_comm_out, UnderwaterGPS , queue_size=1)

        rospy.spin()

if __name__ == "__main__":

    rospy.init_node("uw_comm_gps")

    try:
        uw_gps = UWCommGPS()
    except rospy.ROSInterruptException:
        pass
