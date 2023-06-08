#!/usr/bin/env python3

import rospy
import numpy as np
import actionlib
from smarc_bt.msg import GotoWaypointActionFeedback, GotoWaypointResult, GotoWaypointAction, GotoWaypointGoal

from uwgps_interface import UWGPSInterface

class UWGPSPayload(object):

    def shutdown_node(self):

        self.wp_ac.cancel_goal()
        if self.wp_ac.wait_for_result(rospy.Duration(2.)):
            rospy.loginfo("UW GPS goals preempted. Shutting down")
        else:
            rospy.logwarn("UW GPS goals did not preempt")


    def __init__(self):

        uwgps_frame = rospy.get_param('~uwgps_frame', 'uwgps_link')
        self.base_url = rospy.get_param('~uwgps_server_ip', "https://demo.waterlinked.com")
        self.rpm_chase = rospy.get_param('~rpm_chase', 500)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 1.)
        self.node_freq = rospy.get_param('~node_freq', 1.)
        
        self.uwgps_int = UWGPSInterface()
        rospy.on_shutdown(self.shutdown_node)

        goto_wp_as = rospy.get_param("~goto_wp_server", "/goto_waypoint")
        self.wp_ac = actionlib.SimpleActionClient(goto_wp_as, GotoWaypointAction)
        while not self.wp_ac.wait_for_server(timeout=rospy.Duration(5)) and not rospy.is_shutdown():
            rospy.loginfo("UWGPS waiting for %s", goto_wp_as)


        r = rospy.Rate(self.node_freq)
        while not rospy.is_shutdown():

            print("------------------------")

            # Acoustic position: x,y,z wrt to antenna
            acoustic_position = self.uwgps_int.get_acoustic_position(self.base_url)

            if acoustic_position:
                print("Current acoustic position. X: {}, Y: {}, Z: {}".format(
                    acoustic_position["x"],
                    acoustic_position["y"],
                    acoustic_position["z"]))

                wp = GotoWaypointGoal()
                wp.waypoint.travel_rpm = self.rpm_chase
                wp.waypoint.speed_control_mode = 1 # RPM control
                wp.waypoint.goal_tolerance = self.goal_tolerance
                wp.waypoint.pose.header.stamp = rospy.Time.now()
                wp.waypoint.pose.header.frame_id = uwgps_frame
                wp.waypoint.pose.pose.position.x = acoustic_position["x"]
                wp.waypoint.pose.pose.position.y = acoustic_position["y"]
                wp.waypoint.pose.pose.position.z = 0.

                # We can send goals continuously and they'll be preempted automatically
                self.wp_ac.send_goal(wp)

            r.sleep()


if __name__ == "__main__":

    rospy.init_node("payload_uwgps_node")

    try:
        uw_gps = UWGPSPayload()
    except rospy.ROSInterruptException:
        pass