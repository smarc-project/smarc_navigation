#!/usr/bin/env python3

import rospy
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped, PointStamped
from geodesy import utm
from sensor_msgs.msg import NavSatFix, Imu
# from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import PointStamped, TransformStamped, Quaternion, PoseWithCovarianceStamped

import tf
import tf2_ros

from uwgps_interface import UWGPSInterface

class UWGPSStation:

    def gps_cb(self, gps_msg):
         
        if gps_msg.status.status != -1:

            gps_utm = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude)
            self.gps_msgs.append(gps_utm)

            if self.init_heading:
                rospy.loginfo("Station node: broadcasting transform %s to %s" % (self.utm_frame, self.gps_frame))            

                easting_avg = np.sum([msg.easting for msg in self.gps_msgs])/len(self.gps_msgs)                
                northing_avg = np.sum([msg.northing for msg in self.gps_msgs])/len(self.gps_msgs)                
                
                # euler = euler_from_quaternion([self.init_quat.x, self.init_quat.y, self.init_quat.z, self.init_quat.w])
                # quat = quaternion_from_euler(euler) # -0.3 for feb_24 with floatsam
                               
                transformStamped = TransformStamped()
                transformStamped.transform.translation.x = easting_avg
                transformStamped.transform.translation.y = northing_avg
                transformStamped.transform.translation.z = 0.
                transformStamped.transform.rotation = Quaternion(*self.sbg_quat)
                transformStamped.header.frame_id = self.utm_frame
                transformStamped.child_frame_id = self.gps_frame
                transformStamped.header.stamp = rospy.Time.now()
                self.br.sendTransform(transformStamped)
       
        else:

            rospy.logwarn("Station GPS msg invalid")


    def sbg_cb(self, sbg_msg):

        # self.init_quat = sbg_msg.orientation
        # self.init_heading = True

        if not self.map2base:

            transform_stamped = TransformStamped()
            transform_stamped.transform.translation.x = 0.
            transform_stamped.transform.translation.y = 0.
            transform_stamped.transform.translation.z = 0.
            transform_stamped.transform.rotation = Quaternion(
                *sbg_msg.orientation)
            transform_stamped.header.frame_id = self.map_frame
            transform_stamped.child_frame_id = self.base_frame
            transform_stamped.header.stamp = rospy.Time.now()
            self.static_tf_bc.sendTransform(transform_stamped)
            self.map2base = True


    def __init__(self):

        self.base_frame = rospy.get_param('~base_frame', "base_link")
        self.map_frame = rospy.get_param('~map_frame', "map")
        self.gps_frame = rospy.get_param('~gps_frame', "gps_link")
        self.utm_frame = rospy.get_param('~utm_frame', "utm")
        uwgps_frame = rospy.get_param('~uwgps_frame', 'uwgps_link')

        self.base_url = rospy.get_param('~uwgps_server_ip', "https://demo.waterlinked.com")
        self.node_freq = rospy.get_param('~node_freq', 1.)

        payload_gps = rospy.get_param('~station_gps', '/sam/external/uw_gps_latlon')
        self.gps_msgs = []
        self.wl_gps_sub = rospy.Subscriber(payload_gps, NavSatFix, self.gps_cb)

        # self.init_heading = False
        # self.init_m2o = False
        self.sbg_topic = rospy.get_param('~sbg_topic', '/sam/core/imu')
        self.sbg_sub = rospy.Subscriber(self.sbg_topic, Imu, self.sbg_cb,  queue_size=10)
        
        uwgps_topic = rospy.get_param('~uwgps_topic', '/station/uwgps')
        self.point_pub = rospy.Publisher(uwgps_topic, PointStamped, queue_size=10)
        
        self.uwgps_int = UWGPSInterface()
        self.listener = tf.TransformListener()
        goal_point_prev = PointStamped()
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        self.br = tf.TransformBroadcaster()
        self.map2base = False

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

                goal_point = PointStamped()
                goal_point.header.frame_id = uwgps_frame
                goal_point.header.stamp = rospy.Time(0)
                goal_point.point.x = acoustic_position["x"] 
                goal_point.point.y = acoustic_position["y"] 
                goal_point.point.z = acoustic_position["z"] 

                if goal_point.point != goal_point_prev.point:
                    goal_point_prev = goal_point

                    try:
                        goal_base = self.listener.transformPoint(self.map_frame, goal_point)
                        # For visualization
                        self.point_pub.publish(goal_base)
                        
                        print("Goal in command station map frame")
                        print(goal_base.point)

                    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        print("UWGPS module: Could not transform UGWPS WP to base_link")
                        pass

            r.sleep()


if __name__ == "__main__":

    rospy.init_node("station_uwgps_node")

    try:
        uw_gps = UWGPSStation()
    except rospy.ROSInterruptException:
        pass
