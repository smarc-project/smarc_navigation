#!/usr/bin/env python3

import rospy
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped, PointStamped
from geodesy import utm
from sensor_msgs.msg import NavSatFix, Imu
# from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import PointStamped, TransformStamped, Quaternion, PoseWithCovarianceStamped
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Bool
import tf
import tf2_ros

from uwgps_interface import UWGPSInterface

class UWGPSStation:

    def gps_cb(self, gps_msg):
         
        if gps_msg.status.status != -1:

            gps_utm = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude)
            self.gps_msgs.append(gps_utm)

            # if self.init_heading:
            rospy.loginfo_once("Station node: broadcasting transform %s to %s" % (self.utm_frame, self.map_frame))            

            easting_avg = np.sum([msg.easting for msg in self.gps_msgs])/len(self.gps_msgs)                
            northing_avg = np.sum([msg.northing for msg in self.gps_msgs])/len(self.gps_msgs)                
            rot = [0.,0.,0.,1.]
            # euler = euler_from_quaternion([self.init_quat.x, self.init_quat.y, self.init_quat.z, self.init_quat.w])
            # quat = quaternion_from_euler(euler) # -0.3 for feb_24 with floatsam
                            
            transformStamped = TransformStamped()
            transformStamped.transform.translation.x = easting_avg
            transformStamped.transform.translation.y = northing_avg
            transformStamped.transform.translation.z = 0.
            transformStamped.transform.rotation = Quaternion(*rot)
            transformStamped.header.frame_id = self.utm_frame
            transformStamped.child_frame_id = self.map_frame
            transformStamped.header.stamp = rospy.Time.now()
            self.static_tf_bc.sendTransform(transformStamped)

            # Republish to visualize in GUI
            gps_geo = GeoPoint()
            gps_geo.latitude = gps_msg.latitude
            gps_geo.longitude = gps_msg.longitude
            self.gps_geo_pub.publish(gps_geo)
       
        else:
            rospy.logwarn("Station GPS msg invalid")
            rospy.logwarn("")


    def sbg_cb(self, sbg_msg):

        rospy.loginfo_once("Station node: broadcasting transform %s to %s" % (self.map_frame, self.base_frame))            

        self.sbg_init = True
        self.sbg_t = rospy.get_time()
        
        transform_stamped = TransformStamped()
        transform_stamped.transform.translation.x = 0.
        transform_stamped.transform.translation.y = 0.
        transform_stamped.transform.translation.z = 0.
        transform_stamped.transform.rotation = sbg_msg.orientation
        transform_stamped.header.frame_id = self.map_frame
        transform_stamped.child_frame_id = self.base_frame
        transform_stamped.header.stamp = sbg_msg.header.stamp
        self.static_tf_bc.sendTransform(transform_stamped)

    def gui_cb(self, send_uwgps_msg):
        self.send_uwgps = True

    def __init__(self):

        self.base_frame = rospy.get_param('~base_frame', "base_link")
        self.map_frame = rospy.get_param('~map_frame', "map")
        self.gps_frame = rospy.get_param('~gps_frame', "gps_link")
        self.utm_frame = rospy.get_param('~utm_frame', "utm")
        uwgps_frame = rospy.get_param('~uwgps_frame', 'uwgps_link')

        self.base_url = rospy.get_param('~uwgps_server_ip', "https://demo.waterlinked.com")
        self.node_freq = rospy.get_param('~node_freq', 1.)

        self.gps_msgs = []
        gps_geo_top = rospy.get_param('~gps_geo_top', '/station/uwgps')
        self.gps_geo_pub = rospy.Publisher(gps_geo_top, GeoPoint, queue_size=100)
        
        payload_gps = rospy.get_param('~station_gps_top', '/sam/external/gps')
        self.wl_gps_sub = rospy.Subscriber(payload_gps, NavSatFix, self.gps_cb)

        # self.init_heading = False
        self.sbg_init = False
        self.sbg_t = 0.
        self.sbg_prev_t = 0.
        self.listener = tf.TransformListener()
        goal_point_prev = PointStamped()
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        self.br = tf.TransformBroadcaster()


        self.sbg_topic = rospy.get_param('~sbg_topic', '/sam/core/imu')
        self.sbg_sub = rospy.Subscriber(self.sbg_topic, Imu, self.sbg_cb,  queue_size=100)
        
        uwgps_topic = rospy.get_param('~uwgps_topic', '/station/uwgps')
        self.point_pub = rospy.Publisher(uwgps_topic, PointStamped, queue_size=100)

        self.send_uwgps = False
        send_uwgps_top = rospy.get_param('~send_uwgps_top', '/sam/core/imu')
        self.gui_sub = rospy.Subscriber(send_uwgps_top, Bool, self.gui_cb,  queue_size=1)
        
        self.uwgps_int = UWGPSInterface()

        r = rospy.Rate(self.node_freq)
        while not rospy.is_shutdown():

            print("------------------------")
            
            # Check rate of incoming data from SBG
            if self.sbg_init:
                if (abs(self.sbg_t - self.sbg_prev_t) > (1./self.node_freq) + 0.5):
                    rospy.logwarn("SBG data not coming in ")
                    rospy.logwarn("")
            else:
                rospy.logwarn("SBG has not started ")
                rospy.logwarn("")
            self.sbg_prev_t = self.sbg_t    

            # Leave this here to allow logwarn to work in every iteration of the loop

            # Acoustic position: x,y,z wrt to antenna
            acoustic_position = self.uwgps_int.get_acoustic_position(self.base_url)
            if acoustic_position:

                goal_point = PointStamped()
                goal_point.header.frame_id = uwgps_frame
                goal_point.header.stamp = rospy.Time(0)
                goal_point.point.x = acoustic_position["x"] 
                goal_point.point.y = acoustic_position["y"] 
                goal_point.point.z = acoustic_position["z"] 


            #if goal_point.point != goal_point_prev.point:
            #    goal_point_prev = goal_point

                try:
                    goal_base = self.listener.transformPoint(self.utm_frame, goal_point)
                    print("Current acoustic position in {} frame X: {}, Y: {}, Z: {}".format(
                            self.utm_frame,
                            goal_base.point.x,
                            goal_base.point.y,
                            goal_base.point.z))
                    # print("Goal in command station map frame")
                    # print(goal_base.point)
                    
                    # Publish to the UW comms when requested from the GUI
                    if self.send_uwgps:
                        self.send_uwgps = False
                        self.point_pub.publish(goal_base)

                except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("UWGPS module: Could not transform UGWPS WP to {}".format(self.utm_frame))
                    rospy.logwarn("")
                    pass
            else:
                rospy.logwarn("No acoustic position received")
                rospy.logwarn("")

            r.sleep()


if __name__ == "__main__":

    rospy.init_node("station_uwgps_node")

    try:
        uw_gps = UWGPSStation()
    except rospy.ROSInterruptException:
        pass
