#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf
from geodesy import utm
import numpy as np
import tf2_ros
import message_filters

class PublishGPSPose(object):

    def __init__(self, name):

        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.utm_frame = rospy.get_param('~utm_frame', 'utm')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.gps_main_frame = rospy.get_param('~gps_main_frame', 'sam/gps_link')
        self.rtk_prt_frame = rospy.get_param('~rtk_prt_frame', 'rtk_prt_link')
        self.rtk_stb_frame = rospy.get_param('~rtk_stb_frame', 'rtk_stb_link')
        
        # Broadcast UTM to map frame
        self.listener = tf.TransformListener()        
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        
        # Auxiliar ones for floatsam
        self.rtk_odom_top = rospy.get_param('~rtk_odom_topic', 'gps_odom_sam')
        self.rtk_prt_odom_top = rospy.get_param('~rtk_prt_odom_top', 'gps_odom_sam')
        self.rtk_stb_odom_top = rospy.get_param('~rtk_stb_odom_top', 'gps_odom_sam')
        self.rtk_odom_pub = rospy.Publisher(self.rtk_odom_top, Odometry, queue_size=10)
        self.rtk_prt_pub = rospy.Publisher(self.rtk_prt_odom_top, Odometry, queue_size=10)
        self.rtk_stb_pub = rospy.Publisher(self.rtk_stb_odom_top, Odometry, queue_size=10)
        
        self.rtk_prt_top = rospy.get_param('~rtk_prt_topic', 'core/rtk_0_gps')
        self.rtk_stb_top = rospy.get_param('~rtk_stb_topic', 'core/rtk_0_gps')
        self.gps_prt_sub = message_filters.Subscriber(self.rtk_prt_top, NavSatFix)
        self.gps_stb_sub = message_filters.Subscriber(self.rtk_stb_top, NavSatFix)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.gps_prt_sub, self.gps_stb_sub],
                                                          20, slop=20.0, allow_headerless=False)
        self.ts.registerCallback(self.rtk_callback)

        # GPS odom in UTM frame
        self.gps_main_topic = rospy.get_param('~aux_gps_topic', 'core/gps')
        self.gps_odom_top = rospy.get_param('~aux_gps_odom_topic', 'gps_odom_sam')
        self.gps_main_pub = rospy.Publisher(self.gps_odom_top, Odometry, queue_size=10)
        self.gps_main_sub = rospy.Subscriber(self.gps_main_topic, NavSatFix, self.main_gps_cb)
        

    def main_gps_cb(self, sam_gps):

        if sam_gps.status.status != -1:

            utm_sam = utm.fromLatLong(sam_gps.latitude, sam_gps.longitude)
            rot = [0., 0., 0., 1.]
            
            try:
                (world_trans, world_rot) = self.listener.lookupTransform(self.utm_frame, 
                                                                        self.map_frame,
                                                                        rospy.Time(0))

            except (tf.LookupException, tf.ConnectivityException):
                rospy.loginfo("GPS node: broadcasting transform %s to %s" % (self.utm_frame, self.map_frame))            
                transformStamped = TransformStamped()
                transformStamped.transform.translation.x = utm_sam.easting
                transformStamped.transform.translation.y = utm_sam.northing
                transformStamped.transform.translation.z = 0.
                transformStamped.transform.rotation = Quaternion(*rot)               
                transformStamped.header.frame_id = self.utm_frame
                transformStamped.child_frame_id = self.map_frame
                transformStamped.header.stamp = rospy.Time.now()
                self.static_tf_bc.sendTransform(transformStamped)

                return

            # For SAM GPS
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = self.utm_frame
            odom_msg.child_frame_id = self.gps_main_frame
            odom_msg.pose.covariance = [0.] * 36
            odom_msg.pose.pose.position.x = utm_sam.easting
            odom_msg.pose.pose.position.y = utm_sam.northing
            odom_msg.pose.pose.position.z = 0.
            odom_msg.pose.pose.orientation = Quaternion(*rot)
            self.gps_main_pub.publish(odom_msg)


    def rtk_callback(self, prt_msg, stb_msg):

        if prt_msg.status.status == -1:
            return

        # lat long to UTM
        utm_prt = utm.fromLatLong(prt_msg.latitude, prt_msg.longitude)
        utm_stb = utm.fromLatLong(stb_msg.latitude, stb_msg.longitude)
        # prt - stb
        diff = np.array([utm_prt.easting, utm_prt.northing]) - \
            np.array([utm_stb.easting, utm_stb.northing])
        
        # mid point
        utm_mid = diff/2. + np.array([utm_stb.easting, utm_stb.northing])
        heading = np.arctan2(diff[1], diff[0]) - np.pi/2.0

        rot = [0., 0., 0., 1.]
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = self.utm_frame
        odom_msg.pose.pose.orientation = Quaternion(*rot)
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.pose.position.z = 0.

        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = utm_mid[0]
        odom_msg.pose.pose.position.y = utm_mid[1]
        self.rtk_odom_pub.publish(odom_msg)

        odom_msg.child_frame_id = self.rtk_prt_frame
        odom_msg.pose.pose.position.x = utm_prt.easting
        odom_msg.pose.pose.position.y = utm_prt.northing
        self.rtk_prt_pub.publish(odom_msg)

        odom_msg.child_frame_id = self.rtk_stb_frame
        odom_msg.pose.pose.position.x = utm_stb.easting
        odom_msg.pose.pose.position.y = utm_stb.northing
        self.rtk_stb_pub.publish(odom_msg)



if __name__ == "__main__":

    rospy.init_node('usv_gps_node', anonymous=False) #True)

    check_server = PublishGPSPose(rospy.get_name())

    rospy.spin()
