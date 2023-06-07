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

        self.gps_topic = rospy.get_param('~gps_topic', '/sam/core/gps')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.utm_frame = rospy.get_param('~utm_frame', 'utm')
        self.gps_frame = rospy.get_param('~gps_frame', 'sam/gps_link')
        self.gps_odom_top = rospy.get_param('~gps_odom_topic', 'gps_odom_sam')

        # GPS odom in UTM frame
        self.gps_sam_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.sam_gps_cb)
        self.gps_sam_pub = rospy.Publisher(self.gps_odom_top, Odometry, queue_size=10)
        
        # Broadcast UTM to map frame
        self.listener = tf.TransformListener()        
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        
        # Auxiliar ones for floatsam
        self.odom_pub = rospy.Publisher('gps_odom', Odometry, queue_size=10)
        self.gps_prt_pub = rospy.Publisher('gps_odom_prt', Odometry, queue_size=10)
        self.gps_stb_pub = rospy.Publisher('gps_odom_stb', Odometry, queue_size=10)
        self.gps_prt_sub = message_filters.Subscriber("/sam/core/gps/prt", NavSatFix)
        self.gps_stb_sub = message_filters.Subscriber("/sam/core/gps/stb", NavSatFix)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.gps_prt_sub, self.gps_stb_sub],
                                                          20, slop=20.0, allow_headerless=False)
        self.ts.registerCallback(self.gps_callback)


    def sam_gps_cb(self, sam_gps):

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
            odom_msg.child_frame_id = self.gps_frame
            odom_msg.pose.covariance = [0.] * 36
            odom_msg.pose.pose.position.x = utm_sam.easting
            odom_msg.pose.pose.position.y = utm_sam.northing
            odom_msg.pose.pose.position.z = 0.
            odom_msg.pose.pose.orientation = Quaternion(*rot)
            self.gps_sam_pub.publish(odom_msg)


    def gps_callback(self, prt_msg, stb_msg):

        if prt_msg.status.status == -1:
            return

        # lat long to UTM
        utm_prt = utm.fromLatLong(prt_msg.latitude, prt_msg.longitude)
        utm_stb = utm.fromLatLong(stb_msg.latitude, stb_msg.longitude)
        # prt - stb
        diff = np.array([utm_prt.northing, utm_prt.easting]) - np.array([utm_stb.northing, utm_stb.easting]) 
        # mid point
        utm_mid = diff/2. + np.array([utm_stb.northing, utm_stb.easting]) 
        heading = np.arctan2(diff[1], diff[0]) - np.pi/2.0

    #TODO: flip easting northing where required :D
        #### Auxiliar ones for floatsam
        rot = [0., 0., 0., 1.]
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = self.utm_frame
        odom_msg.child_frame_id = self.gps_frame
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.pose.position.x = utm_mid[0]
        odom_msg.pose.pose.position.y = utm_mid[1]
        odom_msg.pose.pose.position.z = 0.
        odom_msg.pose.pose.orientation = Quaternion(*rot)
        self.odom_pub.publish(odom_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = self.utm_frame
        odom_msg.child_frame_id = self.gps_frame
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.pose.position.x = utm_prt.northing
        odom_msg.pose.pose.position.y = utm_prt.easting
        odom_msg.pose.pose.position.z = 0.
        odom_msg.pose.pose.orientation = Quaternion(*rot)
        self.gps_prt_pub.publish(odom_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = self.utm_frame
        odom_msg.child_frame_id = self.gps_frame
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.pose.position.x = utm_stb.northing
        odom_msg.pose.pose.position.y = utm_stb.easting
        odom_msg.pose.pose.position.z = 0.
        odom_msg.pose.pose.orientation = Quaternion(*rot)
        self.gps_stb_pub.publish(odom_msg)



if __name__ == "__main__":

    rospy.init_node('gps_node', anonymous=False) #True)

    check_server = PublishGPSPose(rospy.get_name())

    rospy.spin()
