#!/usr/bin/python

import rospy
from rospy import ROSException
from std_msgs.msg import Header, Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, TransformStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sam_msgs.msg import GetGPSFixAction, GetGPSFixFeedback, GetGPSFixResult
from sam_msgs.msg import PercentStamped 
from nav_msgs.msg import Odometry
import actionlib
import tf_conversions
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geodesy import utm
import math
import numpy as np
import tf2_ros
from sbg_driver.msg import SbgEkfEuler
import message_filters

class PublishGPSPose(object):

    def __init__(self, name):

        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('gps_odom', Odometry, queue_size=10)
        self.gps_prt_pub = rospy.Publisher('gps_odom_prt', Odometry, queue_size=10)
        self.gps_stb_pub = rospy.Publisher('gps_odom_stb', Odometry, queue_size=10)
        self.gps_sam_pub = rospy.Publisher('gps_odom_sam', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher('gps_pose', PoseWithCovarianceStamped, queue_size=10)

        # self.sbg_sub = rospy.Subscriber("/sam/sbg/ekf_euler", SbgEkfEuler, self.sbg_cb)

        self.listener = tf.TransformListener()        
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()

        self.first_gps = True
        self.sbg_init = False
        
        self.gps_prt_sub = message_filters.Subscriber("/sam/core/gps/prt", NavSatFix)
        self.gps_stb_sub = message_filters.Subscriber("/sam/core/gps/stb", NavSatFix)
        self.gps_sam_sub = message_filters.Subscriber("/sam/core/gps", NavSatFix)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.gps_prt_sub, self.gps_stb_sub, self.gps_sam_sub],
                                                          20, slop=20.0, allow_headerless=False)
        self.ts.registerCallback(self.gps_callback)

        # rospy.Subscriber("/sam/core/gps/prt", NavSatFix, self.gps_callback)

    # def sbg_cb(self, sbg_msg):
    #     self.sbg_init = True
    #     self.init_euler = sbg_msg.angle

    #     self.sbg_sub.unregister()

    def gps_callback(self, prt_msg, stb_msg, sam_msg):

        if prt_msg.status.status == -1:
            return

        # lat long to UTM
        utm_prt = utm.fromLatLong(prt_msg.latitude, prt_msg.longitude)
        utm_stb = utm.fromLatLong(stb_msg.latitude, stb_msg.longitude)
        utm_sam = utm.fromLatLong(sam_msg.latitude, sam_msg.longitude)
        # prt - stb
        diff = np.array([utm_prt.northing, utm_prt.easting]) - np.array([utm_stb.northing, utm_stb.easting]) 
        # mid point
        utm_mid = diff/2. + np.array([utm_stb.northing, utm_stb.easting]) 
        heading = np.arctan2(diff[1], diff[0]) - np.pi/2.0

        try:
            # goal_point_local = self.listener.transformPoint("map", goal_point)
            (world_trans, world_rot) = self.listener.lookupTransform("utm", "map", rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("Could not get transform between %s and %s" % ("utm", "map"))            
            rospy.loginfo("so publishing first one...")
            transformStamped = TransformStamped()
            # TODO: use utm_sam (SAM's GPS) for final tests
            transformStamped.transform.translation.x = utm_mid[0]
            # transformStamped.transform.translation.x = utm_sam.northing
            transformStamped.transform.translation.y = utm_mid[1]
            # transformStamped.transform.translation.y = utm_sam.easting
            transformStamped.transform.translation.z = 0.
            transformStamped.transform.rotation.x = 1.               
            transformStamped.transform.rotation.y = 0.
            transformStamped.transform.rotation.z = 0.
            transformStamped.transform.rotation.w = 0.
            transformStamped.header.frame_id = "utm"
            transformStamped.child_frame_id = "map"
            transformStamped.header.stamp = rospy.Time.now()
            self.static_tf_bc.sendTransform(transformStamped)

            return
            
        # pos = np.array([world_trans[0]-easting, world_trans[1]-northing, 0.])
        # self.fullRotation(0., 0.,self.init_euler.z) # from the initial heading of the SBG
        # pos_map = np.matmul(self.rot_t, pos)
        # pos_map[2] = pos
        # pos_map = pos

        rot = [0., 0., 0., 1.]

        quat = quaternion_from_euler(0.,0., heading)
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.frame_id = "utm"
        odom_msg.child_frame_id = "sam/gps_link"
        odom_msg.pose.covariance = [0.] * 36
        #self.odom_msg.pose.pose.orientation.w = 1.
        # odom_msg.pose.pose.position = Point(*pos_map.tolist())
        odom_msg.pose.pose.position.x = utm_mid[0]
        odom_msg.pose.pose.position.y = utm_mid[1]
        odom_msg.pose.pose.position.z = 0.
        # odom_msg.pose.pose.position = goal_point_local.point
        odom_msg.pose.pose.orientation = Quaternion(*rot)
        self.odom_pub.publish(odom_msg)

        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.frame_id = "utm"
        odom_msg.child_frame_id = "sam/gps_link"
        odom_msg.pose.covariance = [0.] * 36
        #self.odom_msg.pose.pose.orientation.w = 1.
        # odom_msg.pose.pose.position = Point(*pos_map.tolist())
        odom_msg.pose.pose.position.x = utm_prt.northing
        odom_msg.pose.pose.position.y = utm_prt.easting
        odom_msg.pose.pose.position.z = 0.
        # odom_msg.pose.pose.position = goal_point_local.point
        odom_msg.pose.pose.orientation = Quaternion(*rot)
        self.gps_prt_pub.publish(odom_msg)

        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.frame_id = "utm"
        odom_msg.child_frame_id = "sam/gps_link"
        odom_msg.pose.covariance = [0.] * 36
        #self.odom_msg.pose.pose.orientation.w = 1.
        # odom_msg.pose.pose.position = Point(*pos_map.tolist())
        odom_msg.pose.pose.position.x = utm_stb.northing
        odom_msg.pose.pose.position.y = utm_stb.easting
        odom_msg.pose.pose.position.z = 0.
        # odom_msg.pose.pose.position = goal_point_local.point
        odom_msg.pose.pose.orientation = Quaternion(*rot)
        self.gps_stb_pub.publish(odom_msg)


        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.frame_id = "utm"
        odom_msg.child_frame_id = "sam/gps_link"
        odom_msg.pose.covariance = [0.] * 36
        #self.odom_msg.pose.pose.orientation.w = 1.
        # odom_msg.pose.pose.position = Point(*pos_map.tolist())
        odom_msg.pose.pose.position.x = utm_sam.northing
        odom_msg.pose.pose.position.y = utm_sam.easting
        odom_msg.pose.pose.position.z = 0.
        # odom_msg.pose.pose.position = goal_point_local.point
        odom_msg.pose.pose.orientation = Quaternion(*rot)
        self.gps_sam_pub.publish(odom_msg)


if __name__ == "__main__":

    rospy.init_node('publish_gps_pose', anonymous=False) #True)

    check_server = PublishGPSPose(rospy.get_name())

    rospy.spin()
