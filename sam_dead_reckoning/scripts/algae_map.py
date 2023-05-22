#!/usr/bin/env python3

import rospy

import numpy as np

# from geometry_msgs.msg import Quaternion, TransformStamped
# from sensor_msgs.msg import NavSatFix
# from nav_msgs.msg import Odometry
# import tf
# from geodesy import utm
# import numpy as np
# import tf2_ros
# import message_filters

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, TransformStamped

import tf
import tf2_ros

from geodesy import utm

import time

class PubBuoyTf(object):
    
    def trigger_measurement_callback(self, msg):
        rospy.loginfo("Trigger measurement callback")
        self.measure_now = True
        
    def publisher_transform(self, navSatFix):
        
        num_buoys = len(self.buoy_positions)
        buoy_frame = "buoy_{}_frame".format(self.cnt)
        print("buoy ", buoy_frame)
        self.cnt += 1
        self.buoy_positions.append(navSatFix)
        
        buoy_utm = utm.fromLatLong(navSatFix.latitude, navSatFix.longitude)
        
        try:
            (world_trans, world_rot) = self.listener.lookupTransform(self.utm_frame, 
                                                                    buoy_frame,
                                                                    rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("GPS node: broadcasting transform %s to %s" % (self.utm_frame, buoy_frame))            
            transformStamped = TransformStamped()
            quat = [0,0,0,1.]
            #quat = tf.transformations.quaternion_from_euler(np.pi, -np.pi/2., 0., axes='rxzy')
            transformStamped.transform.translation.x = buoy_utm.easting
            transformStamped.transform.translation.y = buoy_utm.northing
            transformStamped.transform.translation.z = 0.
            transformStamped.transform.rotation = Quaternion(*quat)               
            transformStamped.header.frame_id = self.utm_frame
            transformStamped.child_frame_id = buoy_frame
            transformStamped.header.stamp = rospy.Time.now()
            self.static_tf_bc.sendTransform(transformStamped)
            
    def sam_gps(self, msg):
        self.last_gps_msg = msg
    
    
    def __init__(self):
        
        self.cnt = 0
        self.utm_frame = rospy.get_param('~utm_frame', 'utm')
        
        self.buoy_positions = []
        
        # Broadcast UTM to map frame
        self.listener = tf.TransformListener()        
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        
        self.lats = [58.2508586667, 58.25076345, 58.2507474333, 58.2506673333, 58.2507919833, 58.2509082667, 58.2509016, 58.2509062833, 58.25101055, 58.2510166]
        self.longs = [11.4514784167, 11.45136245, 11.4513825333, 11.4512373833, 11.4508920167, 11.4510152, 11.45100355,11.4509971833, 11.4511224667 , 11.4511216833]
    
        
    def loop(self):
        
        # Two measurements per second
        self.rate = rospy.Rate(50)
        
        for i in range(len(self.lats)):
            
            navsatfix = NavSatFix()
            navsatfix.latitude = self.lats[i]
            navsatfix.longitude = self.longs[i]
            
            self.publisher_transform(navsatfix)
            rospy.sleep(0.2)

        rospy.spin()
        
        
if __name__ == "__main__":

    rospy.init_node('gps_node', anonymous=False) #True)

    check_server = PubBuoyTf()
    check_server.loop()

    rospy.spin()
