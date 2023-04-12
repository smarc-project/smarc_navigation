#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math

from std_msgs.msg import Bool
from rospy.numpy_msg import numpy_msg
import tf2_ros
import message_filters
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

class DRStatsVisualization(object):
    
    def __init__(self):
        
        # Real mbes pings subscriber
        gps_odom_top = rospy.get_param("~gps_odom_topic", 'gps_odom')
        dr_odom_top = rospy.get_param("~dvl_dr_topic", 'gps_odom')
        pf_odom_top = rospy.get_param("~odom_corrected_topic", 'gps_odom')

        # PF ping subscriber

        self.gps_odom_sub = message_filters.Subscriber(gps_odom_top, Odometry)
        self.dr_odom_sub = message_filters.Subscriber(dr_odom_top, Odometry)
        self.pf_odom_sub = message_filters.Subscriber(pf_odom_top, Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.gps_odom_sub,
                                                               self.dr_odom_sub, 
                                                               self.pf_odom_sub],
                                                              20,
                                                              slop=20.0,
                                                              allow_headerless=False)

        self.ts.registerCallback(self.odom_cb)

        self.listener = tf.TransformListener()
        
        # # When the survey is finished, save the data to disk
        # finished_top = rospy.get_param("~survey_finished_top", '/survey_finished')
        # self.synch_pub = rospy.Subscriber(finished_top, Bool, self.synch_cb)
        # self.survey_finished = False

        # self.cov_traces = [0.]
        self.filter_cnt = 1

        self.gps_odom_vec = np.zeros((3, 1))
        self.dr_odom_vec = np.zeros((3, 1))
        self.pf_odom_vec = np.zeros((3, 1))

        while not rospy.is_shutdown():
            self.visualize()        
            rospy.Rate(1.).sleep()

        # Print out final distances
        rospy.on_shutdown(self.finish_hld)


    def finish_hld(self):
        dr_dist = 0.
        gps_dist = 0.
        pf_dist = 0.
        for i in range(1, self.dr_odom_vec.shape[1]):
            dr_dist += np.linalg.norm(self.dr_odom_vec[:,i] - self.dr_odom_vec[:,i-1])
            gps_dist += np.linalg.norm(self.gps_odom_vec[:,i] - self.gps_odom_vec[:,i-1])
            pf_dist += np.linalg.norm(self.pf_odom_vec[:,i] - self.pf_odom_vec[:,i-1])
        
        print("GPS distance ", gps_dist)
        print("DR distance ", dr_dist)
        print("PF distance ", pf_dist)


   
    def odom_cb(self, gps_msg, dr_msg, pf_msg):

        goal_point = PointStamped()
        goal_point.header.frame_id = "utm"
        goal_point.header.stamp = rospy.Time(0)
        goal_point.point.x = gps_msg.pose.pose.position.x
        goal_point.point.y = gps_msg.pose.pose.position.y
        goal_point.point.z = 0.

        try:
            gps_map = self.listener.transformPoint("sam/odom", goal_point)
            gps = np.array([gps_map.point.x,
                            gps_map.point.y,
                            gps_map.point.z])

            self.gps_odom_vec = np.hstack((self.gps_odom_vec, gps.reshape((3,1))))

            dr = np.array([dr_msg.pose.pose.position.x,
                           dr_msg.pose.pose.position.y,
                           dr_msg.pose.pose.position.z])

            self.dr_odom_vec = np.hstack((self.dr_odom_vec, dr.reshape((3,1))))


            pf = np.array([pf_msg.pose.pose.position.x,
                           pf_msg.pose.pose.position.y,
                           pf_msg.pose.pose.position.z])

            self.pf_odom_vec = np.hstack((self.pf_odom_vec, pf.reshape((3,1))))

            self.filter_cnt += 1
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Visual tools: Transform to utm-->map not available yet")
        pass

    def visualize(self):

        if self.filter_cnt > 0:
            
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            # Plot error between DR PF and GT
            if True:
                plt.subplot(2, 1, 1)
                plt.cla()
                plt.plot(np.linspace(0,self.filter_cnt, self.filter_cnt),
                         np.linalg.norm(self.gps_odom_vec-self.pf_odom_vec, axis=0), "-b")
                plt.grid(True)

                # Error between GPS and DR
                plt.subplot(2, 1, 2)
                plt.cla()
                # print(np.linalg.norm(self.gps_odom_vec-self.dr_odom_vec, axis=0))
                plt.plot(np.linspace(0,self.filter_cnt, self.filter_cnt),
                         np.linalg.norm(self.gps_odom_vec-self.dr_odom_vec, axis=0), "-r")

                plt.grid(True)

            plt.pause(0.00001)




if __name__ == "__main__":
    rospy.init_node("dr_statistics", disable_signals=False)
    try:
        DRStatsVisualization()
    except rospy.ROSInterruptException:
        pass
