#!/usr/bin/python

import rospy
import numpy as np
from sam_msgs.msg import ThrusterRPMs
from geometry_msgs.msg import TwistStamped
from uavcan_ros_bridge.msg import ESCStatus
from sbg_driver.msg import SbgEkfEuler
from nav_msgs.msg import Odometry
import message_filters
from sensor_msgs.msg import Imu
import tf

class SamMM(object):

    def __init__(self):
        
        self.dr_thrust_topic = rospy.get_param('~thrust_dr', '/odom_dr')
        self.rpm_fb_topic = rospy.get_param('~thrust_fb', '/rpm_fb')
        self.imu_topic = rospy.get_param('~sbg_imu', '/sbg/imu')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')

        self.subs_thrust = message_filters.Subscriber(self.rpm_fb_topic, ThrusterRPMs)
        self.subs_imu = message_filters.Subscriber(self.imu_topic, Imu)  
        self.ts = message_filters.ApproximateTimeSynchronizer([self.subs_thrust, self.subs_imu],
                                                          50, slop=50.0, allow_headerless=True)
        self.ts.registerCallback(self.drCB)

        self.pub_odom = rospy.Publisher(self.dr_thrust_topic, Odometry, queue_size=10)

        self.coeff = 0.0005
        self.t_prev = rospy.Time.now()
        self.position_prev = [0.] * 3
        
        rospy.spin()

    def fullRotation(self, roll, pitch, yaw):
        rot_z = np.array([[np.cos(yaw), -np.sin(yaw), 0.0],
                          [np.sin(yaw), np.cos(yaw), 0.0],
                          [0., 0., 1]])
        rot_y = np.array([[np.cos(pitch), 0.0, np.sin(pitch)],
                          [0., 1., 0.],
                          [-np.sin(pitch), np.cos(pitch), 0.0]])
        rot_x = np.array([[1., 0., 0.],
                          [0., np.cos(roll), -np.sin(roll)],
                          [0., np.sin(roll), np.cos(roll)]])

        self.rot_t = rot_z*rot_y*rot_x

    def drCB(self, rpm_msg, imu_msg):

        print("Hola!")
        # Velocity at time t
        thrust = (rpm_msg.thruster_1_rpm + rpm_msg.thruster_2_rpm) * self.coeff
        t_now = rospy.Time.now()
        self.dt = (t_now - self.t_prev)

        # Current angles
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(imu_msg.orientation)
        self.fullRotation(roll, pitch, yaw)
        vel_t = np.matmul(self.rot_t, np.array([thrust, 0., 0.]))

        # Integrate velocities
        position_t = self.position_prev + vel_t * self.dt.to_sec() 
        quat_t = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        odom_msg = Odometry()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = position_t[0]
        odom_msg.pose.pose.position.y = position_t[1]
        odom_msg.pose.pose.position.z = position_t[2]
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.covariance[0] = 20.
        odom_msg.pose.covariance[7] = 20.
        odom_msg.pose.covariance[16] = 40.

        odom_msg.pose.pose.orientation.x = quat_t[0]
        odom_msg.pose.pose.orientation.y = quat_t[1]
        odom_msg.pose.pose.orientation.z = quat_t[2]
        odom_msg.pose.pose.orientation.w = quat_t[3]

        self.pub_odom.publish(odom_msg)

        self.t_prev = t_now 
        self.position_prev = position_t 

    #  def thrustCB(self, rpm_msg):
#
        #  self.rpm_0 = rpm_msg.thruster_1_rpm
        #  self.rpm_1 = rpm_msg.thruster_2_rpm
#
        #  twist_msg = TwistStamped()
        #  twist_msg.header.stamp = rospy.Time.now()
        #  twist_msg.header.frame_id = "sam/base_link"
        #  twist_msg.twist.linear.x = (self.rpm_0 + self.rpm_1) * self.coeff
        #  twist_msg.twist.linear.y = 0.0
#            msg_odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
#                                         0.0, 1, 0.0, 0.0, 0.0, 0.0,
#                                         0.0, 0.0, 1, 0.0, 0.0, 0.0,
#                                         0.0, 0.0, 0.0, 1, 0.0, 0.0,
#                                         0.0, 0.0, 0.0, 0.0, 1, 0.0,
#                                         0.0, 0.0, 0.0, 0.0, 0.0, 1]

        #  self.control_pub.publish(twist_msg)


if __name__ == "__main__":

    rospy.init_node('sam_mm')
    try:
        SamMM()
    except rospy.ROSInterruptException:
        pass
