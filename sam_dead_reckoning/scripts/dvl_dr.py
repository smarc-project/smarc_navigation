#!/usr/bin/python

import rospy
import numpy as np
import tf
from smarc_msgs.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped, Quaternion
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import message_filters
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolRequest
from sbg_driver.msg import SbgEkfEuler

class DVL2DR(object):

    def __init__(self):
        self.dvl_topic = rospy.get_param('~dvl_topic', '/sam/core/dvl')
        self.dvl_dr_top = rospy.get_param('~dvl_dr_topic', '/sam/dr/dvl_dr')                         #topic used in the launch file for the DVL sensor
        self.imu_topic = rospy.get_param('~imu', '/sam/core/imu')
        self.sbg_topic = rospy.get_param('~sbg_euler', '/sam/core/imu')
        self.base_frame = rospy.get_param('~base_frame', 'sam/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')
        self.dvl_frame = rospy.get_param('~dvl_link', 'dvl_link')
        self.filt_odom_top = rospy.get_param('~dr_odom_filtered', '/sam/dr/local/odom/filtered')

        self.sub_dvl = message_filters.Subscriber(self.dvl_topic, DVL)
        self.sub_imu = message_filters.Subscriber(self.imu_topic, Imu)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_dvl, self.sub_imu],
                                                          20, slop=20.0, allow_headerless=False)
        self.ts.registerCallback(self.drCB)

        self.listener = tf.TransformListener()
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        self.br = tf.TransformBroadcaster()

        self.pub_odom = rospy.Publisher(self.dvl_dr_top, Odometry, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.filt_odom_top, Odometry, self.odom_cb)
        self.euler_sub = rospy.Subscriber(self.sbg_topic, SbgEkfEuler, self.euler_cb)

        self.t_prev = rospy.Time.now()
        self.pose_prev = [0.] * 6
        self.odom_init = False
        self.init_heading = False
        #  self.roll_init = 0.
        #  self.pitch_init = 0.
        #  self.yaw_init = 0.
        self.transformStamped = TransformStamped()

        rospy.spin()

    def euler_cb(self, euler_msg):
        self.init_z = euler_msg.angle.z
        self.init_heading = True
        self.euler_sub.unregister()


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

        self.rot_t = np.matmul(rot_z, np.matmul(rot_y, rot_x))

    def drCB(self, dvl_msg, imu_msg):

        try:
            # goal_point_local = self.listener.transformPoint("map", goal_point)
            (world_trans, world_rot) = self.listener.lookupTransform("map", self.odom_frame, rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException):
            if self.init_heading:
                rospy.loginfo("Could not get transform between %s and %s" % ("map", self.odom_frame))            
                rospy.loginfo("so publishing first one...")
                # euler = euler_from_quaternion([imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w])
                # print(euler)
                quat = quaternion_from_euler(0.,0., self.init_z)

                self.transformStamped.transform.translation.x = 0.
                self.transformStamped.transform.translation.y = 0.
                self.transformStamped.transform.translation.z = 0.
                self.transformStamped.transform.rotation = Quaternion(*quat)
                self.transformStamped.header.frame_id = "map"
                self.transformStamped.child_frame_id = self.odom_frame
                self.transformStamped.header.stamp = rospy.Time.now()
                self.static_tf_bc.sendTransform(self.transformStamped)
                self.t_prev = rospy.Time.now()
            return

        # Velocity at time t
        t_now = rospy.Time.now()
        self.dt = (t_now - self.t_prev).to_sec()
        # self.dt = 0.19
        # print("dt, ", self.dt.to_sec())
        pose_t = [0.] * 6

        # Current pitch of floatsam
        # quat_t_imu = np.array([imu_msg.orientation.x,
        #                     imu_msg.orientation.y,
        #                     imu_msg.orientation.z,
        #                     imu_msg.orientation.w])

        # print("Rot Imu ", tf.transformations.euler_from_quaternion(quat_t_imu))

        # Integrate angular position
        vel_rot = np.array([imu_msg.angular_velocity.x,
                            imu_msg.angular_velocity.y,
                            imu_msg.angular_velocity.z])

        rot_t = np.array(self.pose_prev[3:6]) + vel_rot * self.dt

        roll_t = (rot_t[0] + np.pi) % (2 * np.pi) - np.pi
        pitch_t = (rot_t[1] + np.pi) % (2 * np.pi) - np.pi
        yaw_t = (rot_t[2] + np.pi) % (2 * np.pi) - np.pi

        # Or read them directly from compass
        # (roll_t, pitch_t, yaw_t) = tf.transformations.euler_from_quaternion([imu_msg.orientation.x,
        #                                                                   imu_msg.orientation.y,
        #                                                                   imu_msg.orientation.z,
        #                                                                   imu_msg.orientation.w])
        
        pose_t[3:6] = [roll_t, pitch_t, yaw_t]
        self.fullRotation(pose_t[3],pose_t[4],pose_t[5])
        step_t = np.matmul(self.rot_t, np.array([dvl_msg.velocity.x,
                                               dvl_msg.velocity.y,
                                               0.])*self.dt)

        # Integrate velocities
        pose_t[0:3] = self.pose_prev[0:3] + step_t
        quat_t = tf.transformations.quaternion_from_euler(pose_t[3],pose_t[4],pose_t[5])

        odom_msg = Odometry()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = "sam_test"
        odom_msg.pose.pose.position.x = pose_t[0]
        odom_msg.pose.pose.position.y = pose_t[1]
        odom_msg.pose.pose.position.z = 0.
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.pose.orientation = Quaternion(*quat_t)
        self.pub_odom.publish(odom_msg)

        self.t_prev = t_now
        self.pose_prev = pose_t

        self.br.sendTransform([pose_t[0], pose_t[1], 0.],
                    quat_t,
                    rospy.Time.now(),
                    "sam_test",
                    self.odom_frame)

        self.transformStamped.header.stamp = rospy.Time.now()
        self.static_tf_bc.sendTransform(self.transformStamped)
        



if __name__ == "__main__":
    rospy.init_node('dvl_dr')
    try:
        pi = DVL2DR()
    except rospy.ROSInterruptException:
        pass
