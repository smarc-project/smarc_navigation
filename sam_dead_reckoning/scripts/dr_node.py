#!/usr/bin/python

import rospy
import numpy as np
import tf
from smarc_msgs.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped, Quaternion, Vector3
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
        self.stim_topic = rospy.get_param('~imu', '/sam/core/imu')
        self.sbg_topic = rospy.get_param('~sbg_euler', '/sam/core/imu')
        self.base_frame = rospy.get_param('~base_frame', 'sam/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.dvl_frame = rospy.get_param('~dvl_frame', 'dvl_link')
        self.dvl_period = rospy.get_param('~dvl_period', 0.2)
        self.dr_period = rospy.get_param('~dr_period', 0.02)
        # self.dr_pub_period = rospy.get_param('~dr_pub_period', 0.1)

        self.listener = tf.TransformListener()
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        self.br = tf.TransformBroadcaster()
        self.transformStamped = TransformStamped()

        self.pub_odom = rospy.Publisher(self.dvl_dr_top, Odometry, queue_size=100)
        self.euler_sub = rospy.Subscriber(self.sbg_topic, SbgEkfEuler, self.euler_cb)
        self.dvl_sub = rospy.Subscriber(self.dvl_topic, DVL, self.dvl_cb)
        self.stim_sub = rospy.Subscriber(self.stim_topic, Imu, self.stim_cb)

        self.t_prev = rospy.Time.now()
        self.pose_prev = [0.] * 6
        self.init_heading = False
        
        # Stim integration
        self.rot_t = [0.] * 3
        self.t_stim_prev = 0.
        self.init_stim = False
        self.vel_rot = [0.] * 3

        # Useful when working with rosbags
        self.t_start = 0.   
        self.t_now = 0. 
        self.t_pub = 0. 

        # DVL integration
        self.pos_t = [0.] * 3
        self.t_dvl_prev = 0.
        self.dvl_on = False
        self.dvl_latest = DVL()
        self.dvl_latest.velocity.x = 0.
        self.dvl_latest.velocity.y = 0.
        self.dvl_latest.velocity.z = 0.

        # Motion model 
        self.mm_on = False

        rospy.Timer(rospy.Duration(self.dr_period), self.dr_timer)

        rospy.spin()

    def dr_timer(self, event):
        
        try:
            # goal_point_local = self.listener.transformPoint("map", goal_point)
            (world_trans, world_rot) = self.listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException):
            if self.init_heading:
                rospy.loginfo("Could not get transform between %s and %s" % (self.map_frame, self.odom_frame))            
                rospy.loginfo("so publishing first one...")

                # SBG points to north while map's x axis points east (ENU) so 
                # the map --> odom tf needs an extra +90 deg turn in z 
                quat = quaternion_from_euler(0.,0., self.init_z + np.pi/2.)

                self.transformStamped.transform.translation.x = 0.
                self.transformStamped.transform.translation.y = 0.
                self.transformStamped.transform.translation.z = 0.
                self.transformStamped.transform.rotation = Quaternion(*quat)
                self.transformStamped.header.frame_id = self.map_frame
                self.transformStamped.child_frame_id = self.odom_frame
                self.transformStamped.header.stamp = rospy.Time.now()
                self.static_tf_bc.sendTransform(self.transformStamped)
            return

        pose_t = np.concatenate([self.pos_t, self.rot_t])    # Catch latest estimate from IMU
        rot_vel_t = self.vel_rot    # TODO: rn this keeps the last vels even if the IMU dies
        lin_vel_t = [0.] * 3

        # DVL data coming in
        if self.dvl_on:

            # Integrate linear velocities from DVL
            # If last DVL msg isn't too old
            if self.t_now - self.t_dvl_prev < self.dvl_period:
                rot_mat_t = self.fullRotation(pose_t[3],pose_t[4],pose_t[5])
                lin_vel_t = np.array([self.dvl_latest.velocity.x,
                                    self.dvl_latest.velocity.y,
                                    0.])
                step_t = np.matmul(rot_mat_t, lin_vel_t * self.dr_period)

                # Current pose
                self.pos_t = self.pos_t + step_t        
                pose_t[0:3] = self.pos_t
            
            # Otherwise, integrate motion model estimate
            else:
                rospy.logdebug("Missing DVL data, motion model kicking in")

                # Integrate linear velocities from motion model
                # TODO: subscribe to rpm topics
                if self.mm_on:
                    # rot_mat_t = self.fullRotation(pose_t[3],pose_t[4],pose_t[5])
                    # lin_vel_t = np.array([self.dvl_latest.velocity.x,
                    #                 self.dvl_latest.velocity.y,
                    #                 0.])
                    # step_t = np.matmul(rot_mat_t, lin_vel_t * self.dr_period)

                    # Current pose
                    self.pos_t = self.pos_t + step_t        
                    pose_t[0:3] = self.pos_t


        # if self.t_now - self.t_pub > self.dr_pub_period:
        # Publish and broadcast aux frame for testing
        quat_t = tf.transformations.quaternion_from_euler(pose_t[3],pose_t[4],pose_t[5])
        odom_msg = Odometry()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = "sam_test"
        odom_msg.pose.pose.position.x = pose_t[0]
        odom_msg.pose.pose.position.y = pose_t[1]
        odom_msg.pose.pose.position.z = 0.
        odom_msg.twist.twist.linear.x = lin_vel_t[0]
        odom_msg.twist.twist.linear.y = lin_vel_t[1]
        odom_msg.twist.twist.linear.z = lin_vel_t[2]
        odom_msg.twist.twist.angular.x = rot_vel_t[0]
        odom_msg.twist.twist.angular.y = rot_vel_t[1]
        odom_msg.twist.twist.angular.z = rot_vel_t[2]
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.pose.orientation = Quaternion(*quat_t)
        self.pub_odom.publish(odom_msg)

        self.br.sendTransform([pose_t[0], pose_t[1], 0.],
                    quat_t,
                    rospy.Time.now(),
                    "sam_test",
                    self.odom_frame)

        self.t_now += 0.02


    def euler_cb(self, euler_msg):
        self.init_z = euler_msg.angle.z
        self.init_heading = True
        # self.euler_sub.unregister()


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

        return np.matmul(rot_z, np.matmul(rot_y, rot_x))

    def stim_cb(self, imu_msg):
        if self.init_stim:
            self.rot_stim = np.array([imu_msg.orientation.x,
                                    imu_msg.orientation.y,
                                    imu_msg.orientation.z,
                                    imu_msg.orientation.w])
            
            # Integrate angular velocities
            self.vel_rot = np.array([imu_msg.angular_velocity.x,
                                imu_msg.angular_velocity.y,
                                imu_msg.angular_velocity.z])

            dt = imu_msg.header.stamp.to_sec() - self.t_stim_prev
            self.rot_t = np.array(self.rot_t) + self.vel_rot * dt

            for rot in self.rot_t:
                rot = (rot + np.pi) % (2 * np.pi) - np.pi

            self.t_stim_prev = imu_msg.header.stamp.to_sec()

        else:
            rospy.loginfo("Stim data coming in")
            self.t_stim_prev = imu_msg.header.stamp.to_sec()
            self.init_stim = True

    # TODO: potential synch issue here with stim and dvl on variables
    def dvl_cb(self, dvl_msg):
        if self.init_stim:

            # Project velocities into odom frame to correct for pitching of floatsam
            euler = euler_from_quaternion(self.rot_stim)
            mat_t = self.fullRotation(euler[0],euler[1],euler[2])

            aux  = np.matmul(mat_t, np.array([dvl_msg.velocity.x,
                                            dvl_msg.velocity.y,
                                            dvl_msg.velocity.z]))
            dvl_msg.velocity.x = aux[0]
            dvl_msg.velocity.y = aux[1]
            dvl_msg.velocity.z = aux[2]
            self.dvl_latest = dvl_msg
        
        if self.dvl_on:
            dt = dvl_msg.header.stamp.to_sec() - self.t_dvl_prev
            self.t_dvl_prev = dvl_msg.header.stamp.to_sec()

            # if dt > self.dvl_period:
            #     print("Missed DVL meas")            

        else:
            rospy.loginfo("DVL data coming in")
            self.t_dvl_prev = dvl_msg.header.stamp.to_sec()
            self.t_now = dvl_msg.header.stamp.to_sec()
            self.t_pub = dvl_msg.header.stamp.to_sec()
            self.dvl_on = True


if __name__ == "__main__":
    rospy.init_node('dr_node')
    try:
        pi = DVL2DR()
    except rospy.ROSInterruptException:
        pass
