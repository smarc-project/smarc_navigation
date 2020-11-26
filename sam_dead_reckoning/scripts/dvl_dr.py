#!/usr/bin/python

import rospy
import numpy as np
import tf
from cola2_msgs.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped
import tf
import message_filters
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolRequest

class DVL2DR(object):

    def __init__(self):
        self.dvl_topic = rospy.get_param('~dvl_topic', '/sam/core/dvl')
        self.dvl_dr_top = rospy.get_param('~dvl_dr_topic', '/sam/dr/dvl_dr')
        self.imu_topic = rospy.get_param('~sbg_imu', '/sam/core/sbg_imu')
        self.base_frame = rospy.get_param('~base_frame', 'sam/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')
        self.dvl_frame = rospy.get_param('~dvl_link', 'dvl_link')
        self.filt_odom_top = rospy.get_param('~dr_odom_filtered', '/sam/dr/local/odom/filtered')

        self.sub_dvl = message_filters.Subscriber(self.dvl_topic, DVL)
        self.sub_imu = message_filters.Subscriber(self.imu_topic, Imu)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_dvl, self.sub_imu],
                                                          20, slop=20.0, allow_headerless=True)
        self.ts.registerCallback(self.drCB)

        self.pub_odom = rospy.Publisher(self.dvl_dr_top, Odometry, queue_size=10)

        self.odom_sub = rospy.Subscriber(self.filt_odom_top, Odometry, self.odom_cb)

        self.t_prev = rospy.Time.now()
        self.position_prev = [0.] * 3
        self.odom_init = False
        #  self.roll_init = 0.
        #  self.pitch_init = 0.
        #  self.yaw_init = 0.

        rospy.spin()

    def odom_cb(self, odom_msg):
        self.filtered_odom = [odom_msg.pose.pose.position.x,
                              odom_msg.pose.pose.position.y,
                              odom_msg.pose.pose.position.z,
                             odom_msg.header.stamp]
        self.odom_init = True
        self.odom_sub.unregister()


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
        # Get this message only once to init the DR based on latest odom filtered
        if self.odom_init:
            self.position_prev = self.filtered_odom[:3]
            self.t_prev = self.filtered_odom[3]
            self.odom_init = False

        # Velocity at time t
        t_now = rospy.Time.now()
        self.dt = (t_now - self.t_prev)

        # TODO: compute yaw from odom, not straight from compass
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([imu_msg.orientation.x,
                                                                          imu_msg.orientation.y,
                                                                          imu_msg.orientation.z,
                                                                          imu_msg.orientation.w])
        roll_t = ((roll) + 2 * np.pi) % (2 * np.pi)
        pitch_t = ((pitch) + 2 * np.pi) % (2 * np.pi)
        yaw_t = ((yaw) + 2 * np.pi) % (2 * np.pi)

        self.fullRotation(roll_t, pitch_t, yaw_t)
        step_t = np.matmul(self.rot_t, np.array([dvl_msg.velocity.x,
                                               dvl_msg.velocity.y,
                                               dvl_msg.velocity.z])*self.dt.to_sec())

        # Integrate velocities
        position_t = self.position_prev + step_t
        quat_t = tf.transformations.quaternion_from_euler(roll_t, pitch_t, yaw_t)

        odom_msg = Odometry()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = self.dvl_frame
        odom_msg.pose.pose.position.x = position_t[0]
        odom_msg.pose.pose.position.y = position_t[1]
        odom_msg.pose.pose.position.z = position_t[2]
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.covariance[0] = 10.
        odom_msg.pose.covariance[7] = 10.
        odom_msg.pose.covariance[16] = 20.

        odom_msg.pose.pose.orientation.x = quat_t[0]
        odom_msg.pose.pose.orientation.y = quat_t[1]
        odom_msg.pose.pose.orientation.z = quat_t[2]
        odom_msg.pose.pose.orientation.w = quat_t[3]

        self.pub_odom.publish(odom_msg)

        self.t_prev = t_now
        self.position_prev = position_t

        br = tf.TransformBroadcaster()
        br.sendTransform(position_t,
                    quat_t,
                    rospy.Time.now(),
                    "sam_test",
                    self.odom_frame)



if __name__ == "__main__":
    rospy.init_node('dvl_dr')
    try:
        pi = DVL2DR()
    except rospy.ROSInterruptException:
        pass
