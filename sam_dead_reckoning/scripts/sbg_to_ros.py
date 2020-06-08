#!/usr/bin/python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import TwistWithCovarianceStamped
import tf
import message_filters
from sbg_driver.msg import SbgImuData, SbgEkfQuat, SbgEkfEuler, SbgMag, SbgUtcTime
from sensor_msgs.msg import Imu

class SBG2ROS(object):

    def sbg_cb(self, sbg_imu, sbg_quat, sbg_mag):
        imu_msg = Imu()
        imu_msg.header.frame_id = self.imu_frame
        imu_msg.header.stamp = sbg_imu.header.stamp

        # NED to ENU
        imu_msg.orientation.x = sbg_quat.quaternion.y
        imu_msg.orientation.y = sbg_quat.quaternion.x
        imu_msg.orientation.z = - sbg_quat.quaternion.z
        imu_msg.orientation.w = sbg_quat.quaternion.w

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([imu_msg.orientation.x,
                                                                      imu_msg.orientation.y, 
                                                                      imu_msg.orientation.z,
                                                                      imu_msg.orientation.w])
 
        
        quat_t = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        imu_msg.orientation.x = quat_t[0]
        imu_msg.orientation.y = quat_t[1]
        imu_msg.orientation.z = quat_t[2]
        imu_msg.orientation.w = quat_t[3]
        
        imu_msg.angular_velocity.x = sbg_imu.gyro.y
        imu_msg.angular_velocity.y = sbg_imu.gyro.x
        imu_msg.angular_velocity.z = -sbg_imu.gyro.z

        imu_msg.linear_acceleration.x = sbg_imu.accel.y
        imu_msg.linear_acceleration.y = sbg_imu.accel.x
        imu_msg.linear_acceleration.z = -sbg_imu.accel.z
        
        imu_msg.linear_acceleration_covariance = [0.] * 9
        imu_msg.linear_acceleration_covariance[0] = 100
        imu_msg.linear_acceleration_covariance[4] = 100
        imu_msg.linear_acceleration_covariance[8] = 100
        
        self.imu_pub.publish(imu_msg)

    def __init__(self):
        self.sbg_imu_top = rospy.get_param('~sbg_imu_data', 'sbg/imu_data')
        self.sbg_quat_top = rospy.get_param('~sbg_ekf_quat', 'sbg/ekf_quat')
        self.sbg_mag_top = rospy.get_param('~sbg_mag', 'sbg/mag')
        self.sbg_utc_top = rospy.get_param('~sbg_utc_time', 'sbg/utc_time')
        self.imu_frame = rospy.get_param('~sbg_frame', 'sbg_link')
        self.sbg_imu_out = rospy.get_param('~sbg_imu_out', 'sbg_imu')

        self.imu_data_sub = message_filters.Subscriber(self.sbg_imu_top, SbgImuData)
        self.imu_quat_sub = message_filters.Subscriber(self.sbg_quat_top, SbgEkfQuat)
        self.imu_mag_sub = message_filters.Subscriber(self.sbg_mag_top, SbgMag)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.imu_data_sub, 
                                                               self.imu_quat_sub, self.imu_mag_sub],
                                                              10, slop=1.0, allow_headerless=False)
        self.ts.registerCallback(self.sbg_cb)

        self.imu_pub = rospy.Publisher(self.sbg_imu_out, Imu, queue_size=10)

        rospy.spin()

   

if __name__ == "__main__":
    rospy.init_node('sbg_2_ros_node')
    try:
        pi = SBG2ROS()
    except rospy.ROSInterruptException:
        pass
