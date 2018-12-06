#!/usr/bin/python

# Copyright 2018 Nils Bore (nbore@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import numpy as np

class ImuRepublisher(object):

    def __init__(self):

        msg = Imu()
        self.pub = rospy.Publisher("/imu", Imu)

    def imu_callback(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "sam_auv/base_link" 
        cov = list(msg.orientation_covariance)
        cov[0] = 0.5
        cov[4] = 0.5
        cov[8] = 0.5
        msg.orientation_covariance = cov
        #msg.orientation_covariance[0] = 0.5
        #msg.orientation_covariance[4] = 0.5
        #msg.orientation_covariance[8] = 0.5
        #msg.linear_acceleration_covariance[0] = 0.005
        #msg.linear_acceleration_covariance[4] = 0.005
        #msg.linear_acceleration_covariance[8] = 0.005
        cov = list(msg.linear_acceleration_covariance)
        cov[0] = 0.005
        cov[4] = 0.005
        cov[8] = 0.005
        msg.linear_acceleration_covariance = cov
        #msg.orientation_covariance = cov
        #msg.angular_velocity_covariance[0] = 0.00009
        #msg.angular_velocity_covariance[4] = 0.00009
        #msg.angular_velocity_covariance[8] = 0.00009
        cov = list(msg.angular_velocity_covariance)
        cov[0] = 0.00009
        cov[4] = 0.00009
        cov[8] = 0.00009
        msg.angular_velocity_covariance = cov
        self.pub.publish(msg)


rospy.init_node("imu_publisher")

ir = ImuRepublisher()
sub = rospy.Subscriber("/uavcan_imu", Imu, ir.imu_callback)

rospy.spin()


