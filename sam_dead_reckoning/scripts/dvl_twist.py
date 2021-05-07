#!/usr/bin/env python

import rospy
from smarc_msgs.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped

class DVLTwist(object):

    def __init__(self):
        self.dvl_topic = rospy.get_param('~dvl_topic', '/sam/core/dvl')
        self.twist_topic = rospy.get_param('~twist_topic', 'dvl_twist')
        self.dvl_frame = rospy.get_param('~dvl_link', 'sam/dvl_link')
        self.sub = rospy.Subscriber(self.dvl_topic, DVL, self.dvl_callback)
        self.pub = rospy.Publisher(self.twist_topic, TwistWithCovarianceStamped, queue_size=10)

    def dvl_callback(self, dvl_msg):

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header = dvl_msg.header
        twist_msg.twist.twist.linear = dvl_msg.velocity
        for i in range(0, 3):
            for j in range(0, 3):
                twist_msg.twist.covariance[6*i+j] = dvl_msg.velocity_covariance[3*i+j]

        self.pub.publish(twist_msg)

if __name__ == "__main__":
    rospy.init_node('dvl_twist')
    dvl_twist = DVLTwist()
    rospy.spin()
