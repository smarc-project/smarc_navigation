#!/usr/bin/python

import rospy
import numpy as np
import tf
from cola2_msgs.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped
import tf

class DVL2Twist(object):

	def __init__(self):
            self.dvl_topic = rospy.get_param('~dvl_topic', 'dvl')
            self.dvl_twist_topic = rospy.get_param('~dvl_twist_topic', 'sam')
            self.subs = rospy.Subscriber(self.dvl_topic, DVL, self.dvlCB)
            self.pub = rospy.Publisher(self.dvl_twist_topic, TwistWithCovarianceStamped, queue_size=10)
            
            rospy.spin()


	def dvlCB(self, dvl_msg):
            twist_msg = TwistWithCovarianceStamped()
            twist_msg.header = dvl_msg.header
            twist_msg.header.frame_id = "sam/dvl_link"
            twist_msg.twist.twist.linear = dvl_msg.velocity
            for i in range(0,3):
                for j in range(0,3):
                    twist_msg.twist.covariance[i*6+j] = dvl_msg.velocity_covariance[i*3+j]
            
            self.pub.publish(twist_msg)



if __name__ == "__main__":
    rospy.init_node('dvl_to_twist')
    try:
        pi = DVL2Twist()
    except rospy.ROSInterruptException:
        pass
