#!/usr/bin/python3

import rospy
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseWithCovarianceStamped


class TargetInitializer:
    def __init__(self):
        # Target's initial position [m].
        self.tgt_prior_x = rospy.get_param("~prior_tgt_x")
        self.tgt_prior_y = rospy.get_param("~prior_tgt_y")
        self.tgt_prior_z = rospy.get_param("~prior_tgt_z")

        # Target's initial orientation [rad].
        self.tgt_prior_rpy = [
            rospy.get_param("~prior_tgt_roll"),
            rospy.get_param("~prior_tgt_pitch"),
            rospy.get_param("~prior_tgt_yaw"),
        ]

        # Chaser's initial position [m].
        self.chaser_prior_x = rospy.get_param("~prior_chaser_x")
        self.chaser_prior_y = rospy.get_param("~prior_chaser_y")
        self.chaser_prior_z = rospy.get_param("~prior_chaser_z")

        # Target's initial orientation [rad].
        self.chaser_prior_rpy = [
            rospy.get_param("~prior_chaser_roll"),
            rospy.get_param("~prior_chaser_pitch"),
            rospy.get_param("~prior_chaser_yaw"),
        ]

        cov_string = rospy.get_param("~init_covariance")
        cov_string = cov_string.replace("[", "")
        cov_string = cov_string.replace("]", "")
        cov_list = list(cov_string.split(", "))
        self.init_cov = list(map(float, cov_list))

        rospy.logwarn(f"Estim init received: {self.init_cov}")

        # base frame name.
        self.tgt_frame = rospy.get_param("~target_link_id")
        self.chaser_frame = rospy.get_param("~chaser_link_id")

        # Target's map frame name.
        self.map_frame = rospy.get_param("~map_frame_id")

        # Publishers.
        self.tgt_pose_pub = rospy.Publisher(
            rospy.get_param("~target_initial_topic"),
            PoseWithCovarianceStamped,
            queue_size=1,
            latch=True,
        )

        self.chaser_pose_pub = rospy.Publisher(
            rospy.get_param("~chaser_initial_topic"),
            PoseWithCovarianceStamped,
            queue_size=1,
            latch=True,
        )

        # TF service.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def publish_initial_pose(self):
        noise = self.add_noise(self.init_cov)

        # Target's initial pose.
        tgt_rpy_noise = [
            self.tgt_prior_rpy[0] + noise[9],
            self.tgt_prior_rpy[1] + noise[10],
            self.tgt_prior_rpy[2] + noise[11],
        ]

        tgt_rot = Rotation.from_euler("xyz", tgt_rpy_noise)
        tgt_quat = tgt_rot.as_quat()

        # Build pose message.
        tgt_pose_msg = PoseWithCovarianceStamped()
        tgt_pose_msg.header.stamp = rospy.Time.now()
        tgt_pose_msg.header.frame_id = self.map_frame
        tgt_pose_msg.pose.pose.position.x = self.tgt_prior_x + noise[6]
        tgt_pose_msg.pose.pose.position.y = self.tgt_prior_y + noise[7]
        tgt_pose_msg.pose.pose.position.z = self.tgt_prior_z + noise[8]
        tgt_pose_msg.pose.pose.orientation.x = tgt_quat[0]
        tgt_pose_msg.pose.pose.orientation.y = tgt_quat[1]
        tgt_pose_msg.pose.pose.orientation.z = tgt_quat[2]
        tgt_pose_msg.pose.pose.orientation.w = tgt_quat[3]

        # Add the diagonal covariance.
        tgt_pose_msg.pose.covariance[0] = self.init_cov[0]
        tgt_pose_msg.pose.covariance[7] = self.init_cov[1]
        tgt_pose_msg.pose.covariance[14] = self.init_cov[2]
        tgt_pose_msg.pose.covariance[21] = self.init_cov[3]
        tgt_pose_msg.pose.covariance[28] = self.init_cov[4]
        tgt_pose_msg.pose.covariance[35] = self.init_cov[5]


        # Chaser's initial pose.
        chaser_rpy_noise = [
            self.chaser_prior_rpy[0] + noise[3],
            self.chaser_prior_rpy[1] + noise[4],
            self.chaser_prior_rpy[2] + noise[5],
        ]

        chaser_rot = Rotation.from_euler("xyz", chaser_rpy_noise)
        chaser_quat = chaser_rot.as_quat()

        # Build pose message.
        chaser_pose_msg = PoseWithCovarianceStamped()
        chaser_pose_msg.header.stamp = rospy.Time.now()
        chaser_pose_msg.header.frame_id = self.map_frame
        chaser_pose_msg.pose.pose.position.x = self.tgt_prior_x + noise[0]
        chaser_pose_msg.pose.pose.position.y = self.tgt_prior_y + noise[1]
        chaser_pose_msg.pose.pose.position.z = self.tgt_prior_z + noise[2]
        chaser_pose_msg.pose.pose.orientation.x = chaser_quat[0]
        chaser_pose_msg.pose.pose.orientation.y = chaser_quat[1]
        chaser_pose_msg.pose.pose.orientation.z = chaser_quat[2]
        chaser_pose_msg.pose.pose.orientation.w = chaser_quat[3]

        # Add the diagonal covariance.
        chaser_pose_msg.pose.covariance[0] = self.init_cov[0]
        chaser_pose_msg.pose.covariance[7] = self.init_cov[1]
        chaser_pose_msg.pose.covariance[14] = self.init_cov[2]
        chaser_pose_msg.pose.covariance[21] = self.init_cov[3]
        chaser_pose_msg.pose.covariance[28] = self.init_cov[4]
        chaser_pose_msg.pose.covariance[35] = self.init_cov[5]

        # Publish initial poses.
        self.tgt_pose_pub.publish(tgt_pose_msg)
        self.chaser_pose_pub.publish(chaser_pose_msg)

        rospy.loginfo("(TargetInitializer): Published initial poses.")

    def add_noise(self, noise):
        """
        Add noise to poses
        """
        noise_cov = np.diag(noise)
        pose_noise = np.sqrt(noise_cov).dot(np.random.randn(12, 1)).T
        return pose_noise[0]


if __name__ == "__main__":
    """
    Run the initializer.
    """
    rospy.init_node("target_initializer")

    tgt_init = TargetInitializer()

    tgt_init.publish_initial_pose()

    while not rospy.is_shutdown():
        rospy.spin()
