#!/usr/bin/env python3
"""
SLAM-based particle filter for underwater docking.
"""

import rospy
import numpy as np
import tf2_ros
import tf
from tf import transformations

from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion, Point
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from slam_particle import Particle, matrix_from_tf
from resampling import residual_resample
from numpy import linalg as LA


class SlamParticleFilter(object):
    """
    Particle filter for SLAM
    """

    def __init__(self):
        # Read necessary parameters
        self.particle_count = rospy.get_param('~particle_count', 10)
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ds_base_frame = rospy.get_param('~ds_base_frame', 'ds_base_link')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera')
        self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')
        self.base_frame_2d = rospy.get_param('~base_frame_2d', 'sam/base_link')
        self.ds_range = rospy.get_param('~ds_range', 10)

        # Initialize tf listener
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)
        self.listener = tf.TransformListener()

        # Read covariance values
        meas_std = float(rospy.get_param('~measurement_std', 0.01))
        cov_string = rospy.get_param('~motion_covariance')
        cov_string = cov_string.replace('[','')
        cov_string = cov_string.replace(']','')
        cov_list = list(cov_string.split(", "))
        motion_cov = list(map(float, cov_list))

        cov_string = rospy.get_param('~init_covariance')
        cov_string = cov_string.replace('[','')
        cov_string = cov_string.replace(']','')
        cov_list = list(cov_string.split(", "))
        init_cov = list(map(float, cov_list))

        cov_string = rospy.get_param('~resampling_noise_covariance')
        cov_string = cov_string.replace('[','')
        cov_string = cov_string.replace(']','')
        cov_list = list(cov_string.split(", "))
        self.res_noise_cov = list(map(float, cov_list))

        # Initialize particle poses publisher
        self.sam_poses = PoseArray()
        self.sam_poses.header.frame_id = self.odom_frame
        sam_pose_array_topic = rospy.get_param("~sam_particle_poses_topic", '/sam_particle_poses')
        self.sam_pose_array_pub = rospy.Publisher(sam_pose_array_topic, PoseArray, queue_size=10)

        self.ds_poses = PoseArray()
        self.ds_poses.header.frame_id = self.base_frame
        ds_pose_array_top = rospy.get_param("~ds_particle_poses_topic", '/ds_particle_poses')
        self.ds_pose_array_pub = rospy.Publisher(ds_pose_array_top, PoseArray, queue_size=10)

        # Initialize localization odom publisher
        # SAM Odom
        self.sam_localization_pose = Odometry()
        self.sam_localization_pose.header.frame_id = self.odom_frame
        self.sam_localization_pose.child_frame_id = self.base_frame
        sam_localization_topic = rospy.get_param("~odom_corrected_topic", '/average_pose')
        self.sam_localization_pub = rospy.Publisher(sam_localization_topic,
                                                    Odometry, queue_size=100)

        self.sam_localization_tf = tf.TransformBroadcaster()

        # Docking Station in sam/base_link
        self.ds_localization_pose = Odometry()
        self.ds_localization_pose.header.frame_id = self.base_frame
        self.ds_localization_pose.child_frame_id = self.ds_base_frame
        ds_localization_topic = rospy.get_param("~ds_corrected_topic", '/ds_pose')
        self.ds_localization_pub = rospy.Publisher(ds_localization_topic, Odometry, queue_size=100)

        self.ds_localization_tf = tf.TransformBroadcaster()

        # Docking Station in odom
        self.ds_odom_pose = Odometry()
        self.ds_odom_pose.header.frame_id = self.odom_frame
        self.ds_odom_pose.child_frame_id = "ds_odom_frame"
        self.ds_odom_tf = tf.TransformBroadcaster()

        # Transforms from auv_2_ros
        try:
            rospy.loginfo("Waiting for transforms")
            map2odom_tf = tf_buffer.lookup_transform(self.map_frame, self.odom_frame,
                                               rospy.Time(0), rospy.Duration(60))
            self.map2odom_mat = matrix_from_tf(map2odom_tf)
            rospy.loginfo("PF: got transform %s to %s" % (self.map_frame, self.odom_frame))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("PF: Could not lookup transform %s to %s" 
                         % (self.map_frame, self.odom_frame))
            return

        # Initialize list of particles
        self.particles = np.empty(self.particle_count, dtype=object)

        # Uniform initial sampling over the range
        ds_distribution = np.random.uniform([-self.ds_range, -self.ds_range],
                                            [self.ds_range, self.ds_range],
                                            size=(self.particle_count, 2))

        for i in range(self.particle_count):
            self.particles[i] = Particle(self.particle_count, i, self.map2odom_mat,
                                         init_ds = ds_distribution[i],
                                         init_cov=init_cov, meas_std=meas_std,
                                         process_cov=motion_cov)

        # Start timing now
        self.time = rospy.Time.now().to_sec()
        self.old_time = rospy.Time.now().to_sec()

        # Perception topic
        perception_topic = rospy.get_param('~perception_topic', '/perception')
        rospy.Subscriber(perception_topic, PoseWithCovarianceStamped,
                         self.perception_cb, queue_size=100)

        # Establish subscription to odometry message (intentionally last)
        odom_top = rospy.get_param("~odom_topic", 'odom')
        rospy.Subscriber(odom_top, Odometry, self.odom_cb, queue_size=100)

        # PF pub and broadcaster loop
        rospy.Timer(rospy.Duration(0.1), self.localization_loop)

        # PF filter created. Start auv_2_ros survey playing
        rospy.loginfo("Particle filter class successfully created")

        rospy.spin()


    def perception_cb(self, docking_station_pose):
        """
        Compute new particle weights based on perception input
        Resample particles based on new weights.
        """
        # Meas update
        weights = self.update(docking_station_pose)

        # Particle resampling
        self.resample(weights)


    def update(self, docking_station_pose):
        """
        Update individual particle weights
        """
        weights = []
        for i in range(0, self.particle_count):
            # Compute particle weight
            self.particles[i].compute_weight(docking_station_pose)
            weights.append(self.particles[i].w)

        weights_array = np.asarray(weights)
        # Add small non-zero value to avoid hitting zero
        weights_array += 1.e-200

        return weights_array


    def resample(self, weights):
        """
        Resample particles with new weights
        """
        # Normalize weights
        weights /= weights.sum()

        indices = residual_resample(weights)
        keep = list(set(indices))
        lost = [i for i in range(self.particle_count) if i not in keep]
        dupes = indices[:].tolist()
        for i in keep:
            dupes.remove(i)

        self.reassign_poses(lost, dupes)

        # Add noise to particles
        for i in range(self.particle_count):
            self.particles[i].add_noise(self.res_noise_cov)


    def reassign_poses(self, lost, dupes):
        """
        Reasign the poses, faster than deepcopy()
        """
        for l, d in zip(lost, dupes):
            # Faster to do separately than using deepcopy()
            self.particles[l].p_pose = self.particles[d].p_pose


    def odom_cb(self, odom_msg):
        """
        Odometry message callback to invoke the prediction step
        """
        self.time = odom_msg.header.stamp.to_sec()

        if self.old_time and self.time > self.old_time:
            # Motion prediction
            self.predict(odom_msg)

        self.old_time = self.time


    def predict(self, odom_t):
        """
        Predict motion based on motion model
        """
        dt = self.time - self.old_time
        for i in range(0, self.particle_count):
            self.particles[i].motion_pred(odom_t, dt)


    def localization_loop(self, event):
        """
        Main localization loop runs at every timestep to broadcast current
        estimated pose
        """
        sam_pose_list, ds_pose_list = self.convert_poses()

        self.update_localization_poses(sam_pose_list, ds_pose_list)

        self.broadcast_transforms()

        self.publish_poses()


    def convert_poses(self):
        """
        Extract SAM's and DS's pose from the particle's state and
        put in the corresponding array for later use. 
        """
        self.sam_poses.poses = []
        self.ds_poses.poses = []

        sam_pose_list = []
        ds_pose_list = []

        for i in range(self.particle_count):
            sam_pose_i = Pose()
            sam_pose_i.position.x = self.particles[i].p_pose[0]
            sam_pose_i.position.y = self.particles[i].p_pose[1]
            sam_pose_i.position.z = self.particles[i].p_pose[2]
            sam_pose_i.orientation = Quaternion(*quaternion_from_euler(
                self.particles[i].p_pose[3],
                self.particles[i].p_pose[4],
                self.particles[i].p_pose[5]))

            ds_pose_i = Pose()
            ds_pose_i.position.x = self.particles[i].p_pose[6]
            ds_pose_i.position.y = self.particles[i].p_pose[7]
            ds_pose_i.position.z = self.particles[i].p_pose[8]
            ds_pose_i.orientation = Quaternion(*quaternion_from_euler(
                self.particles[i].p_pose[9],
                self.particles[i].p_pose[10],
                self.particles[i].p_pose[11]))

            self.sam_poses.poses.append(sam_pose_i)
            self.ds_poses.poses.append(ds_pose_i)

            sam_pose_list.append(self.particles[i].p_pose[0:6])
            ds_pose_list.append(self.particles[i].p_pose[6:12])

        self.sam_poses.header.stamp = rospy.Time.now()
        self.ds_poses.header.stamp = rospy.Time.now()

        return sam_pose_list, ds_pose_list


    def update_localization_poses(self, sam_pose_list, ds_pose_list):
        """
        Update poses based on the average of the respective particles.
        """
        # Update SAM pose based on particles
        sam_poses_array = np.array(sam_pose_list)

        self.sam_localization_pose.pose.pose.position = self.average_pose_position(sam_poses_array)
        self.sam_localization_pose.pose.pose.orientation = self.average_pose_orientation(sam_poses_array)
        self.sam_localization_pose.header.stamp = rospy.Time.now()
        self.sam_localization_pose.pose.covariance = self.calculate_covariance(sam_poses_array,
                                                                               self.sam_localization_pose.pose.pose.position)

        # Update docking station pose based on particles
        ds_poses_array = np.array(ds_pose_list)

        self.ds_localization_pose.pose.pose.position = self.average_pose_position(ds_poses_array)
        self.ds_localization_pose.pose.pose.orientation = self.average_pose_orientation(ds_poses_array)
        self.ds_localization_pose.header.stamp = rospy.Time.now()
        self.ds_localization_pose.pose.covariance = self.calculate_covariance(ds_poses_array,
                                                                                self.ds_localization_pose.pose.pose.position)


    def average_pose_position(self, poses_array):
        """
        Average position over all particles
        """
        ave_pose = poses_array.mean(axis = 0)

        pose_point = Point(*ave_pose[0:3])

        return pose_point


    def average_pose_orientation(self, poses_array):
        """
        Avg of orientations as eigenvector with largest eigenvalue of Q*Qt
        """
        ors_quat = []
        for i in range(poses_array[:, 3:6].shape[0]):
            ors_quat.append(quaternion_from_euler(
                poses_array[:, 3:6][i, 0], poses_array[:, 3:6][i, 1], poses_array[:, 3:6][i, 2]))

        Q_T = np.asarray(ors_quat)
        Q_quat = np.matmul(Q_T.T, Q_T)
        w, v, = LA.eig(Q_quat)
        l_v = v[:, np.argmax(w)]

        quat_l_v = Quaternion(*l_v)

        return quat_l_v


    def calculate_covariance(self, poses_array, ave_pose):
        """
        Calculate the covariance of a pose
        """
        cov = np.zeros((3, 3))
        for i in range(self.particle_count):
            dx = (poses_array[i, 0:3] - [ave_pose.x, ave_pose.y, ave_pose.z])
            cov += np.diag(dx*dx.T)
            cov[0,1] += dx[0]*dx[1]
            cov[0,2] += dx[0]*dx[2]
            cov[1,2] += dx[1]*dx[2]
        cov /= self.particle_count
        cov[1,0] = cov[0,1]

        cov_array = [0.]*36
        for i in range(3):
            for j in range(3):
                cov_array[i*3 + j] = cov[i,j]

        return cov_array


    def broadcast_transforms(self):
        """
        Broadcast all poses to the TF tree
        """
        # Wrangle poses
        sam_ave_pose = [self.sam_localization_pose.pose.pose.position.x,
                        self.sam_localization_pose.pose.pose.position.y,
                        self.sam_localization_pose.pose.pose.position.z]
        sam_l_v = [self.sam_localization_pose.pose.pose.orientation.x,
                   self.sam_localization_pose.pose.pose.orientation.y,
                   self.sam_localization_pose.pose.pose.orientation.z,
                   self.sam_localization_pose.pose.pose.orientation.w]

        ds_ave_pose = [self.ds_localization_pose.pose.pose.position.x,
                        self.ds_localization_pose.pose.pose.position.y,
                        self.ds_localization_pose.pose.pose.position.z]
        ds_l_v = [self.ds_localization_pose.pose.pose.orientation.x,
                   self.ds_localization_pose.pose.pose.orientation.y,
                   self.ds_localization_pose.pose.pose.orientation.z,
                   self.ds_localization_pose.pose.pose.orientation.w]

        # Broadcast TFs
        # Why is z = 0?
        # What is the base_frame_2d?
        self.sam_localization_tf.sendTransform([sam_ave_pose[0], sam_ave_pose[1], 0.],
                                    sam_l_v,
                                    rospy.Time.now(),
                                    self.base_frame,
                                    self.odom_frame)

        euler = euler_from_quaternion(sam_l_v)
        quat_t = quaternion_from_euler(0., 0., euler[2])
        self.sam_localization_tf.sendTransform([sam_ave_pose[0], sam_ave_pose[1], 0.],
                                    quat_t,
                                    rospy.Time.now(),
                                    self.base_frame_2d,
                                    self.odom_frame)

        self.ds_localization_tf.sendTransform([ds_ave_pose[0], ds_ave_pose[1], 0.],
                                              ds_l_v,
                                              rospy.Time.now(),
                                              self.ds_base_frame,
                                              self.base_frame)

        # Transform into odom frame for the TF odom --> docking station
        R_sam_odom = transformations.quaternion_matrix(sam_l_v)
        R_ds_base = transformations.quaternion_matrix(ds_l_v)

        R_ds_odom = np.matmul(R_sam_odom,R_ds_base)

        quat_ds_odom = transformations.quaternion_from_matrix(R_ds_odom)

        t_ds_odom = [0.]*3
        t_ds_odom[0] = sam_ave_pose[0] + ds_ave_pose[0]
        t_ds_odom[1] = sam_ave_pose[1] + ds_ave_pose[1]
        t_ds_odom[2] = sam_ave_pose[2] + ds_ave_pose[2]

        self.ds_odom_tf.sendTransform(t_ds_odom,
                                    quat_ds_odom,
                                    rospy.Time.now(),
                                    self.ds_odom_pose.child_frame_id,
                                    self.odom_frame)


    def publish_poses(self):
        """
        Publish everything to the corresponding topic
        """
        # Publish particles as arrows
        self.sam_pose_array_pub.publish(self.sam_poses)
        self.ds_pose_array_pub.publish(self.ds_poses)

        # Publish average estimated pose
        self.sam_localization_pub.publish(self.sam_localization_pose)
        self.ds_localization_pub.publish(self.ds_localization_pose)


if __name__ == '__main__':

    rospy.init_node('slamParticleFilter', disable_signals=False)
    try:
        SlamParticleFilter()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch pf")
