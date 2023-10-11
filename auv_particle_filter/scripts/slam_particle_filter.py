#!/usr/bin/env python3
"""
SLAM-based particle filter for underwater docking.
"""

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy as np

import tf2_ros
import tf

from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion, Point
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf.transformations import translation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, quaternion_from_matrix

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
        self.ds_range = rospy.get_param('~ds_range', 10)

        # Frames
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ds_base_frame = rospy.get_param('~ds_base_frame', 'ds_base_link')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera')
        self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')
        self.base_frame_2d = rospy.get_param('~base_frame_2d', 'sam/base_link')

        # Initialize tf listener
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        # self.listener = tf.TransformListener()

        # Read covariance values
        self.meas_std = float(rospy.get_param('~measurement_std', 0.01))
        cov_string = rospy.get_param('~motion_covariance')
        cov_string = cov_string.replace('[','')
        cov_string = cov_string.replace(']','')
        cov_list = list(cov_string.split(", "))
        self.motion_cov = list(map(float, cov_list))

        cov_string = rospy.get_param('~init_covariance')
        cov_string = cov_string.replace('[','')
        cov_string = cov_string.replace(']','')
        cov_list = list(cov_string.split(", "))
        self.init_cov = list(map(float, cov_list))

        cov_string = rospy.get_param('~resampling_noise_covariance')
        cov_string = cov_string.replace('[','')
        cov_string = cov_string.replace(']','')
        cov_list = list(cov_string.split(", "))
        self.res_noise_cov = list(map(float, cov_list))

        # SAM Odom from DR
        self.sam_odom = Odometry()

        # Topics
        sam_pose_array_topic = rospy.get_param("~sam_particle_poses_topic", '/sam_particle_poses')
        ds_pose_array_top = rospy.get_param("~ds_particle_poses_topic", '/ds_particle_poses')
        sam_localization_topic = rospy.get_param("~odom_corrected_topic", '/average_pose')
        ds_localization_topic = rospy.get_param("~ds_corrected_topic", '/ds_pose')
        sam_map_topic = rospy.get_param("~sam_map_topic", '/sam_map')
        ds_map_topic = rospy.get_param("~ds_map_topic", '/ds_map')
        perception_topic = rospy.get_param('~perception_topic', '/perception')
        odom_top = rospy.get_param("~odom_topic", 'odom')
        ds_init_topic = rospy.get_param('~ds_init_estimate')
        weight_topic = rospy.get_param('~particle_weight_topic')
        sam_particle_topic = rospy.get_param('~sam_particle_topic')

        # Initialize particle poses publisher
        self.sam_poses = PoseArray()
        self.sam_poses.header.frame_id = self.odom_frame
        self.sam_pose_array_pub = rospy.Publisher(sam_pose_array_topic, PoseArray, queue_size=10)

        self.ds_poses = PoseArray()
        self.ds_poses.header.frame_id = self.odom_frame
        self.ds_pose_array_pub = rospy.Publisher(ds_pose_array_top, PoseArray, queue_size=10)

        # Initialize localization odom publisher
        # SAM Odom
        self.sam_localization_pose = Odometry()
        self.sam_localization_pose.header.frame_id = self.odom_frame
        self.sam_localization_pose.child_frame_id = self.base_frame
        self.sam_localization_pub = rospy.Publisher(sam_localization_topic,
                                                    Odometry, queue_size=10)

        # FIXME: Update that to TF2
        self.sam_localization_tf = tf.TransformBroadcaster()

        # Docking Station in sam/base_link
        self.ds_localization_pose = Odometry()
        self.ds_localization_pose.header.frame_id = self.odom_frame
        self.ds_localization_pose.child_frame_id = self.ds_base_frame
        self.ds_localization_pub = rospy.Publisher(ds_localization_topic, Odometry, queue_size=100)

        self.ds_localization_tf = tf.TransformBroadcaster()

        self.docking_station_pose_perception = PoseWithCovarianceStamped()
        self.ds_init_prior = np.zeros(6)

        # Weight publisher
        self.particle_weights = np.empty(self.particle_count)
        self.particle_weight_pub = rospy.Publisher(weight_topic, numpy_msg(Floats), queue_size=10)

        # Particle pose array pubplisher
        self.sam_poses_array = np.empty((self.particle_count, 6))
        self.sam_particle_pub = rospy.Publisher(sam_particle_topic, numpy_msg(Floats), queue_size=10)


        # Transforms from auv_2_ros
        # This is actually map --> odom! The documentation is weird.
        try:
            rospy.loginfo("Waiting for transforms")
            map_to_odom_tf = self.tf_buffer.lookup_transform(self.map_frame, self.odom_frame,
                                               rospy.Time(0), rospy.Duration(60))
            self.map_to_odom_mat = matrix_from_tf(map_to_odom_tf)
            rospy.loginfo("PF: got transform %s to %s" % (self.map_frame, self.odom_frame))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("PF: Could not lookup transform %s to %s" 
                         % (self.map_frame, self.odom_frame))
            return

        # Docking station prior topic
        rospy.Subscriber(ds_init_topic, PoseWithCovarianceStamped,
                         self.docking_station_prior_cb, queue_size=1)
        
        # Initialize list of particles
        self.particles = np.empty(self.particle_count, dtype=object)
        self.particles_initialised = False
        self.sam_initialised = False

        # First initialization to get odom->sam tf. Then particles will be reinitialized
        for i in range(self.particle_count):
            self.particles[i] = Particle(self.particle_count, i, self.map_to_odom_mat,
                                        init_sam = [0.0] * 6,
                                        init_ds = self.ds_init_prior,
                                        init_cov=self.init_cov, meas_std=self.meas_std,
                                        process_cov=self.motion_cov)

        # Start timing now
        self.time = rospy.Time.now().to_sec()
        self.old_time = rospy.Time.now().to_sec()
        self.last_cb_time = rospy.Time.now().to_sec()

        # Perception topic
        rospy.Subscriber(perception_topic, PoseWithCovarianceStamped,
                         self.perception_cb, queue_size=5)

        # Establish subscription to odometry message (intentionally last)
        rospy.Subscriber(odom_top, Odometry, self.odom_cb, queue_size=100)

        # PF pub and broadcaster loop
        rospy.Timer(rospy.Duration(0.1), self.localization_loop)

        # PF filter created
        rospy.loginfo("Particle filter class successfully created")

        rospy.spin()


    def docking_station_prior_cb(self, docking_station_prior):
        """
        Getting the initial prior esitmate of the docking station.
        Due to some some frame issue when developing the PF, we
        need to convert the initial estimate the same way we have
        to transform the map to odom frame in the launch file.
        """
        t_ds_prior = [docking_station_prior.pose.pose.position.x,
                      docking_station_prior.pose.pose.position.y,
                      docking_station_prior.pose.pose.position.z]
        quat_ds_prior = [docking_station_prior.pose.pose.orientation.x,
                      docking_station_prior.pose.pose.orientation.y,
                      docking_station_prior.pose.pose.orientation.z,
                      docking_station_prior.pose.pose.orientation.w]

        T_ds_prior = translation_matrix(t_ds_prior)
        R_ds_prior = quaternion_matrix(quat_ds_prior)
        M_ds_prior = np.matmul(T_ds_prior, R_ds_prior)

        M_ds_prior_odom = np.matmul(self.map_to_odom_mat, M_ds_prior)

        t_ds_prior_odom = M_ds_prior_odom[0:3,3]
        quat_ds_prior_odom = quaternion_from_matrix(M_ds_prior_odom)
        rpy_ds_prior_odom = euler_from_quaternion(quat_ds_prior_odom)

        self.ds_init_prior[0:3] = t_ds_prior_odom
        self.ds_init_prior[3:6] = rpy_ds_prior_odom

        # First initialization to get odom->sam tf. Then particles will be reinitialized
        for i in range(self.particle_count):
            self.particles[i] = Particle(self.particle_count, i, self.map_to_odom_mat,
                                        init_sam = [0.0] * 6,
                                        init_ds = self.ds_init_prior,
                                        init_cov=self.init_cov, meas_std=self.meas_std,
                                        process_cov=self.motion_cov)
        print("DS Estimate initialized with prior: {}".format(self.ds_init_prior))



    def perception_cb(self, docking_station_pose):
        """
        Compute new particle weights based on perception input
        Resample particles based on new weights.
        TODO: Once we see the docking station again after loosing it, 
        reinitialize the particles based on the new position.
        This is a reasonable assumption when we only care about the
        relative position and not the absolute one. For the later we
        could do localization and then update SAM's position instead.
        But that's a different problem.
        """
        self.current_cb_time = rospy.Time.now().to_sec()
        if not self.particles_initialised: # or self.current_cb_time - self.last_cb_time > 4:
            cb_time = self.current_cb_time - self.last_cb_time
            print("Reinitialize")
            print("Flag: {}, time diff: {}".format(self.particles_initialised, cb_time))

            # SAM position in odom frame:
            init_sam = np.zeros(6)
            t_sam = [self.sam_odom.pose.pose.position.x,
                     self.sam_odom.pose.pose.position.y,
                     self.sam_odom.pose.pose.position.z]
            quat_sam = [self.sam_odom.pose.pose.orientation.x,
                        self.sam_odom.pose.pose.orientation.y,
                        self.sam_odom.pose.pose.orientation.z,
                        self.sam_odom.pose.pose.orientation.w]
            rpy_sam = euler_from_quaternion(quat_sam)
            init_sam[0:3] = t_sam
            init_sam[3:6] = rpy_sam

            # Docking station perception:
            # this is the relative measurement to SAM, thus we need to add the distance to the
            # odom frame to these measurements.
            init_ds = np.zeros(6)
            t_docking_station = [docking_station_pose.pose.pose.position.x,
                                docking_station_pose.pose.pose.position.y,
                                docking_station_pose.pose.pose.position.z]
            quat_docking_station = [docking_station_pose.pose.pose.orientation.x,
                                docking_station_pose.pose.pose.orientation.y,
                                docking_station_pose.pose.pose.orientation.z,
                                docking_station_pose.pose.pose.orientation.w]

            ## Appending docking station position to SAM position
            # Build transformation matrix for SAM
            T_sam = translation_matrix(t_sam)
            R_sam = quaternion_matrix(quat_sam)
            M_sam = np.matmul(T_sam, R_sam)

            # Build transformation matrix for DS from perception
            T_docking_station_perception = translation_matrix(t_docking_station)
            R_docking_station_perception = quaternion_matrix(quat_docking_station)
            M_docking_station_perception = np.matmul(T_docking_station_perception, R_docking_station_perception)

            # Docking station wrt odom frame
            M_docking_station_odom = np.matmul(M_sam, M_docking_station_perception)

            t_docking_station_odom = translation_from_matrix(M_docking_station_odom)
            quat_docking_station_odom = quaternion_from_matrix(M_docking_station_odom)
            rpy_docking_station_odom = euler_from_quaternion(quat_docking_station_odom)

            init_ds[0:3] = t_docking_station_odom
            init_ds[3:6] = rpy_docking_station_odom

            print("Init Particles: {}".format(init_ds))

            for i in range(self.particle_count):
                self.particles[i] = Particle(self.particle_count, i, self.map_to_odom_mat,
                                            init_sam = init_sam,
                                            init_ds = init_ds,
                                            init_cov=self.init_cov, meas_std=self.meas_std,
                                            process_cov=self.motion_cov)

            self.particles_initialised = True
            self.sam_initialised = True
        else:
            # Meas update
            weights = self.update(docking_station_pose)
            self.particle_weights = np.array(weights, dtype=np.float32)

            # Particle resampling
            self.resample(weights)

            self.docking_station_pose_perception = docking_station_pose

        self.last_cb_time = self.current_cb_time


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
        Reasign the poses, separately faster than deepcopy()
        """
        for l, d in zip(lost, dupes):
            self.particles[l].p_pose = self.particles[d].p_pose


    def odom_cb(self, odom_msg):
        """
        Odometry message callback to invoke the prediction step
        """
        self.sam_odom = odom_msg

        self.time = odom_msg.header.stamp.to_sec()

        if not self.sam_initialised:
            # SAM position in odom frame:
            init_sam = np.zeros(6)
            t_sam = [self.sam_odom.pose.pose.position.x,
                     self.sam_odom.pose.pose.position.y,
                     self.sam_odom.pose.pose.position.z]
            quat_sam = [self.sam_odom.pose.pose.orientation.x,
                        self.sam_odom.pose.pose.orientation.y,
                        self.sam_odom.pose.pose.orientation.z,
                        self.sam_odom.pose.pose.orientation.w]
            rpy_sam = euler_from_quaternion(quat_sam)
            init_sam[0:3] = t_sam
            init_sam[3:6] = rpy_sam
            # First initialization to get odom->sam tf. Then particles will be reinitialized
            for i in range(self.particle_count):
                self.particles[i] = Particle(self.particle_count, i, self.map_to_odom_mat,
                                            init_sam = init_sam,
                                            init_ds = self.ds_init_prior,
                                            init_cov=self.init_cov, meas_std=self.meas_std,
                                            process_cov=self.motion_cov)
            self.sam_initialised = True

        # if self.particles_initialised:
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
        # if self.particles_initialised:
        sam_pose_list, ds_pose_list = self.convert_poses()

        self.update_localization_poses(sam_pose_list, ds_pose_list)

        self.broadcast_transforms()

        self.publish_poses()

        # self.print_states()


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
        self.sam_poses_array = np.array(sam_pose_list, dtype=np.float32)

        self.sam_localization_pose.pose.pose.position = self.average_pose_position(self.sam_poses_array)
        self.sam_localization_pose.pose.pose.orientation = self.average_pose_orientation(self.sam_poses_array)
        self.sam_localization_pose.header.stamp = rospy.Time.now()
        self.sam_localization_pose.pose.covariance = self.calculate_covariance(self.sam_poses_array,
                                                                               self.sam_localization_pose.pose.pose)

        # Update docking station pose based on particles
        ds_poses_array = np.array(ds_pose_list)

        self.ds_localization_pose.pose.pose.position = self.average_pose_position(ds_poses_array)
        self.ds_localization_pose.pose.pose.orientation = self.average_pose_orientation(ds_poses_array)
        self.ds_localization_pose.header.stamp = rospy.Time.now()
        self.ds_localization_pose.pose.covariance = self.calculate_covariance(ds_poses_array,
                                                                                self.ds_localization_pose.pose.pose)


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
        Calculate the covariance of a pose.
        Computing standard deviation (dx), then squaring them for 
        diagonal elements and multiply the off-diagonal elements.
        Then normalize over the number of particles.
        Note, it could be that this needs to be changed to a proper
        Rao Blackwellized calculation because of the multivariate
        nature of the poses.
        """
        cov = np.zeros((6, 6))
        pose_ave = np.zeros(6)
        pose_ave[0:3] = [ave_pose.position.x, ave_pose.position.y, ave_pose.position.z]
        pose_ave[3:6] = euler_from_quaternion([ave_pose.orientation.x,
                                               ave_pose.orientation.y,
                                               ave_pose.orientation.z,
                                               ave_pose.orientation.w])

        for i in range(self.particle_count):
            pose_particle = poses_array[i, 0:6]

            dx = pose_particle - pose_ave

            for ii in range(0,6):
                for jj in range(0,6):
                    cov[ii,jj] += dx[ii] * dx[jj]

        cov /= self.particle_count

        cov_array = np.reshape(cov,36)

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
        sam_l_v /= np.linalg.norm(sam_l_v)

        ds_ave_pose = [self.ds_localization_pose.pose.pose.position.x,
                        self.ds_localization_pose.pose.pose.position.y,
                        self.ds_localization_pose.pose.pose.position.z]
        ds_l_v = [self.ds_localization_pose.pose.pose.orientation.x,
                   self.ds_localization_pose.pose.pose.orientation.y,
                   self.ds_localization_pose.pose.pose.orientation.z,
                   self.ds_localization_pose.pose.pose.orientation.w]
        ds_l_v /= np.linalg.norm(ds_l_v)

        # check if you normalize the quaternions. That could lead to the error
        # if np.linalg.norm(sam_l_v) > 1:
        #     print("Norm sam_l_v: {}".format(np.linalg.norm(sam_l_v)))

        # if np.linalg.norm(ds_l_v) > 1:
        #     print("Norm ds_l_v: {}".format(np.linalg.norm(ds_l_v)))


        # Broadcast TFs
        # Why is z = 0? NOTE: Fixed that for the sam_localization_tf
        # What is the base_frame_2d?
        self.sam_localization_tf.sendTransform([sam_ave_pose[0], sam_ave_pose[1], sam_ave_pose[2]],
                                    sam_l_v,
                                    rospy.Time.now(),
                                    self.base_frame,
                                    self.odom_frame)

        # euler = euler_from_quaternion(sam_l_v)
        # quat_t = quaternion_from_euler(0., 0., euler[2])
        # # QUESTION: Can I send the two different TFs with the same TF broadcaster?
        # self.sam_localization_tf.sendTransform([sam_ave_pose[0], sam_ave_pose[1], 0.],
        #                             quat_t,
        #                             rospy.Time.now(),
        #                             self.base_frame_2d,
        #                             self.odom_frame)

        self.ds_localization_tf.sendTransform([ds_ave_pose[0], ds_ave_pose[1], ds_ave_pose[2]],
                                              ds_l_v,
                                              rospy.Time.now(),
                                              self.ds_base_frame,
                                              self.odom_frame)

        # Transform into odom frame for the TF odom --> docking station
        # # FIXME: Check that you did the TFs right, with the inversion of the tf matrix.
        # # See obsidian for the way to do it.
        # R_sam_odom = transformations.quaternion_matrix(sam_l_v)
        # R_ds_base = transformations.quaternion_matrix(ds_l_v)

        # R_ds_odom = np.matmul(R_sam_odom,R_ds_base)

        # quat_ds_odom = transformations.quaternion_from_matrix(R_ds_odom)

        # t_ds_odom = [0.]*3
        # t_ds_odom[0] = sam_ave_pose[0] + ds_ave_pose[0]
        # t_ds_odom[1] = sam_ave_pose[1] + ds_ave_pose[1]
        # t_ds_odom[2] = sam_ave_pose[2] + ds_ave_pose[2]

        # t_ds_odom_mat = transformations.translation_matrix(t_ds_odom)

        # odom_to_ds_mat = np.matmul(t_ds_odom_mat,R_ds_odom)     # Note the order of the arguments!

        # self.ds_odom_tf.sendTransform(t_ds_odom,
        #                             quat_ds_odom,
        #                             rospy.Time.now(),
        #                             self.ds_odom_pose.child_frame_id,
        #                             self.odom_frame)

        #FIXME: # Transform map --> docking station
        # odom_to_map_inv = self.odom_to_map_mat.T
        # odom_to_map_inv[3,0:3] = np.zeros(3)
        # t_odom_to_map_inv = -np.dot(self.odom_to_map_mat[0:3,0:3], self.odom_to_map_mat[0:3,3])
        # odom_to_map_inv[0:3,3] = t_odom_to_map_inv

        # map_to_ds_mat = np.matmul(odom_to_map_inv,odom_to_ds_mat)
        # t_map_to_ds = transformations.translation_from_matrix(map_to_ds_mat)
        # quat_map_to_ds = transformations.quaternion_from_matrix(map_to_ds_mat)

        # self.ds_map_pose.header.frame_id = self.odom_frame
        # self.ds_map_pose.header.stamp = rospy.Time.now()
        # self.ds_map_pose.pose.pose.position.x = t_map_to_ds[0]
        # self.ds_map_pose.pose.pose.position.y = t_map_to_ds[1]
        # self.ds_map_pose.pose.pose.position.z = t_map_to_ds[2]
        # self.ds_map_pose.pose.pose.orientation.x = quat_map_to_ds[0]
        # self.ds_map_pose.pose.pose.orientation.y = quat_map_to_ds[1]
        # self.ds_map_pose.pose.pose.orientation.z = quat_map_to_ds[2]
        # self.ds_map_pose.pose.pose.orientation.w = quat_map_to_ds[3]
        # self.ds_map_pose.pose.covariance = self.ds_localization_pose.pose.covariance

        # # Transform map --> sam/base_link
        # t_sam_odom_mat = transformations.translation_matrix(sam_ave_pose)
        # odom_to_sam_mat = np.matmul(t_sam_odom_mat, R_sam_odom)
        # map_to_sam_mat = np.matmul(odom_to_map_inv, odom_to_sam_mat)

        # t_map_to_sam = transformations.translation_from_matrix(map_to_sam_mat)
        # quat_map_to_sam = transformations.quaternion_from_matrix(map_to_sam_mat)

        # self.sam_map_pose.header.frame_id = self.odom_frame
        # self.sam_map_pose.header.stamp = rospy.Time.now()
        # self.sam_map_pose.pose.pose.position.x = t_map_to_sam[0]
        # self.sam_map_pose.pose.pose.position.y = t_map_to_sam[1]
        # self.sam_map_pose.pose.pose.position.z = t_map_to_sam[2]
        # self.sam_map_pose.pose.pose.orientation.x = quat_map_to_sam[0]
        # self.sam_map_pose.pose.pose.orientation.y = quat_map_to_sam[1]
        # self.sam_map_pose.pose.pose.orientation.z = quat_map_to_sam[2]
        # self.sam_map_pose.pose.pose.orientation.w = quat_map_to_sam[3]
        # self.sam_map_pose.pose.covariance = self.sam_localization_pose.pose.covariance


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
        # self.ds_map_pub.publish(self.ds_map_pose)
        # self.sam_map_pub.publish(self.sam_map_pose)

        # print(self.sam_poses_array)

        self.particle_weight_pub.publish(self.particle_weights)
        self.sam_particle_pub.publish(self.sam_poses_array)

    def print_states(self):
        """
        Bunch of print statements to check for things.
        """
        # print("SAM: {}".format(self.sam_map_pose.pose.pose.position))
        # print("DS: {}".format(self.ds_map_pose.pose.pose.position))


if __name__ == '__main__':

    rospy.init_node('slamParticleFilter', disable_signals=False)
    try:
        SlamParticleFilter()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch pf")
