#!/usr/bin/env python3
"""
Particle module for SLAM-based particle filter to detect underwater docking station.
"""

# Standard dependencies
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf.transformations import translation_matrix
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from tf.transformations import euler_matrix, euler_from_matrix


class Particle(object):
    """
    Class with particle definition for particle filter based SLAM
    """
    def __init__(self, p_num, index, odom_to_map_mat,
                 init_sam = [0.]*6,
                 init_ds = [0., 0.],
                 init_cov=[0.]*12, meas_std=0.01,
                 process_cov=[0.]*12):

        self.p_num = p_num  # What is this?
        self.index = index  # What is this?

        self.p_pose = [0.]*12   # now [SAM, DS], both with 6 DoF
        self.p_pose[0:6] = init_sam
        self.p_pose[6:12] = init_ds # Initial estimate for the docking station position
        self.odom_to_map_mat = odom_to_map_mat
        self.init_cov = init_cov
        self.process_cov = np.asarray(process_cov)
        # self.meas_cov = np.diag([meas_std**2]*2)
        self.w = 0.0
        self.add_noise(init_cov)


    def add_noise(self, noise):
        """
        Add noise to poses
        """
        noise_cov =np.diag(noise)
        current_pose = np.asarray(self.p_pose)
        noisy_pose = current_pose + np.sqrt(noise_cov).dot(np.random.randn(12,1)).T
        self.p_pose = noisy_pose[0]


    def motion_pred(self, odom_t, dt):
        """
        Predict where the particle will be based on SAM's and the docking stations'
        motion model.
        TODO: Remove sensor readings here and place them in the update step. Then you 
        actually do filtering and not only observation.
        """
        # Generate noise
        noise_vec = (np.sqrt(self.process_cov)*np.random.randn(1, 12)).flatten()

        ## SAM's motion model
        # Angular motion: integrate yaw, read roll and pitch directly
        # FIXME: There is still sketchy stuff going on. Check that you use an ENU odom message and adjust
        #       all transformations accordingly. Could be somewhere before. The motion model shouldn't
        #       have any changes in signs or flipping x and y. It's a very quick fix for now, but not
        #       proper.
        # Also, how does that work on SAM then? Make sure to check that as well.
        vel_rot = np.array([odom_t.twist.twist.angular.x,
                            odom_t.twist.twist.angular.y,
                            -odom_t.twist.twist.angular.z])

        rot_step_t = vel_rot * dt + noise_vec[3:6]

        R_rot_step_t = euler_matrix(*rot_step_t)
        R_sam_t = euler_matrix(*self.p_pose[3:6])

        R_sam_update = np.matmul(R_sam_t, R_rot_step_t)

        rot_step_t_transformed = euler_from_matrix(R_sam_update)

        rot_t = np.array(self.p_pose[3:6]) + rot_step_t_transformed
        # yaw_t = (rot_t[2] + np.pi) % (2 * np.pi) - np.pi
        yaw_t = rot_step_t_transformed[2]

        euler_t = euler_from_quaternion(np.array([odom_t.pose.pose.orientation.x,
                                                  odom_t.pose.pose.orientation.y,
                                                  odom_t.pose.pose.orientation.z,
                                                  odom_t.pose.pose.orientation.w]))

        roll_t = euler_t[0]
        pitch_t = euler_t[1]

        # debug only!
        # yaw_t = euler_t[2]
        # print("Yaw diff: {}".format(yaw_t-euler_t[2]))

        self.p_pose[3:6] = [roll_t, pitch_t, yaw_t]

        # Linear motion
        vel_p = np.array([odom_t.twist.twist.linear.y,
                         odom_t.twist.twist.linear.x,
                         -odom_t.twist.twist.linear.z])

        rot_mat_t = self.full_rotation(roll_t, pitch_t, yaw_t)
        step_t = np.matmul(rot_mat_t, vel_p * dt) + noise_vec[0:3]

        self.p_pose[0] += step_t[0]
        self.p_pose[1] += step_t[1]
        self.p_pose[2] = odom_t.pose.pose.position.z    # Depth can be read directly

        ## Docking Station motion model
        # Assume the docking station doesn't move. For a dynamic motion model, fill
        # in the lines. 
        # Angular motion
        # ds_rot_vel = np.array([0., 0., 0.])
        # ds_rot_t = np.array(self.p_pose[9:12]) + ds_rot_vel * dt + noise_vec[9:12]

        # self.p_pose[9:12] = ds_rot_t

        # # Linear motion
        # ds_lin_vel = np.array([0., 0., 0.])

        # ds_rot_mat_t = self.full_rotation(*ds_rot_t)
        # ds_lin_t = np.matmul(ds_rot_mat_t, ds_lin_vel * dt) + noise_vec[6:9]

        # self.p_pose[6:9] += ds_lin_t


    def full_rotation(self, roll, pitch, yaw):
        """
        Calculate a rotation around x, y, and z axis
        """
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


    def compute_weight(self, docking_station_pose):
        """
        Compute the weight of the particle based on the distance between
        SAM and the docking station measured by the perception node
        and encoded in the particle's states.

        Note, the covariance is stored as an array in row-major order:
        https://en.wikipedia.org/wiki/Row-_and_column-major_order
        Take that into account when accessing the values.

        Also note, Used 1/distance to compute the weight,
        because we went to penealize big differences between the
        particle states and the perception measurements.

        Last, note that we can't just subtract the Euler angles. We need
        to properly compute the difference by multiplying the two rotation 
        matrices, while inverting the latter one. 
        """

        ## Calculate difference between the poses
        # 1. Difference between SAM and DS from particle -> relative pose particle
        # Build transformation matrices:
        # Translation matrices
        t_sam_particle = self.p_pose[0:3]
        T_ds_particle = translation_matrix(self.p_pose[6:9])

        # Rotation matrices:
        quat_sam_particle = quaternion_from_euler(*self.p_pose[3:6])
        quat_ds_particle = quaternion_from_euler(*self.p_pose[9:12])

        R_sam_particle = quaternion_matrix(quat_sam_particle)
        R_ds_particle = quaternion_matrix(quat_ds_particle)

        t_inv_sam_particle = -np.dot(R_sam_particle[0:3,0:3].T, t_sam_particle)
        T_inv_sam_particle = translation_matrix(t_inv_sam_particle)

        M_inv_sam_particle = np.matmul(T_inv_sam_particle, R_sam_particle.T)
        M_ds_particle = np.matmul(T_ds_particle, R_ds_particle)

        M_diff_particle = np.matmul(M_inv_sam_particle, M_ds_particle)

        # 2. Difference between relative pose particle and relative pose perception
        # By definition of the particle filter, we want (x - my), where x is the particle
        # and my the mean. In our case, my is the perception measurement.
        t_perception = [docking_station_pose.pose.pose.position.x,
                        docking_station_pose.pose.pose.position.y,
                        docking_station_pose.pose.pose.position.z]
        quat_perception = [docking_station_pose.pose.pose.orientation.x,
                           docking_station_pose.pose.pose.orientation.y,
                           docking_station_pose.pose.pose.orientation.z,
                           docking_station_pose.pose.pose.orientation.w]
        R_perception = quaternion_matrix(quat_perception)

        t_inv_perception = -np.matmul(R_perception[0:3,0:3].T, t_perception)
        T_inv_perception = translation_matrix(t_inv_perception)

        M_inv_perception = np.matmul(T_inv_perception, R_perception.T)

        M_diff = np.matmul(M_diff_particle, M_inv_perception)

        # 3. Recover states and angles for mahalanobis distance
        diff = np.zeros(6)

        t_diff = M_diff[0:3,3]
        quat_diff = quaternion_from_matrix(M_diff)
        rpy_diff = euler_from_quaternion(quat_diff)

        diff[0:3] = t_diff
        diff[3:6] = rpy_diff

        # 4. For debugging purposes, the various states
        pose_sam = np.zeros(6)
        pose_sam[0:3] = self.p_pose[0:3]
        pose_sam[3:6] = np.rad2deg(self.p_pose[3:6])

        pose_ds = np.zeros(6)
        pose_ds[0:3] = self.p_pose[6:9]
        pose_ds[3:6] = np.rad2deg(self.p_pose[9:12])

        diff_particle = np.zeros(6)
        quat_particle_diff = quaternion_from_matrix(M_diff_particle)
        rpy_particle_diff = euler_from_quaternion(quat_particle_diff)
        diff_particle[0:3] = M_diff_particle[0:3, 3]
        diff_particle[3:6] = np.rad2deg(rpy_particle_diff)

        diff_perception = np.zeros(6)
        rpy_perception_diff = euler_from_quaternion(quat_perception)
        diff_perception[0:3] = t_perception
        diff_perception[3:6] = np.rad2deg(rpy_perception_diff)

        meas_cov_tmp = np.array([0.0]*36)

        for i, value in enumerate(docking_station_pose.pose.covariance):
            meas_cov_tmp[i] = value

        meas_cov = np.reshape(meas_cov_tmp,(6,6))
        inv_cov = np.linalg.inv(meas_cov)

        mahal_dist = np.sqrt(np.matmul(diff, np.matmul(inv_cov, diff)))

        self.w = 1/(mahal_dist + 1.e-200)


def matrix_from_tf(transform):
    """
    Transform a transform message into a numpy matrix.
    """
    if transform._type == 'geometry_msgs/TransformStamped':
        transform = transform.transform

    trans = (transform.translation.x,
             transform.translation.y,
             transform.translation.z)
    quat_ = (transform.rotation.x,
             transform.rotation.y,
             transform.rotation.z,
             transform.rotation.w)

    tmat = translation_matrix(trans)
    qmat = quaternion_matrix(quat_)

    return np.dot(tmat, qmat)
