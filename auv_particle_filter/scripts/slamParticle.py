#!/usr/bin/env python

# Standard dependencies
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf.transformations import translation_matrix
from tf.transformations import quaternion_matrix


class Particle(object):
    """
    Class with particle definition for particle filter based SLAM
    """
    def __init__(self, p_num, index, map2odom_matrix,
                 init_ds = [0., 0.],
                 init_cov=[0.]*12, meas_std=0.01,
                 process_cov=[0.]*12):

        self.p_num = p_num  # What is this?
        self.index = index  # What is this?

        self.p_pose = [0.]*12   # now [SAM, DS], both with 6 DoF
        self.p_pose[6:8] = init_ds # Initial estimate for the docking station position
        self.map2odom_mat = map2odom_matrix
        self.init_cov = init_cov
        self.process_cov = np.asarray(process_cov)
        self.meas_cov = np.diag([meas_std**2]*2)
        self.w = 0.0
        self.add_noise(init_cov)

    def add_noise(self, noise):
        """
        Add noise to covariances and poses
        """
        noise_cov =np.diag(noise)
        current_pose = np.asarray(self.p_pose)
        noisy_pose = current_pose + np.sqrt(noise_cov).dot(np.random.randn(12,1)).T
        self.p_pose = noisy_pose[0]


    def motion_pred(self, odom_t, dt):
        """
        Predict where the particle will be based on SAM's and the docking stations'
        motion model.
        """
        # Generate noise
        noise_vec = (np.sqrt(self.process_cov)*np.random.randn(1, 12)).flatten()

        ## SAM's motion model
        # Angular motion: integrate yaw, read roll and pitch directly
        vel_rot = np.array([0.,
                            0.,
                            odom_t.twist.twist.angular.z])

        rot_t = np.array(self.p_pose[3:6]) + vel_rot * dt + noise_vec[3:6]
        yaw_t = (rot_t[2] + np.pi) % (2 * np.pi) - np.pi

        euler_t = euler_from_quaternion(np.array([odom_t.pose.pose.orientation.x,
                                                  odom_t.pose.pose.orientation.y,
                                                  odom_t.pose.pose.orientation.z,
                                                  odom_t.pose.pose.orientation.w]))

        roll_t = euler_t[0]
        pitch_t = euler_t[1]
        self.p_pose[3:6] = [roll_t, pitch_t, yaw_t]

        # Linear motion
        vel_p = np.array([odom_t.twist.twist.linear.x,
                         odom_t.twist.twist.linear.y,
                         odom_t.twist.twist.linear.z])

        rot_mat_t = self.full_rotation(roll_t,pitch_t, yaw_t)
        step_t = np.matmul(rot_mat_t, vel_p * dt) + noise_vec[0:3]

        self.p_pose[0] += step_t[0]
        self.p_pose[1] += step_t[1]
        self.p_pose[2] = odom_t.pose.pose.position.z    # Depth can be read directly

        ## Docking Station motion model
        # Assume the docking station doesn't move.
        # Angular motion
        ds_rot_vel = np.array([0., 0., 0.])
        ds_rot_t = np.array(self.p_pose[9:12]) + ds_rot_vel * dt * noise_vec[9:12]

        self.p_pose[9:12] = ds_rot_t

        # Linear motion
        ds_lin_vel = np.array([0., 0., 0.])

        ds_rot_mat_t = self.full_rotation(*ds_rot_t)
        ds_lin_t = np.matmul(ds_rot_mat_t, ds_lin_vel * dt) + noise_vec[6:9]

        self.p_pose[6:9] += ds_lin_t


    def get_p_pose(self):
        """
        Convert pose into position vector and rotation matrix
        """
        t_particle = translation_matrix(self.p_pose[0:3])
        q = quaternion_from_euler(self.p_pose[3],self.p_pose[4],self.p_pose[5])
        q_particle = quaternion_matrix(q)
        mat = np.dot(t_particle, q_particle)

        trans_mat = self.map2odom_mat.dot(mat)
        p = trans_mat[0:3, 3]
        R = trans_mat[0:3, 0:3]

        return (p, R)


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
        """
        # TODO: Used 1/distance to compute the weight,
        # because we went to penealize big differences between the
        # particle states and the perception measurements.

        perception_diff = np.zeros(2)
        perception_diff[0] = self.p_pose[6] - docking_station_pose.pose.pose.position.x
        perception_diff[1] = self.p_pose[7] - docking_station_pose.pose.pose.position.y

        perception_diff_norm = np.linalg.norm(perception_diff)

        particle_diff = np.zeros(2)
        particle_diff[0] = self.p_pose[6] - self.p_pose[0]
        particle_diff[1] = self.p_pose[7] - self.p_pose[1]

        particle_diff_norm = np.linalg.norm(particle_diff)

        self.w = 1/(abs(perception_diff_norm - particle_diff_norm) + 1.e-200)



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
