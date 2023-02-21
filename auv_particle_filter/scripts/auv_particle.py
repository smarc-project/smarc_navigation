#!/usr/bin/env python

# Standard dependencies
import math
import rospy
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf.transformations import translation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, quaternion_from_matrix

from scipy.stats import multivariate_normal


class SamParticle(object):
    def __init__(self, p_num, index, m2o_matrix,
                 init_cov=[0.,0.,0.,0.,0.,0.], meas_std=0.01,
                 process_cov=[0.,0.,0.,0.,0.,0.]):

        self.p_num = p_num
        self.index = index

        # self.weight = 1.
        self.p_pose = [0.]*6
        self.m2o_tf_mat = m2o_matrix
        self.init_cov = init_cov
        self.process_cov = np.asarray(process_cov)
        self.meas_cov = np.diag([meas_std**2]*2)
        self.w = 0.0
        self.add_noise(init_cov)

    def add_noise(self, noise):
        noise_cov =np.diag(noise)
        current_pose = np.asarray(self.p_pose)
        noisy_pose = current_pose + np.sqrt(noise_cov).dot(np.random.randn(6,1)).T
        self.p_pose = noisy_pose[0]

    def motion_pred(self, odom_t, dt):
        # Generate noise
        noise_vec = (np.sqrt(self.process_cov)*np.random.randn(1, 6)).flatten()

        # Angular motion
        vel_rot = np.array([odom_t.twist.twist.angular.x,
                            odom_t.twist.twist.angular.y,
                            odom_t.twist.twist.angular.z])

        rot_t = np.array(self.p_pose[3:6]) + vel_rot * dt + noise_vec[3:6]

        roll_t = (rot_t[0] + np.pi) % (2 * np.pi) - np.pi
        pitch_t = (rot_t[1] + np.pi) % (2 * np.pi) - np.pi
        yaw_t = (rot_t[2] + np.pi) % (2 * np.pi) - np.pi
        self.p_pose[3:6] = [roll_t, pitch_t, yaw_t]

        # Linear motion
        vel_p = np.array([odom_t.twist.twist.linear.x,
                         odom_t.twist.twist.linear.y,
                         odom_t.twist.twist.linear.z])
        
        rot_mat_t = self.fullRotation(roll_t,pitch_t, yaw_t)
        step_t = np.matmul(rot_mat_t, vel_p * dt) + noise_vec[0:3]

        self.p_pose[0] += step_t[0]
        self.p_pose[1] += step_t[1]
        # Seems to be a problem when integrating depth from Ping vessel, so we just read it
        self.p_pose[2] = odom_t.pose.pose.position.z

    def get_p_mbes_pose(self):
        # Find particle's mbes_frame pose in the map frame 
        t_particle = translation_matrix(self.p_pose[0:3])
        q = quaternion_from_euler(self.p_pose[3],self.p_pose[4],self.p_pose[5])
        q_particle = quaternion_matrix(q)
        mat = np.dot(t_particle, q_particle)
        
        trans_mat = self.m2o_tf_mat.dot(mat)
        self.p = trans_mat[0:3, 3]
        self.R = trans_mat[0:3, 0:3]
        
        return (self.p, self.R)

    
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

        return np.matmul(rot_z, np.matmul(rot_y, rot_x))


    def compute_weight(self, gps_fix):
        # Current particle pose in the map frame
        p_part, r_mbes = self.get_p_mbes_pose()
        
        # TODO: combine uncertainties of odom and GPS estimates
        self.w = multivariate_normal.pdf([gps_fix.point.x, gps_fix.point.y], mean=p_part[0:2],
                                     cov=self.meas_cov)                                



def matrix_from_tf(transform):
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
