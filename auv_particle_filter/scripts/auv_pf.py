#!/usr/bin/env python

# Standard dependencies
import rospy
import sys
import numpy as np
import tf2_ros
import tf

from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, PointStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from rospy_tutorials.msg import Floats

from tf.transformations import identity_matrix, quaternion_from_euler, euler_from_quaternion, rotation_matrix

# For sim mbes action client
from auv_particle import SamParticle, matrix_from_tf
from resampling import residual_resample

class auv_pf(object):

    def __init__(self):
        # Read necessary parameters
        self.pc = rospy.get_param('~particle_count', 10) # Particle Count
        self.map_frame = rospy.get_param('~map_frame', 'map') # map frame_id
        self.utm_frame = rospy.get_param('~utm_frame', 'utm') # mbes frame_id
        self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')

        # Initialize tf listener
        tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfBuffer)
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

        # Global variables
        self.pred_odom = None

        # Initialize particle poses publisher
        self.poses = PoseArray()
        self.poses.header.frame_id = self.odom_frame
        pose_array_top = rospy.get_param("~particle_poses_topic", '/particle_poses')
        self.pf_pub = rospy.Publisher(pose_array_top, PoseArray, queue_size=10)

        # Initialize loc odom publisher
        self.loc_pose = Odometry()
        self.loc_pose.header.frame_id = self.odom_frame
        self.loc_pose.child_frame_id = "sam/base_link"
        loc_top = rospy.get_param("~odom_corrected_topic", '/average_pose')
        self.loc_pub = rospy.Publisher(loc_top, Odometry, queue_size=100)

        self.loc_tf = tf.TransformBroadcaster()      

        # Transforms from auv_2_ros
        try:
            rospy.loginfo("Waiting for transforms")
            m2o_tf = tfBuffer.lookup_transform(self.map_frame, self.odom_frame,
                                               rospy.Time(0), rospy.Duration(35))
            self.m2o_mat = matrix_from_tf(m2o_tf)
            rospy.loginfo("Got map to odom")

        except:
            rospy.loginfo("ERROR: Could not lookup transform from base_link to mbes_link")

        # Initialize list of particles
        self.particles = np.empty(self.pc, dtype=object)
        for i in range(self.pc):
            self.particles[i] = SamParticle(self.pc, i, self.m2o_mat, 
                                            init_cov=init_cov, meas_std=meas_std,
                                            process_cov=motion_cov)

        # Start timing now
        self.time = rospy.Time.now().to_sec()
        self.old_time = rospy.Time.now().to_sec()

        # Create particle to compute DR
        self.dr_particle = SamParticle(self.pc, self.pc+1, self.m2o_mat, 
                                        init_cov=[0.]*6, meas_std=meas_std,
                                        process_cov=motion_cov)
        
        # Aux topic to simulate diving
        dive_top = rospy.get_param("~aux_dive", '/dive')
        rospy.Subscriber(dive_top, Bool, self.dive_cb, queue_size=100)
        self.diving = False

        # GPS odom topic, in UTM frame
        gps_top = rospy.get_param("~gps_odom_topic", '/gps')
        rospy.Subscriber(gps_top, Odometry, self.gps_odom_cb, queue_size=100)
        
        # Establish subscription to odometry message (intentionally last)
        odom_top = rospy.get_param("~odometry_topic", 'odom')
        rospy.Subscriber(odom_top, Odometry, self.odom_callback, queue_size=100)
        
        # PF pub and broadcaster loop
        rospy.Timer(rospy.Duration(0.1), self.loc_loop)

        # PF filter created. Start auv_2_ros survey playing
        rospy.loginfo("Particle filter class successfully created")

        rospy.spin()


    def dive_cb(self, dive_msg):
        self.diving = dive_msg.data

    def gps_odom_cb(self, gps_odom):
        if self.old_time and self.time > self.old_time:
            # To simulate diving as absence of GPS in floatsam        
            if not self.diving:
                # Meas update
                weights = self.update(gps_odom, self.odom_latest)

                # Particle resampling
                self.resample(weights)
    
    def update(self, gps_odom, odom):
        
        weights = []
        for i in range(0, self.pc):
           
            # Current uncertainty of GPS meas
            # self.particles[i].meas_cov = gps_odom.pose.covariance
            
            goal_point = PointStamped()
            goal_point.header.frame_id = self.utm_frame
            goal_point.header.stamp = rospy.Time(0)
            goal_point.point.x = gps_odom.pose.pose.position.x
            goal_point.point.y = gps_odom.pose.pose.position.y
            goal_point.point.z = 0.

            try:
                gps_map = self.listener.transformPoint(self.map_frame, goal_point)
                
                # Compute particle weight
                self.particles[i].compute_weight(gps_map)
                weights.append(self.particles[i].w)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("PF: Transform to utm-->map not available yet")
            pass
            

        weights_array = np.asarray(weights)
        # Add small non-zero value to avoid hitting zero
        weights_array += 1.e-200

        return weights_array

    def resample(self, weights):
        # Normalize weights
        #  print (weights)
        weights /= weights.sum()

        # N_eff = self.pc
        # if weights.sum() == 0.:
        #     rospy.loginfo("All weights zero!")
        # else:
        #     N_eff = 1/np.sum(np.square(weights))

        # Resampling?
        # if N_eff < self.pc/2. :
        indices = residual_resample(weights)
        keep = list(set(indices))
        lost = [i for i in range(self.pc) if i not in keep]
        dupes = indices[:].tolist()
        for i in keep:
            dupes.remove(i)

        self.reassign_poses(lost, dupes)
        # Add noise to particles
        for i in range(self.pc):
            self.particles[i].add_noise(self.res_noise_cov)


    def reassign_poses(self, lost, dupes):
        for i in range(len(lost)):
            # Faster to do separately than using deepcopy()
            self.particles[lost[i]].p_pose = self.particles[dupes[i]].p_pose
    

    def odom_callback(self, odom_msg):
        self.time = odom_msg.header.stamp.to_sec()
        self.odom_latest = odom_msg

        if self.old_time and self.time > self.old_time:
            # Motion prediction
            self.predict(odom_msg)    
    
        # self.publish_stats(odom_msg)

        self.old_time = self.time

    def predict(self, odom_t):
        dt = self.time - self.old_time
        for i in range(0, self.pc):
            self.particles[i].motion_pred(odom_t, dt)

        # Predict DR
        self.dr_particle.motion_pred(odom_t, dt)

    
    def update_loc_pose(self, pose_list):

        poses_array = np.array(pose_list)
        ave_pose = poses_array.mean(axis = 0)
        self.loc_pose.pose.pose.position.x = ave_pose[0]
        self.loc_pose.pose.pose.position.y = ave_pose[1]
        self.loc_pose.pose.pose.position.z = ave_pose[2]
        roll  = ave_pose[3]
        pitch = ave_pose[4]

        # TODO: fix wrapping up yaw
        poses_array[:,5] = [(yaw + np.pi) % (2 * np.pi) - np.pi 
                             for yaw in  poses_array[:,5]]
        yaw = np.mean(poses_array[:,5])
        
        quat_t = quaternion_from_euler(roll, pitch, yaw)
        self.loc_pose.pose.pose.orientation = Quaternion(*quat_t)
        self.loc_pose.header.stamp = rospy.Time.now()
        
        # Calculate covariance
        self.cov = np.zeros((3, 3))
        for i in range(self.pc):
            dx = (poses_array[i, 0:3] - ave_pose[0:3])
            self.cov += np.diag(dx*dx.T) 
            self.cov[0,1] += dx[0]*dx[1] 
            self.cov[0,2] += dx[0]*dx[2] 
            self.cov[1,2] += dx[1]*dx[2] 
        self.cov /= self.pc
        self.cov[1,0] = self.cov[0,1]
        # print(self.cov)

        self.loc_pose.pose.covariance = [0.]*36
        for i in range(3):
            for j in range(3):
                self.loc_pose.pose.covariance[i*3 + j] = self.cov[i,j]
        
        self.loc_pub.publish(self.loc_pose)

        self.loc_tf.sendTransform([ave_pose[0], ave_pose[1], 0.],
                                    quat_t,
                                    rospy.Time.now(),
                                    "sam/base_link",
                                    self.odom_frame)

    # TODO: publish markers instead of poses
    #       Optimize this function
    def loc_loop(self, event):
        self.poses.poses = []
        pose_list = []
        for i in range(self.pc):
            pose_i = Pose()
            pose_i.position.x = self.particles[i].p_pose[0]
            pose_i.position.y = self.particles[i].p_pose[1]
            pose_i.position.z = self.particles[i].p_pose[2]
            pose_i.orientation = Quaternion(*quaternion_from_euler(
                self.particles[i].p_pose[3],
                self.particles[i].p_pose[4],
                self.particles[i].p_pose[5]))

            self.poses.poses.append(pose_i)
            pose_list.append(self.particles[i].p_pose)
        
        # Publish particles with time odometry was received
        self.poses.header.stamp = rospy.Time.now()
        self.update_loc_pose(pose_list)

        # Publish particles as arrows
        self.pf_pub.publish(self.poses)



if __name__ == '__main__':

    rospy.init_node('auv_pf', disable_signals=False)
    try:
        auv_pf()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch pf")
        pass
