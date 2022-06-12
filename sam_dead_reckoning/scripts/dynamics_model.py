#!/usr/bin/env python

import rospy
import numpy as np
#  from sam_msgs.msg import ThrusterRPMs
from smarc_msgs.msg import DualThrusterFeedback, ThrusterFeedback
from sam_msgs.msg import PercentStamped, ThrusterAngles
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class SamACC(object):

    def __init__(self):
        
        self.state_fb_topic = rospy.get_param("~state_fb_topic", "sam/dr/odom")
        self.dr_thrust_topic = rospy.get_param('~thrust_acc', '/sam/dr/motion_dr')
        self.rpm1_fb_topic = rospy.get_param('~thrust1_fb', '/sam/core/thruster1_fb')
        self.rpm2_fb_topic = rospy.get_param('~thrust2_fb', '/sam/core/thruster2_fb')
        self.vbs_topic = rospy.get_param('~vbs_fb', '/sam/core/vbs_fb')
        self.lcg_topic = rospy.get_param('~lcg_fb', '/sam/core/lcg_fb')
        self.thrust_vector_topic = rospy.get_param("~thrust_vector_topic", "/sam/core/thrust_vector_cmd")

        self.base_frame = rospy.get_param('~base_frame', 'sam/base_link')
        #self.odom_frame = rospy.get_param('~odom_frame', 'sam/odom')

        self.sub1_thrust = rospy.Subscriber(self.rpm1_fb_topic, ThrusterFeedback, self.thrust1CB)
        self.sub2_thrust = rospy.Subscriber(self.rpm2_fb_topic, ThrusterFeedback, self.thrust2CB)
        self.sub_vbs = rospy.Subscriber(self.vbs_topic, PercentStamped, self.vbs_cb)
        self.sub_lcg = rospy.Subscriber(self.lcg_topic, PercentStamped, self.lcg_cb)
        self.sub_tvec = rospy.Subscriber(self.thrust_vector_topic, ThrusterAngles, self.thrust_vec_cb)
        self.sub_state = rospy.Subscriber(self.state_fb_topic, Odometry, self.feedback_cb )

        self.control_pub = rospy.Publisher(self.dr_thrust_topic, TwistStamped, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.eom)

        self.coeff = 0.0005
        self.rpm_1 = 0.
        self.rpm_2 = 0.
        self.d_e = 0.
        self.d_r = 0.
        self.vbs_val = 0.
        self.lcg_val = 0.
        self.current_state = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

        rospy.spin()

    def normalize(self,value):
        value_n= 0.02*value -1
        return value_n
    
    def vbs_cb(self, vbs_msg):
        self.vbs_val = self.normalize(vbs_msg.value)
  
    def lcg_cb(self, lcg_msg):
        self.lcg_val = self.normalize(lcg_msg.value)

    def thrust1CB(self, rpm_msg):
        self.rpm_1 = rpm_msg.rpm.rpm

    def thrust2CB(self, rpm_msg):
        self.rpm_2 = rpm_msg.rpm.rpm
    
    def thrust_vec_cb(self, vec_msg):
        self.d_e = vec_msg.thruster_vertical_radians
        self.d_r = vec_msg.thruster_horizontal_radians


    def feedback_cb(self, odom_fb):
        
        self.current_state = self.get_state_feedback(odom_fb)
        

    def get_state_feedback(self, odom_msg):
        # Insert functon to subscribe from \ODOM here
        #print('getting state feedback')
        # Converting from ENU to NED, e.g. see https://www.programmersought.com/article/83764943652/ or https://robotics.stackexchange.com/questions/19669/rotating-ned-to-enu
        # that is why we switch y and x, rotate the yaw by 90 degrees and have the opposite sign on z.
        x = odom_msg.pose.pose.position.y
        y = odom_msg.pose.pose.position.x
        z = -odom_msg.pose.pose.position.z
        eta0 = odom_msg.pose.pose.orientation.w
        eps1 = odom_msg.pose.pose.orientation.x
        eps2 = odom_msg.pose.pose.orientation.y
        eps3 = odom_msg.pose.pose.orientation.z

        rpy= euler_from_quaternion([eps1,eps2,eps3,eta0])
        roll = rpy[0]
        pitch = rpy[1]
        yaw = (1.571-rpy[2])

        u = odom_msg.twist.twist.linear.y
        v = odom_msg.twist.twist.linear.x
        w = -odom_msg.twist.twist.linear.z
        p= odom_msg.twist.twist.angular.y
        q= odom_msg.twist.twist.angular.x
        r= -odom_msg.twist.twist.angular.z

        current_state = np.array([x,y,z,roll,pitch,yaw,u,v,w,p,q,r]) #wth euler angles
        #rospy.loginfo_throttle(5,'Feedback:')
        #rospy.loginfo_throttle(5,current_state[:3])
        #rospy.loginfo_throttle(5,'Setpoint:')
        #rospy.loginfo_throttle(5,self.current_setpoint[:3])

        return current_state

    def skew(self, l):
        l = l.flatten()
        l1 = l[0]
        l2 = l[1]
        l3 = l[2]
        return np.array([[0, -l3, l2], [l3, 0, -l1], [-l2, l1, 0]])
        
    def eom(self,ev):
        # extract states and controls
        x,y,z,phi,theta, psi, u, v, w, p, q, r = self.current_state
        rpm1, rpm2, de, dr, lcg, vbs = self.rpm_1, self.rpm_2, self.d_e, self.d_r, self.lcg_val, self.vbs_val
       

        eta = np.array([[x], [y], [z], [phi], [theta], [psi]])
        nu = np.array([[u], [v], [w], [p], [q], [r]])

        # scaling controls
        rpm_scale = 1.
        d_scale = 1
        vbs_scale = 1.
        lcg_scale = 1.
        
        rpm1 = rpm1 * rpm_scale
        rpm2 = rpm2 * rpm_scale
        de = de * d_scale
        dr = dr * d_scale
        vbs = vbs * vbs_scale
        #vbs = 0.
        lcg = lcg * lcg_scale
        #lcg = 0.
        

        # assign parameters
        m = 14 #15.4  # mass
        Ixx = 0.0294 
        Iyy = 1.6202
        Izz = 1.6202 
        I_o = np.array([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])

        # cg position
        x_g = 0.1 + lcg*0.01
        y_g = 0.0
        z_g = 0.0
        r_g = np.array([x_g, y_g, z_g])

        # cb position
        x_b = 0.1
        y_b = 0.0
        z_b = 0.0
        r_b = np.array([x_b, y_b, z_b])

        # center of pressure position
        x_cp = 0.1
        y_cp = 0.0
        z_cp = 0.0
        r_cp = np.array([x_cp, y_cp, z_cp])

        W = m * 9.81
        B = W + vbs* 1.5

        # Hydrodynamic coefficients
        Xuu = 3. #1.0
        Yvv = 10. #100.0
        Zww = 50. #100.0
        Kpp = 0.1 #10.0
        Mqq = 40 #100.0
        Nrr = 10 #150.0

        # Control actuators
        K_T = np.array([0.0175, 0.0175])
        Q_T = 0*np.array([0.001, -0.001])

        # Mass and inertia matrix
        M = np.block([[m * np.eye(3, 3), -m * self.skew(r_g)], [m * self.skew(r_g), I_o]])
        assert M.shape == (6, 6), M

        # Coriolis and centripetal matrix
        nu1 = np.array([[u], [v], [w]])
        nu2 = np.array([[p], [q], [r]])
        top_right = -m * self.skew(nu1) - m * self.skew(nu2) * self.skew(r_g)
        bottom_left = -m * self.skew(nu1) + m * self.skew(r_g) * self.skew(nu2)
        Ionu2 = I_o.dot(nu2)
        bottom_right = -self.skew(Ionu2)
        C_RB = np.block([[np.zeros([3, 3]), top_right], [bottom_left, bottom_right]])
        assert C_RB.shape == (6, 6), C_RB

        # Damping matrix
        Forces = np.diag([Xuu * abs(u), Yvv * abs(v), Zww * abs(w)])
        Moments = np.diag([Kpp * abs(p), Mqq * abs(q), Nrr * abs(r)])
        Coupling = np.matmul(self.skew(r_cp), Forces)
        D = np.block([[Forces, np.zeros([3, 3])], [-Coupling, Moments]])
        assert D.shape == (6, 6), D

        # rotational transform between body and NED in Euler        
        T_euler = np.array(
            [
                [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                [0, np.cos(phi), -np.sin(phi)],
                [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)],
            ]
        )

        R_euler = np.array(
            [
                [
                    np.cos(psi)*np.cos(theta),
                    -np.sin(psi)*np.cos(phi)+np.cos(psi)*np.sin(theta)*np.sin(phi),
                    np.sin(psi)*np.sin(phi)+np.cos(psi)*np.cos(phi)*np.sin(theta),
                ],
                [
                    np.sin(psi)*np.cos(theta),
                    np.cos(psi)*np.cos(phi)+np.sin(phi)*np.sin(theta)*np.sin(psi),
                    -np.cos(psi)*np.sin(phi)+np.sin(theta)*np.sin(psi)*np.cos(phi),
                ],
                [
                    -np.sin(theta),
                    np.cos(theta)*np.sin(phi),
                    np.cos(theta)*np.cos(phi),
                ],
            ]
        )
        assert R_euler.shape == (3, 3), R_euler

        J_eta = np.block([[R_euler, np.zeros([3, 3])], [np.zeros([3, 3]), T_euler]])
        assert J_eta.shape == (6, 6), J_eta

        # buoyancy in quaternions
        f_g = np.array([[0], [0], [W]])
        f_b = np.array([[0], [0], [-B]])
        row1 = [np.linalg.inv(R_euler).dot((f_g + f_b))]
        row2 = [
            self.skew(r_g).dot(np.linalg.inv(R_euler)).dot(f_g)
            + self.skew(r_b).dot(np.linalg.inv(R_euler)).dot(f_b)
        ]
        geta = np.block([row1, row2])
        assert geta.shape == (6, 1), geta


        F_T = K_T.dot(np.array([[rpm1], [rpm2]]))
        M_T = Q_T.dot(np.array([[rpm1], [rpm2]]))
        tauc = np.block(
            [
                [F_T * np.cos(de) * np.cos(dr)],
                [-F_T * np.sin(dr)],
                [F_T * np.sin(de) * np.cos(dr)],
                [M_T * np.cos(de) * np.cos(dr)],
                [-M_T * np.sin(dr)],
                [M_T * np.sin(de) * np.cos(dr)],
            ]
        )
        assert tauc.shape == (6, 1), tauc
        # Kinematics
        etadot = np.block([J_eta.dot(nu)])

        assert etadot.shape == (6, 1), etadot

        # Dynamics
        invM = np.linalg.inv(M)
        crbd = C_RB + D
        other = crbd.dot(nu)-geta
        other2 = tauc - other
        nudot = invM.dot(other2)

        assert nudot.shape == (6, 1), nudot
        #sdot = np.block([[etadot], [nudot]])
        nu = nudot.flatten()
        rospy.loginfo_throttle(5,nu)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = self.base_frame
        twist_msg.twist.linear.x = nu[1]
        twist_msg.twist.linear.y = nu[0]
        twist_msg.twist.linear.z = -nu[2]
        twist_msg.twist.angular.x = nu[4]
        twist_msg.twist.angular.y = nu[3]
        twist_msg.twist.angular.z = -nu[5]
        self.control_pub.publish(twist_msg)


    """
    def timerCB(self, ev):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = self.base_frame
        twist_msg.twist.linear.x = (self.rpm_1 + self.rpm_2) * self.coeff
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        self.control_pub.publish(twist_msg)
    """

if __name__ == "__main__":

    rospy.init_node('sam_am')   
    try:
        SamACC()
    except rospy.ROSInterruptException:
        pass

