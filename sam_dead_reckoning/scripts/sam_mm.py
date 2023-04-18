#!/usr/bin/env python3
import numpy as np


def abs_approx(value):
    # differentiable-approximation-of-the-absolute-value-function
    alpha = 8
    return value * np.tanh(alpha * value)

class SAM(object):

    def __init__(self):
        # SAM properties
        self.m = 15.4
        self.Izz = 1.6202

        self.rpm_constant = 0.0
        self.dr_constant = 0.0

        # Hydrodynamic coefficients
        self.Xuu = 3  # 0.8 #1.
        self.Yvv = 20.0  # 100.
        self.Nrr = 20.0  # 100.

        # ## TANK PARAMETERS
        # self.Xuu = 10  # 0.8 #1.
        # self.Yvv = 18.0  # 100.
        # self.Nrr = 18.0  # 100.

    def eom(self, control):

    # def eom(self, state, control):
        # extract states and controls
        # x, y, psi, u, v, r = state

        rpm, dr = control

        # scale controls from -1 to 1 to the correct ranges
        # rpm_scale = 1 #0.15
        # d_scale = -1.

        ## TANK PARAMETERS
        # Potentially we need to adjust these. They differ from Joris model
        rpm_scale = 1 #0.15
        # d_scale = -2.5

        # let's change it again, seems different in real life now :)
        # d_scale = -5.0
        d_scale = -1.0

        rpm = rpm * rpm_scale
        dr = dr * d_scale

        # eta = np.array([[x], [y], [psi]])
        # nu = np.array([[u], [v], [r]])

        ## KYNEMATICS MODEL
        # rotational transform for kinematics
        # J_eta = np.array([[np.cos(psi), -np.sin(psi), 0.],
        #                   [np.sin(psi), np.cos(psi), 0.],
        #                   [0., 0., 1.]])

        # etadot = np.block([J_eta.dot(nu)])

        ## DYNAMIC MODEL

        # cg position
        x_g = 0.4
        y_g = 0.

        # # center of pressure position
        # x_cp = 0.1
        # y_cp = 0.

        # Control actuators
        KT = 0.35

        # # Coriolis and centripetal matrix
        # C_RB = np.array([[0., 0., -self.m * (x_g * r + v)],
        #                  [0., 0., -self.m * (y_g * r - u)],
        #                  [self.m * (x_g * r + v), self.m * (y_g * r - u), 0.]])

        # # Damping matrix
        # D = np.array([[self.Xuu * abs_approx(u), 0., 0.],
        #               [0, self.Yvv * abs_approx(v), 0],
        #               [-y_cp * self.Xuu * abs_approx(u), x_cp * self.Yvv * abs_approx(v), self.Nrr * abs_approx(r)]])


        
        # Mass and inertia matrix
        M = np.array([[self.m, 0., -self.m * y_g],
                      [0, self.m, self.m * x_g],
                      [-self.m * y_g, self.m * x_g, self.Izz]])
        
        # buoyancy in quaternions
        # geta = np.array([[0.],
        #                  [0.],
        #                  [0.]])

        # controls
        F_T = KT * rpm

        tauc = np.block([[F_T * np.cos(dr)],
                         [-F_T * np.sin(dr)],
                         [0.]])


        # Dynamics
        invM = np.linalg.inv(M)
        # crbd = C_RB + D
        # nudot = invM.dot(tauc - crbd.dot(nu) - geta)
        
        nudot = invM.dot(tauc)

        # sdot = np.block([[etadot],
        #                  [nudot]])

        sdot = np.block([[nudot]])

        return sdot.flatten()

    # def motion(self, state, control):
    #     return self.eom(state, control)

    def motion(self, control):
        return self.eom(control)

    def jacF(self, state, control, dt):

        eps = 1e-5
        J = np.zeros([len(state), len(state)], dtype=np.float)

        for i in range(len(state)):
            x1 = np.asarray(state).copy()
            x2 = np.asarray(state).copy()

            x1[i] += eps
            x2[i] -= eps

            f1 = self.eom(x1, control)
            f2 = self.eom(x2, control)

            J[:, i] = 1 / (2 * eps) * (f1 - f2)

        return np.eye(6) + dt * J

