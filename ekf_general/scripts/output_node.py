#!/usr/bin/env python

import rospy
from ekf_general.srv import *
from std_msgs.msg import Float32MultiArray

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from multiprocessing import Process, Queue
from IPython import embed


class PlotterClass():
    """docstring foPlotterClassme"""

    def __init__(self):

        self.top_output = rospy.get_param(rospy.get_name() + '/top_output', '/ekf_output_srv')
        self.top_mu_est = rospy.get_param(rospy.get_name() + '/top_mu_est', '/nu_estimated')
        self.top_mu_real = rospy.get_param(rospy.get_name() + '/top_mu_real', '/nu_real')
        self.new_data = []
        self.data_rcv = False
        self.map = False

        # Map plot service
        self.map_server = rospy.Service(self.top_output, plot_map, self.plot_map_cb)
        #  mu coordinates plotting subs
        self.mu_est_subs = rospy.Subscriber(self.top_mu_est, Float32MultiArray, self.mu_est_cb)
        #  mu_real coordinates plotting subs
        self.mu_real_subs = rospy.Subscriber(self.top_mu_real, Float32MultiArray, self.mu_real_cb)

        color_lm = 'ro'
        color_trj = 'bs'

        plt.ion()
        fig, ax = plt.subplots()
        plot = ax.scatter([], [])
        ax.set_xlim(-5, 25)
        ax.set_ylim(-5, 10)

        sleeper = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.data_rcv:
                # embed()
                array = plot.get_offsets()
                array = np.append(array, self.new_data)
                # TODO_NACHO: empty new_data with mutex
                if self.map:
                    plot.set_offsets(array)
                    self.map = False
                else:
                    plot.set_offsets(array)
                fig.canvas.draw()
                self.data_rcv = False
            # self.new_data = []
            sleeper.sleep()

        plt.close('all')
        plt.ioff()

    def plot_map_cb(self, srv):
        i = 0
        x = []
        y = []

        for j in range(0, len(srv.map_landmarks) / 2):
            x.append(srv.map_landmarks[i])
            y.append(srv.map_landmarks[i + 1])
            i = i + 2

        self.new_data = np.vstack((x, y))
        self.new_data = np.transpose(self.new_data)
        self.data_rcv = True
        self.map = True

        return True

    def mu_est_cb(self, msg):
        # embed()
        self.new_data = np.vstack((self.new_data, [msg.data[0], msg.data[1]]))
        self.data_rcv = True

    def mu_real_cb(self, msg):
        # embed()
        self.new_data = np.vstack((self.new_data, [msg.data[0], msg.data[1]]))
        self.data_rcv = True


if __name__ == "__main__":
    rospy.init_node("output_node")
    PlotterClass()
