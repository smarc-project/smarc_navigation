
import rosbag

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation

inbag = rosbag.Bag('_2018-01-24-15-02-58.bag', 'r')

ekf_positions = []
gt_positions = []
time_stamps = []

for topic, msg, t in inbag.read_messages():
    if topic == "/lolo_auv/ekf_odom":
        ekf_positions.append(msg.pose.pose.position)
    elif topic == "/lolo_auv/gt_in_odom":
    	gt_positions.append(msg.pose.pose.position)
    	time_stamps.append(msg.header.stamp)

error_x = []
error_y = []
error_z = []
tk = []
i = 0
for position in gt_positions:
	error_x.append(position.x - ekf_positions[i].x)
	error_y.append(position.y - ekf_positions[i].y)
	error_z.append(position.y - ekf_positions[i].z)
	i = i+1
	tk.append(i)

plt.figure(1, figsize=(11, 20))

plt.subplot(311)
plt.title('Error in x position (m)', fontsize=26)
plt.plot(tk, error_x)

plt.subplot(312)
plt.title('Error in y position (m)', fontsize=26)
plt.plot(tk, error_y)

plt.subplot(313)
plt.title('Error in z position (m)', fontsize=26)
plt.plot(tk, error_z)
plt.xlabel('Time steps', fontsize=26)

plt.show()
