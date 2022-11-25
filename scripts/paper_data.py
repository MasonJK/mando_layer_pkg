#!/usr/bin/env python

import rospy
import rosbag
import matplotlib.pyplot as plt
from matplotlib import gridspec
import numpy as np
from scipy.stats import gaussian_kde


bag = [rosbag.Bag('/home/junwoo/test_bags/0points/0-10.bag'), rosbag.Bag('/home/junwoo/test_bags/20points/20-7.bag')]
fig = plt.figure()
# ax1 = fig.add_subplot(2, 1, 1)
# ax2 = fig.add_subplot(2, 1, 2, sharex=ax1)

gs = gridspec.GridSpec(nrows=2, ncols=1, height_ratios=[10,10], 
                       width_ratios=[5])
ax1 = fig.add_subplot(gs[0])
ax1.figure.set_dpi(600)
ax2 = fig.add_subplot(gs[1], sharex=ax1)
ax2.figure.set_dpi(600)
for i in range(2):
    x = list()
    y = list()

    for topic, msg, t in bag[i].read_messages(topics=['/pedsim_simulator/simulated_agents']):
        for agent in msg.agent_states:
            x.append(agent.pose.position.x)
            y.append(agent.pose.position.y)
    xy = np.vstack([x,y])
    z = gaussian_kde(xy)(xy)

    robot_x = list()
    robot_y = list()

    for topic, msg, t in bag[i].read_messages(topics=['/robot/odom']):
        robot_x.append(msg.pose.pose.position.x)
        robot_y.append(msg.pose.pose.position.y)

    if i == 0:
        ax1.set_title('Baseline')
        ax1.scatter(np.array(y), np.array(x), c=z, s=100, edgecolor='', label='Obstacles')
        ax1.plot(np.array(robot_y), np.array(robot_x), 'b', linewidth='3', label='Robot')
        ax1.set(xlabel='X-Axis [m]',ylabel='Y-Axis [m]')
        ax1.legend(loc='upper right')
        ax1.set_xticks([0,5,10,15,20,25,30])
        ax1.set_yticks([0,5,10,15,20,25,30])
        ax1.set_xlim(0,30)
        ax1.set_ylim(0,30)
        ax1.grid(True)
    if i == 1:
        ax2.set_title('Proposed')
        ax2.scatter(np.array(y), np.array(x), c=z, s=100, edgecolor='', label='Obstacles')
        ax2.plot(np.array(robot_y), np.array(robot_x), 'r', linewidth='3', label='Robot')
        ax2.set(xlabel='X-Axis [m]',ylabel='Y-Axis [m]')
        ax2.legend(loc='upper right')
        ax2.set_xticks([0,5,10,15,20,25,30])
        ax2.set_yticks([0,5,10,15,20,25,30])
        ax2.set_xlim(0,30)
        ax2.set_ylim(0,30)
        ax2.grid(True)
    bag[i].close()

plt.subplots_adjust(left=0.125, bottom=0.1,  right=0.9, top=0.9, wspace=0.2, hspace=0.7)
plt.show()

