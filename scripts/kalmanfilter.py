#!/usr/bin/env python
import rospy  
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from pedsim_msgs.msg import AgentStates, AgentState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import numpy as np
import math


def callback(msg):
    points = PoseArray()
    points.header = msg.header

    # make kalman filter for every object
    for i in msg.agent_states:
        if i.id in kf:
            continue
        else:
            kf[i.id] = [KalmanFilter(dim_x=2, dim_z=1), KalmanFilter(dim_x=2, dim_z=1)]

            kf[i.id][0].F = np.array([[1.,0.1],[0.,1.]])
            kf[i.id][0].H = np.array([[1.,0.]])
            kf[i.id][0].P *= 1000.
            kf[i.id][0].R = 5
            kf[i.id][0].Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)

            kf[i.id][1].F = np.array([[1.,0.1],[0.,1.]])
            kf[i.id][1].H = np.array([[1.,0.]])
            kf[i.id][1].P *= 1000.
            kf[i.id][1].R = 5
            kf[i.id][1].Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)
    
    # for every object, predict possible positions
    for i in msg.agent_states:
        # rospy.loginfo('id: %d\n\tpose - x:%f, y:%f\n\ttwist - x:%f, y:%f\n\n', i.id, i.pose.position.x, i.pose.position.y, i.twist.linear.x, i.twist.linear.y)

        kf[i.id][0].x = np.array([i.pose.position.x, i.twist.linear.x])
        kf[i.id][1].x = np.array([i.pose.position.y, i.twist.linear.y])
        for j in range(prediction_length):
            kf[i.id][0].predict()
            kf[i.id][0].update(kf[i.id][0].x[0], kf[i.id][0].x[1])
            kf[i.id][1].predict()
            kf[i.id][1].update(kf[i.id][1].x[0], kf[i.id][1].x[1])
            point = Pose()
            point.position.x = kf[i.id][0].x[0]
            point.position.y = kf[i.id][1].x[0]
            point.orientation = i.pose.orientation
            points.poses.append(point)    
    
    # d1 = math.sqrt(math.pow(points.poses[0].position.x - points.poses[19].position.x, 2) + math.pow(points.poses[0].position.y - points.poses[19].position.y, 2))
    # print(d1)

    kalman_path = Path()
    kalman_path.header = msg.header
    for pose in points.poses:
        temp_pose = PoseStamped()
        temp_pose.header = msg.header
        temp_pose.pose = pose
        kalman_path.poses.append(temp_pose)
    
    path_pub.publish(kalman_path)
    pointarray_pub.publish(points)

# if __name__ == '__main__':
rospy.init_node('kalmanfilter_node',anonymous=False)
pedsim_sub = rospy.Subscriber("/pedsim_simulator/simulated_agents",AgentStates, callback) 
pointarray_pub = rospy.Publisher("filtered_points", PoseArray, queue_size=1000)
path_pub = rospy.Publisher("filtered_path", Path, queue_size=1000)
kf = {}

length_factor = rospy.get_param("/length_factor")
prediction_length = int(length_factor*5)

rospy.spin()