#!/usr/bin/env python
from fl_learning.srv import *
import rospy
import numpy as np
import os

# Get path of this script
script_path = os.path.dirname(os.path.abspath(__file__))

# Load Q-table
Q_table = np.load(script_path + '/Q_table.npy')

def determine_action(req):
    action = apply_optimal_policy(req)
    return DetermineActionResponse(action)

def determine_action_service():
    rospy.init_node('learning')
    s = rospy.Service('determine_action', DetermineAction, determine_action)
    rospy.spin()

def apply_optimal_policy(req):
    global Q_table
    return np.argmax(Q_table[req.end_effector_x, req.end_effector_y, req.block_x, req.block_y, req.goal_x, req.goal_y, :])    

if __name__ == "__main__":
    determine_action_service()
