#!/usr/bin/env python
from fl_learning.srv import *
import rospy
import random

def determine_action(req):
    action = random_action(req.end_effector_x, req.end_effector_y, req.block_x, req.block_y, req.goal_x, req.goal_y)
    return DetermineActionResponse(action)

def determine_action_service():
    rospy.init_node('learning')
    s = rospy.Service('determine_action', DetermineAction, determine_action)
    rospy.spin()

def random_action(Ex, Ey, Bx, By, Gx, Gy):
    action = random.randint(0,3)
    print (Ex, Ey, Bx, By, Gx, Gy, action)
    return action

if __name__ == "__main__":
    determine_action_service()

