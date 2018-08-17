#!/usr/bin/env python
from fl_learning.srv import *
import rospy
import random


def determine_action_callback(req):
    action = random_action(req.end_effector_x, req.end_effector_y, req.block_x, req.block_y, req.goal_x, req.goal_y)
    return DetermineActionResponse(action)

def random_action(Ex, Ey, Bx, By, Gx, Gy):
    action = random.randint(0,3)
    print (Ex, Ey, Bx, By, Gx, Gy, action)
    return action

def main():
    rospy.init_node('learning')
    s = rospy.Service('determine_action', DetermineAction, determine_action_callback)
    rospy.spin()

if __name__ == "__main__":
    main()

