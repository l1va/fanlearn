#!/usr/bin/env python
import cv2
import rospy
from fl_compvis.srv import *
from fl_compvis.msg import *
from fl_learning.srv import *

xe=ye=xo=yo=xt=yt=0.0


def reset(cor):
	global xt,yt
	input("change the position")
	x_goal=cor.brick.x
	y_goal=cor.brick.y

def main():
    goal_tolrerance=15
    scale=6
    real_or_simulation="real"
    global x_tool,y_tool,x_brick,y_brick,x_goal,y_goal
    rospy.wait_for_service('Computervision')
    compvis = rospy.ServiceProxy('Computervision', COMV)
    rospy.wait_for_service('determine_action')
    determine_action = rospy.ServiceProxy('determine_action', DetermineAction)
    while True:
        if(abs(x_goal-x_brick)<goal_tolrerance and abs(y_goal-y_brick)<goal_tolrerance):
            cor = compvis(real_or_simulation="real")
	    print(cor)
            reset(cor)
        else:
            cor=compvis(real_or_simulation="real")
	    print(cor)
            commands = determine_action(cor.tool.x/scale, cor.tool.y/scale, cor.brick.x/scale, cor.brick.y/scale, x_goal/scale, y_goal/scale)
            print(commands)

main()
