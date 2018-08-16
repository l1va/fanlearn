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
	xt=cor.B.xo
	yt=cor.B.yo

def main():
    goal_tolrerance=15
    scale=6
    global xe,ye,xo,yo,xt,yt
    rospy.wait_for_service('Computervision')
    compvis = rospy.ServiceProxy('Computervision', COMV)
    rospy.wait_for_service('determine_action')
    determine_action = rospy.ServiceProxy('determine_action', DetermineAction)
    while True:
        if(abs(xo-xt)<goal_tolrerance and abs(yo-yt)<goal_tolrerance):
            cor = compvis(1)
            reset(cor)
        else:
            cor=compvis(1)
            commands = determine_action(cor.B.xe/scale, cor.B.ye/scale, cor.B.xo/scale, cor.B.yo/scale, xt/scale, yt/scale)
            print(commands)

main()
