#!/usr/bin/env python
import cv2
import rospy
from fl_compvis.srv import *
from fl_compvis.msg import *

def main():
    rospy.wait_for_service('Computervision')
    compvis = rospy.ServiceProxy('Computervision', COMV)
    while True:
	cor = compvis(2)
        print(cor)
main()
