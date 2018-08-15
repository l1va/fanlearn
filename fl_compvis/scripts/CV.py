#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from fl_compvis.srv import *
from fl_compvis.msg import *


xe=ye=xo=yo=0.0

cam=cv2.VideoCapture(0)
def find_red(im):    
    #im = bridge.imgmsg_to_cv2(image, "bgr8") 
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    #print(hsv[1000][600])
    rl1=(0,100,100)
    rh1=(10,255,255)
    mask1 = cv2.inRange(hsv,rl1,rh1)
    mask1 = cv2.dilate(mask1, None, iterations=2)
    mask1 = cv2.erode(mask1,None,iterations=3)
    rl2=(160,100,100)
    rh2=(179,255,255)
    mask2 = cv2.inRange(hsv,rl2,rh2)
    mask2 = cv2.erode(mask2,None,iterations=2)
    mask2 = cv2.dilate(mask2, None, iterations=3)
    mask=cv2.addWeighted(mask1,1.0,mask2,1.0,0.0)
    #cv2.imshow("1",mask) 
    #cv2.waitKey(30)     
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    M = cv2.moments(cnts[0])
    if M["m00"]<0.01:
	return 0,0
    x = int(M["m10"] / M["m00"])
    y = int(M["m01"] / M["m00"])
    return x,y

def detect(image):
	global xe,ye,xo,yo
	xo,yo=find_red(image)

def callback(req):
	global xe,ye,xo,yo
	#im=rospy.wait_for_message('pylon_camera_node/image_raw', Image)
	#rospy.Subscriber('pylon_camera_node/image_raw', Image,detect)
	g,im=cam.read()
	print(find_red(im))
	a=Coordinates(xe,ye,xo,yo)
    	return COMVResponse(a)
	
    
def listener():
	rospy.init_node('Computervision')
	s = rospy.Service('Computervision', COMV, callback)
	rate = rospy.Rate(5)
	rospy.spin()
    
bridge=CvBridge()
listener()
