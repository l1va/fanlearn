#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy

x=y=ox=oy=tx=ty=0.0
pub = rospy.Publisher('co', numpy_msg(Floats), queue_size=6)

def find_red(image):
    im = bridge.imgmsg_to_cv2(image, "bgr8") 
    print(im.shape)
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
    cv2.imshow("1",mask) 
    cv2.waitKey()     
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    M = cv2.moments(cnts[0])
    if M["m00"]<0.01:
	return 0,0
    x = int(M["m10"] / M["m00"])
    y = int(M["m01"] / M["m00"])
    return x,y

def detect(image):
	global x,y,ox,oy,tx,ty
	tx,ty=find_red(image)
	a = numpy.array([x,y,ox,oy,tx,ty], dtype=numpy.float32)
	pub.publish(a)

def track(image):
	global x,y,ox,oy,tx,ty
	ox,oy=find_red(image)
	a = numpy.array([x,y,ox,oy,tx,ty], dtype=numpy.float32)
	pub.publish(a)

def callback(msg):
	print(msg.data)
	if msg.data==1:
		rospy.Subscriber('pylon_camera_node/image_raw', Image, detect)
	else:
		rospy.Subscriber('pylon_camera_node/image_raw', Image, track)
    
def listener():
    rospy.init_node('CV', anonymous=True)
    rate = rospy.Rate(5)
    rospy.Subscriber("control",Int32,callback)
    rospy.spin()
    
bridge=CvBridge()
listener()
