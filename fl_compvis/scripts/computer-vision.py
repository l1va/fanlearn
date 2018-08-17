#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Image
from fl_compvis.srv import *
from fl_compvis.msg import *
import cv2
from cv_bridge import CvBridge, CvBridgeError


xe=ye=xo=yo=0.0
last_photo=Image()
last_photo.encoding="bgr8"
def find_coord(image):
    global xe,ye,xo,yo
    im = bridge.imgmsg_to_cv2(image, "bgr8")
    im=im[85:690,405:1115]
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    #print(hsv[1000][600])
    rl1=(0,100,100)
    rh1=(20,255,255)
    mask1 = cv2.inRange(hsv,rl1,rh1)
    mask1 = cv2.dilate(mask1, None, iterations=6)
    mask1= cv2.erode(mask1,None,iterations=12)
    rl2=(160,100,100)
    rh2=(179,255,255)
    mask2 = cv2.inRange(hsv,rl2,rh2)
    mask2 = cv2.dilate(mask2, None, iterations=6)
    mask2= cv2.erode(mask2,None,iterations=12)
    mask=cv2.addWeighted(mask1,1.0,mask2,1.0,0.0)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    print(len(cnts))
    if len(cnts)<1:
        return
    M = cv2.moments(cnts[0])
    if M["m00"] <0.01:
        return
    xo = int(M["m10"] / M["m00"])
    yo = int(M["m01"] / M["m00"])
    gl=(40,100,40)
    gh=(79,255,255)
    mask1 = cv2.inRange(hsv,gl,gh)
    mask1 = cv2.dilate(mask1, None, iterations=4)
    m= cv2.erode(mask1,None,iterations=12)
    xe=ye=y1=0
    for i in range(m.shape[0]-1,0,-1):
        for j in range(m.shape[1]):
            if m[i,j]==255:
                ye=i
                x1=j
                while j<m.shape[1] and m[i,j]==255:
                    j+=1
                xe=(x1+j)/2.0
                break

def take_photo(img):
    global last_photo
    last_photo=img

topic=""
def callback(req):
    global last_photo
    global topic
    if req.real_or_simulation == "real" and topic=="":
	topic="pylon_camera_node/image_raw"
        rospy.Subscriber(topic, Image, take_photo)
    elif req.real_or_simulation == "simulation" and topic=="":
	topic="/rrbot/camera1/image_raw"
	rospy.Subscriber(topic, Image, take_photo)
    else:
    	find_coord(last_photo)
    tool=Coordinates(xe,ye)
    brick=Coordinates(xo,yo)
    return COMVResponse(tool,brick)
    
def listener():
    rospy.init_node('CV', anonymous=True)
    rate = rospy.Rate(5)
    s = rospy.Service('Computervision', COMV, callback)
    rospy.spin()
    
bridge=CvBridge()
listener()
