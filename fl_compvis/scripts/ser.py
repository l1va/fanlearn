#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from camera_control_msgs.msg import GrabImagesAction, GrabImagesGoal
from sensor_msgs.msg import Image
import actionlib
from fl_compvis.srv import *
from fl_compvis.msg import *
from camera_control_msgs import *


xe=ye=xo=yo=0.0
def find_coord(im):
    global xe,ye,xo,yo
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    #print(hsv[1000][600])
    rl1=(0,100,100)
    rh1=(10,255,255)
    mask1 = cv2.inRange(hsv,rl1,rh1)
    mask1 = cv2.dilate(mask1, None, iterations=4)
    mask1= cv2.erode(mask1,None,iterations=12)
    rl2=(160,100,100)
    rh2=(179,255,255)
    mask2 = cv2.inRange(hsv,rl2,rh2)
    mask2 = cv2.dilate(mask1, None, iterations=4)
    mask2= cv2.erode(mask1,None,iterations=12)
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
    gl=(45,100,50)
    gh=(85,255,255)
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


def callback(req):
    global xe,ye,xo,yo
    bridge=CvBridge()
    client = actionlib.SimpleActionClient("~/pylon_camera_node/grab_images_raw", GrabImagesAction)
    if not client.wait_for_server(rospy.Duration.from_sec(10.0)):
        print("error")
    goal=GrabImagesGoal()
    goal.exposure_given = True
    goal.exposure_times = [rospy.get_param('~exposure_time', 16416)]
    goal.gain_given = True
    goal.gain_values = [0]
    goal.gain_auto = False
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration.from_sec(10.0)):
        print("error")
    result=client.get_result()
    im = bridge.imgmsg_to_cv2(result.images[0], "bgr8")
    im=im[85:690,405:1115]
    find_coord(im)
    a=Coordinates(xe,ye,xo,yo)
    return COMVResponse(a)

#tracker = cv2.TrackerCSRT_create() 
#success=tracker.init(frame,bbox)
rospy.init_node('Computervision')
s = rospy.Service('Computervision', COMV, callback)
rate=rospy.Rate(5)
rospy.spin()
