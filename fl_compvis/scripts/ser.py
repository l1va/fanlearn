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
def find_red(im):
    global xo,yo
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    #print(hsv[1000][600])
    rl1=(0,100,100)
    rh1=(10,255,255)
    mask1 = cv2.inRange(hsv,rl1,rh1)
    mask1 = cv2.dilate(mask1, None, iterations=4)
    mask1= cv2.erode(mask1,None,iterations=10)
    rl2=(160,100,100)
    rh2=(179,255,255)
    mask2 = cv2.inRange(hsv,rl2,rh2)
    mask2 = cv2.dilate(mask1, None, iterations=4)
    mask2= cv2.erode(mask1,None,iterations=10)
    mask=cv2.addWeighted(mask1,1.0,mask2,1.0,0.0)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    print(len(cnts))
    if len(cnts)<1:
	return
    M = cv2.moments(cnts[0])
    xo = int(M["m10"] / M["m00"])
    yo = int(M["m01"] / M["m00"])

'''def track_ef(im):
    global xe,ye
    success, newbox = tracker.update(frame)
    if success:
	xe = (newbox[0]+newbox[2])/2.0;
        ye = (newbox[1]+newbox[3])/2.0;
        print(xe,ye)'''

def callback(req):
    bridge=CvBridge()
    client = actionlib.SimpleActionClient("/pylon_camera_node/grab_images_raw", GrabImagesAction)
    client.wait_for_server(rospy.Duration.from_sec(10.0))
    goal=GrabImagesGoal()
    goal.exposure_given = True
    goal.exposure_times = [rospy.get_param('~exposure_time', 16416)]
    goal.gain_given = True
    goal.gain_values = [0]
    goal.gain_auto = False
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))
    result=client.get_result()
    im = bridge.imgmsg_to_cv2(result.images[0], "bgr8") 
    find_red(im)
    track_ef(im)
    a=Coordinates(xe,ye,xo,yo)
    return COMVResponse(a)

#tracker = cv2.TrackerCSRT_create() 
#success=tracker.init(frame,bbox)
rospy.init_node('Computervision')
s = rospy.Service('Computervision', COMV, callback)
rate=rospy.Rate(5)
rospy.spin()

