#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Image
from fl_compvis.srv import *
from fl_compvis.msg import *
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def find_coords(img):
    img = bridge.imgmsg_to_cv2(img, "bgr8")
    # if topic=="pylon_camera_node/image_raw":
    #      im=im[85:690,405:1115]
    # else:
    #      im=im[238:860,233:971]
    img = img[85:690,405:1115]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    brick = get_brick_coords(hsv)
    if brick is None:
        return None

    tool = get_tool_coords(hsv)

    return (brick, tool)

def get_brick_coords(hsv):
    rl1 = (0, 100, 100)
    rh1 = (20, 255, 255)
    mask1 = cv2.inRange(hsv, rl1, rh1)
    mask1 = cv2.dilate(mask1, None, iterations=6)
    mask1 = cv2.erode(mask1, None, iterations=12)
    rl2 = (160, 100, 100)
    rh2 = (179, 255, 255)
    mask2 = cv2.inRange(hsv, rl2, rh2)
    mask2 = cv2.dilate(mask2, None, iterations=6)
    mask2 = cv2.erode(mask2, None, iterations=12)
    mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0.0)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    # print(len(cnts))
    if len(cnts) < 1:
        print("!!! counts of contours less than 1")
        return None
    M = cv2.moments(cnts[0])
    if M["m00"] < 0.01:
        print("!!! moment00 is less than 0.01 ")
        return None
    xo = int(M["m10"] / M["m00"])
    yo = int(M["m01"] / M["m00"])
    return Coordinates(xo, yo)

def get_tool_coords(hsv):
    gl = (40, 100, 40)
    gh = (79, 255, 255)
    mask1 = cv2.inRange(hsv, gl, gh)
    mask1 = cv2.dilate(mask1, None, iterations=4)
    m = cv2.erode(mask1, None, iterations=12)
    xe = ye = 0
    # TODO: Where are the comments of next magic?
    for i in range(m.shape[0] - 1, 0, -1):
        for j in range(m.shape[1]):
            if m[i, j] == 255:
                ye = i
                x1 = j
                while j < m.shape[1] and m[i, j] == 255:
                    j += 1
                xe = (x1 + j) / 2.0
                break
    return Coordinates(xe,ye)

def get_coords_callback(empty):
    img = rospy.wait_for_message("/fanlearn_camera/image_raw", Image, timeout=3)
    coords = find_coords(img)
    if coords is None:
        return GetCoordsResponse(False, None, None)
    return GetCoordsResponse(True, coords[1], coords[0])


def main():
    rospy.init_node('get_coords', anonymous=True)
    rospy.Service('get_coordinates', GetCoords, get_coords_callback)
    rospy.spin()


main()
