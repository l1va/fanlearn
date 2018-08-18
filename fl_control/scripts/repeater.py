#!/usr/bin/env python
import rospy
import sys
from fl_compvis.srv import *
from fl_compvis.msg import *
from fl_learning.srv import *
from std_msgs.msg import String
from build_action import *
from geometry_msgs.msg import Pose

p = Pose()
p.position.x = 0.41
p.position.y = 0
p.position.z = 0.44
p.orientation.x = -0.29
p.orientation.y = 0.65
p.orientation.z = 0.25
p.orientation.w = 0.65


def convert(x, y):
    # it returns x in range 605, y in range 710 //85:690,405:1115
    # our robot works from 0.4 to 1 by x and from -0.6 to 0.6

    # it means for x: 0.4 - 0, 1 - 605. But half a table - it is not 600.
    xg = 0.41 + x * 1.5 / 1000.0
    # for y: -0.6 - 710, 0.6 - 0
    yg = -0.6 + (710 - y) * 1.2 / 710.0
    return xg, yg


def main():
    rospy.init_node('repeater', anonymous=True)
    rospy.sleep(0.5)

    c = raw_input('press Enter when all is loaded\n')
    execute_pose(p, planning_time=10)
    print('Learning control mode ...')

    rospy.wait_for_service('get_coordinates')
    compvis = rospy.ServiceProxy('get_coordinates', GetCoords)

    c = raw_input('move the brick to start position and press Enter\n')

    while True:
        cv_resp = compvis()
        if not cv_resp.success:
            print("cannot get coordinates for start pos")
            return
        x, y = cv_resp.brick.x, cv_resp.brick.y
        print(x, y)

        xg, yg = convert(x, y)
        print("converted:", xg, yg)
        p.position.x = xg
        p.position.y = yg
        execute_pose(p, planning_time=2)

        c = raw_input('for next step press Enter\n')


if __name__ == "__main__":
    main()
