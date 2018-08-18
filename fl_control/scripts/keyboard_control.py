#!/usr/bin/env python
import rospy
from build_action import *
from geometry_msgs.msg import Pose

p = Pose()


def run():
    while True:
        c = raw_input('input\n')
        if c == 'w':
            p.position.x += 0.01
        elif c == 's':
            p.position.x -= 0.01
        elif c == 'a':
            p.position.y += 0.01
        elif c == 'd':
            p.position.y -= 0.01
        elif c == 'e':
            p.position.z += 0.01
        elif c == 'q':
            p.position.z -= 0.01
        elif c == 'W':
            p.orientation.x += 0.01
        elif c == 'S':
            p.orientation.x -= 0.01
        elif c == 'A':
            p.orientation.y += 0.01
        elif c == 'D':
            p.orientation.y -= 0.01
        elif c == 'E':
            p.orientation.z += 0.01
        elif c == 'Q':
            p.orientation.z -= 0.01
        elif c == 'C':
            p.orientation.w += 0.01
        elif c == 'Z':
            p.orientation.w -= 0.01
        else:
            print("wrong command:", c)
            continue
        execute_pose(p)


def main():
    rospy.init_node('keyboard_control', anonymous=True)
    rospy.sleep(0.5)
    global p
    p = get_pose()
    run()


if __name__ == "__main__":
    main()
