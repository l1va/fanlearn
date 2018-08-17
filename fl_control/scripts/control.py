#!/usr/bin/env python


import rospy
from helper_classes import Fanuc

def main():
    rospy.init_node('control', anonymous=True)
    rospy.sleep(0.5)

    fanuc = Fanuc()
    c = ''
    while c != chr(27):
        c = raw_input('input\n')
        if c == 'd':
            fanuc.tool.goal.x += 0.01
        elif c == 'a':
            fanuc.tool.goal.x -= 0.01
        elif c == 'w':
            fanuc.tool.goal.y += 0.01
        elif c == 's':
            fanuc.tool.goal.y -= 0.01
        elif c == 'e':
            fanuc.tool.goal.z += 0.01
        elif c == 'q':
            fanuc.tool.goal.z -= 0.01
        elif c == 'D':
            fanuc.tool.goal.q.x += 0.01
        elif c == 'A':
            fanuc.tool.goal.q.x -= 0.01
        elif c == 'W':
            fanuc.tool.goal.q.y += 0.01
        elif c == 'S':
            fanuc.tool.goal.q.y -= 0.01
        elif c == 'E':
            fanuc.tool.goal.q.z += 0.01
        elif c == 'Q':
            fanuc.tool.goal.q.z -= 0.01
        elif c == 'C':
            fanuc.tool.goal.q.w += 0.01
        elif c == 'Z':
            fanuc.tool.goal.q.w -= 0.01

        print(fanuc.tool)
        fanuc.go()


if __name__ == "__main__":
    main()