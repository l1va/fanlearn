#!/usr/bin/env python
import rospy
import sys
from helper_classes import *
from fl_compvis.srv import *
from fl_compvis.msg import *
from fl_learning.srv import *
from std_msgs.msg import String

goal_tolerance = 15
scale = 6
HZ = 10


brick = Brick(goal_tolerance)
if len(sys.argv) > 1:
    if sys.argv[1] == 'Learning':
        contro_mode = 'Learning'
    elif sys.argv[1] == 'Joystick':
        contro_mode = 'Joystick'
else:
    contro_mode = 'Keyboard'

rospy.init_node('control', anonymous=True)
rospy.sleep(0.5)

fanuc = Fanuc()

def GP_handler(data):
    c = data.data
    if c == 'x+':
        fanuc.tool.goal.x += 0.01
    elif c == 'x-':
        fanuc.tool.goal.x -= 0.01
    elif c == 'y+':
        fanuc.tool.goal.y += 0.01
    elif c == 'y-':
        fanuc.tool.goal.y -= 0.01
    elif c == 'z+':
        fanuc.tool.goal.z += 0.01
        print('z+')
    elif c == 'z-':
        fanuc.tool.goal.z -= 0.01
    elif c == 'qx+':
        fanuc.tool.goal.q.x += 0.01
    elif c == 'qx-':
        fanuc.tool.goal.q.x -= 0.01
    elif c == 'qy+':
        fanuc.tool.goal.q.y += 0.01
    elif c == 'qy-':
        fanuc.tool.goal.q.y -= 0.01
    elif c == 'qz+':
        fanuc.tool.goal.q.z += 0.01
    elif c == 'qz-':
        fanuc.tool.goal.q.z -= 0.01
    elif c == 'qw+':
        fanuc.tool.goal.q.w += 0.01
    elif c == 'qw-':
        fanuc.tool.goal.q.w -= 0.01
    print(fanuc.tool)
    fanuc.go()

def main():

    if contro_mode == 'Learning':
        print('Learning control mode ...')

        print('Wait for computer vision node...')
        rospy.wait_for_service('Computervision')
        compvis = rospy.ServiceProxy('Computervision', COMV)

        print('Wait for learning node ...')
        rospy.wait_for_service('determine_action')
        determine_action = rospy.ServiceProxy('determine_action',
                                              DetermineAction)

        cv_resp = compvis(real_or_simulation='real')
        brick.goal = cv_resp.brick

        rate = rospy.Rate(HZ)
        while True:
            if brick.is_goal_achieved():
                cv_resp = compvis(real_or_simulation='real')
                break  # TODO finish logic with getting new goal
            else:
                cv_resp = compvis(real_or_simulation='real')
                commands = determine_action(cv_resp.tool.x / scale,
                                            cv_resp.tool.y / scale,
                                            cv_resp.brick.x / scale,
                                            cv_resp.brick.y / scale,
                                            brick.goal.x / scale,
                                            brick.goal.y / scale)
                print('Command from learning node: {}'.format(commands))
        rate.sleep()

    elif contro_mode == 'Joystick':
        print('Joystick control mode ...')
        while True:
            rospy.Subscriber("Gamepad_wasd", String, GP_handler)
            rospy.spin()


    else: # Keyboard control
        print('Keyboard control mode ...')
        while True:
            rate = rospy.Rate(HZ)  # 10hz
            c = raw_input('input\n')
            if c == 'w':
                fanuc.tool.goal.x += 0.01
            elif c == 's':
                fanuc.tool.goal.x -= 0.01
            elif c == 'a':
                fanuc.tool.goal.y += 0.01
            elif c == 'd':
                fanuc.tool.goal.y -= 0.01
            elif c == 'e':
                fanuc.tool.goal.z += 0.01
            elif c == 'q':
                fanuc.tool.goal.z -= 0.01
            elif c == 'W':
                fanuc.tool.goal.q.x += 0.01
            elif c == 'S':
                fanuc.tool.goal.q.x -= 0.01
            elif c == 'A':
                fanuc.tool.goal.q.y += 0.01
            elif c == 'D':
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

            rate.sleep()

if __name__ == "__main__":
    main()
