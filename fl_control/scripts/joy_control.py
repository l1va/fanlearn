#!/usr/bin/env python
import rospy
from build_action import *
from geometry_msgs.msg import Pose
from std_msgs.msg import String


p = Pose()

def joy_handler(data):
    c = data.data
    if c == 'x+':
        p.position.x += 0.05
    elif c == 'x-':
        p.position.x -= 0.05
    elif c == 'y+':
        p.position.y += 0.05
    elif c == 'y-':
        p.position.y -= 0.05
    elif c == 'z+':
        p.position.z += 0.05
    elif c == 'z-':
        p.position.z -= 0.05
    elif c == 'qx+':
        p.orientation.x += 0.03
    elif c == 'qx-':
        p.orientation.x -= 0.03
    elif c == 'qy+':
        p.orientation.y += 0.03
    elif c == 'qy-':
        p.orientation.y -= 0.03
    elif c == 'qz+':
        p.orientation.z += 0.03
    elif c == 'qz-':
        p.orientation.z -= 0.03
    elif c == 'qw+':
        p.orientation.w += 0.03
    elif c == 'qw-':
        p.orientation.w -= 0.03
    execute_pose(p)

def main():
    rospy.init_node('joy_control', anonymous=True)
    rospy.sleep(0.5)
    global p
    p = get_pose()
    rospy.Subscriber("gamepad_wasd", String, joy_handler)
    rospy.spin()

if __name__ == "__main__":
    main()
