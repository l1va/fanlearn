#!/usr/bin/env python
import rospy
import sys
from helper_classes import *
from fl_compvis.srv import *
from fl_compvis.msg import *
from fl_learning.srv import *
from std_msgs.msg import String

goal_tolerance = 15

def is_goal_achieved():
    dist = sqrt((self.goal.x - self.position.x)**2 +
                    (self.goal.y - self.position.y)**2)
    return self.dist_to_goal() < self.tolerance

def main():
    rospy.init_node('learning_control', anonymous=True)
    rospy.sleep(0.5)
    global p
    p = get_pose()

    print('Learning control mode ...')

    rospy.wait_for_service('Computervision')
    compvis = rospy.ServiceProxy('Computervision', GetCoords)

    rospy.wait_for_service('determine_action')
    determine_action = rospy.ServiceProxy('determine_action',       DetermineAction)

    cv_resp = compvis()
    x,y = cv_resp.brick

    p = get_pose()

    rate = rospy.Rate(10)
    while True:
        if is_goal_achieved():
            break  # TODO finish logic with getting new goal

        cv_resp = compvis()
        action = determine_action(cv_resp.tool.x / scale,
                                    cv_resp.tool.y / scale,
                                    cv_resp.brick.x / scale,
                                    cv_resp.brick.y / scale,
                                    brick.goal.x / scale,
                                    brick.goal.y / scale)
        print('Command from learning node: {}'.format(commands))
        if action == 0:
            p.x+=0.01
        elif action ==1:
            p.x-=0.01
        elif action ==2:
            p.x-=0.01
        elif action == 3:
            p.x -= 0.01
        else:
            print("unknown action")
        execute_pose(p)
        rate.sleep()


if __name__ == "__main__":
    main()
