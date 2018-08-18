#!/usr/bin/env python
import rospy
import sys
from fl_compvis.srv import *
from fl_compvis.msg import *
from fl_learning.srv import *
from std_msgs.msg import String
from build_action import *
from geometry_msgs.msg import Pose

goal_tolerance = 15

p = Pose()

init_pose = Pose()
init_pose.position.x = 0.41
init_pose.position.y = 0
init_pose.position.z = 0.39
init_pose.orientation.x = -0.29
init_pose.orientation.y = 0.65
init_pose.orientation.z = 0.25
init_pose.orientation.w = 0.65


def is_goal_achieved():
    dist = sqrt((self.goal.x - self.position.x)**2 +
                    (self.goal.y - self.position.y)**2)
    return self.dist_to_goal() < self.tolerance

def main():
    rospy.init_node('learning_control', anonymous=True)
    rospy.sleep(0.5)

    c = raw_input('press Enter when all is loaded\n')
    execute_pose(init_pose, planning_time=5)
    global p
    p = init_pose
    print('Learning control mode ...')

    rospy.wait_for_service('Computervision')
    compvis = rospy.ServiceProxy('Computervision', GetCoords)

    rospy.wait_for_service('determine_action')
    determine_action = rospy.ServiceProxy('determine_action',       DetermineAction)

    # print("calling compvis")
    # cv_resp = compvis()
    # x,y = cv_resp.brick
    # print(x,y)
    #
    # p.position.x = x
    # p.position.y = y
    # execute_pose(p, planning_time=5)

    # rate = rospy.Rate(10)
    # while True:
    #     if is_goal_achieved():
    #         break  # TODO finish logic with getting new goal
    #
    #     cv_resp = compvis()
    #     action = determine_action(cv_resp.tool.x / scale,
    #                                 cv_resp.tool.y / scale,
    #                                 cv_resp.brick.x / scale,
    #                                 cv_resp.brick.y / scale,
    #                                 brick.goal.x / scale,
    #                                 brick.goal.y / scale)
    #     print('Command from learning node: {}'.format(commands))
    #     if action == 0:
    #         p.x+=0.01
    #     elif action ==1:
    #         p.x-=0.01
    #     elif action ==2:
    #         p.x-=0.01
    #     elif action == 3:
    #         p.x -= 0.01
    #     else:
    #         print("unknown action")
    #     execute_pose(p)
    #     rate.sleep()


if __name__ == "__main__":
    main()
