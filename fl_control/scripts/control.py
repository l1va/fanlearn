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
init_pose.position.z = 0.44
init_pose.orientation.x = -0.29
init_pose.orientation.y = 0.65
init_pose.orientation.z = 0.25
init_pose.orientation.w = 0.65


def is_goal_achieved():
    dist = sqrt((self.goal.x - self.position.x) ** 2 +
                (self.goal.y - self.position.y) ** 2)
    return self.dist_to_goal() < self.tolerance


def main():
    rospy.init_node('learning_control', anonymous=True)
    rospy.sleep(0.5)

    c = raw_input('press Enter when all is loaded\n')
    execute_pose(init_pose, planning_time=10)
    global p
    p = init_pose
    print('Learning control mode ...')

    rospy.wait_for_service('get_coordinates')
    compvis = rospy.ServiceProxy('get_coordinates', GetCoords)

    rospy.wait_for_service('determine_action')
    determine_action = rospy.ServiceProxy('determine_action', DetermineAction)

    print("calling compvis")
    cv_resp = compvis()
    if not cv_resp.success:
        print("cannot get coordinates")
        return
    x_goal, y_goal = cv_resp.brick.x, cv_resp.brick.y
    print("goal: " ,x_goal, y_goal)
    #x_goal, y_goal = (400.0, 400.0)

    c = raw_input('move the brick to start position and press Enter\n')

    while True:
        cv_resp = compvis()
        if not cv_resp.success:
            print("cannot get coordinates for start pos")
            return
        x, y = cv_resp.brick.x, cv_resp.brick.y
        print(x, y)

        scale = 6
        action = determine_action(cv_resp.tool.x / scale,
                                  cv_resp.tool.y / scale,
                                  cv_resp.brick.x / scale,
                                  cv_resp.brick.y / scale,
                                  x_goal / scale,
                                  y_goal / scale).action

        if action == 0:
            p.position.y -= 0.11
        elif action == 1:
            p.position.x += 0.11
        elif action == 2:
            p.position.y += 0.11
        elif action == 3:
            p.position.x -= 0.11

        execute_pose(p, planning_time=1)
        c = raw_input('for next step press Enter\n')

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
