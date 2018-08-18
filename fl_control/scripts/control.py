#!/usr/bin/env python
import rospy
import sys
from fl_compvis.srv import *
from fl_compvis.msg import *
from fl_learning.srv import *
from std_msgs.msg import String
from build_action import *
from geometry_msgs.msg import Pose
from math import floor
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

def execute_action_service():
    rospy.Service('execute_action', ExecuteAction, execute_action)

def execute_action(action):
    if action.action == 0:
        p.position.x += 0.11
    elif action.action == 1:
        p.position.y -= 0.11
    elif action.action == 2:
        p.position.x -= 0.11
    elif action.action == 3:
        p.position.y += 0.11
        
    execute_pose(p, planning_time=5)

def is_goal_achieved():
    dist = sqrt((self.goal.x - self.position.x) ** 2 +
                (self.goal.y - self.position.y) ** 2)
    return self.dist_to_goal() < self.tolerance


def main():
    rospy.init_node('learning_control', anonymous=True)
    rospy.sleep(0.5)

    execute_action_service()

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
    #x_goal, y_goal = (400.0, 400.0)
    scale = 72
    x_goal, y_goal = floor(x_goal/scale),floor( y_goal/scale)    
    print("goal: " ,x_goal, y_goal)

    c = raw_input('move the brick to start position and press Enter\n')

    while True:
        cv_resp = compvis()
        if not cv_resp.success:
            print("cannot get coordinates for start pos")
            return
        scale = 72
        xb, yb = floor(cv_resp.brick.x/scale), floor(cv_resp.brick.y/scale)
        print(xb, yb)
        xt, yt = floor(cv_resp.tool.x/scale), floor(cv_resp.tool.y/scale)
        print(xt, yt)

        action = determine_action(xt,yt,xb,yb,x_goal,y_goal)
        print('Action - {}'.format(action))

        execute_action(action)
        
        c = raw_input('for next step press Enter\n')



if __name__ == "__main__":
    main()
