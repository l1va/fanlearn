#!/usr/bin/env python
import sys
import rospy
import actionlib

from move_goal_builder import MoveItGoalBuilder

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import MoveGroupActionGoal, MoveGroupAction, ExecuteTrajectoryAction,ExecuteTrajectoryActionGoal
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

class Quartets:
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Coordinates:
    def __init__(self, x=0.55, y=0.0, z=0.93, q_x=0.707, q_y=0.0, q_z=0.707, q_w=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.q = Quartets(q_x, q_y, q_z, q_w)

    def __str__(self):
        return '({} {} {}) - quartets: ({} {} {} {})'.format(self.x, self.y,
                                                             self.z, self.q.x,
                                                             self.q.y, self.q.z,
                                                             self.q.w)

joint_names = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']

speed = 3
time = 2

goal = PoseStamped()
goal.header.frame_id = "/base_link"
goal.pose = Pose()
goal.pose.position.x = 0.55
goal.pose.position.z=0.93
goal.pose.orientation.x=0.707
goal.pose.orientation.z=0.707

def move_to(pubs, positions, velocities, times):
    jt = JointTrajectory()
    jt.joint_names = joint_names
    jt.header.stamp = rospy.Time.now()

    for (position, velocity, time) in zip(positions, velocities, times):
        jtp = JointTrajectoryPoint()
        jtp.positions = position
        jtp.velocities = velocity
        jtp.time_from_start = rospy.Duration(time)
        jt.points.append(jtp)

    for pub in pubs:
        pub.publish(jt)
        rospy.loginfo("%s: starting %.2f sec traj", "self.controller_name",
                      times[-1])

def move_to_point(pubs, joint_positions, speed, time):
    jt = JointTrajectory()
    jt.joint_names = joint_names
    jt.header.stamp = rospy.Time.now()

    jtp = JointTrajectoryPoint()
    jtp.positions = joint_positions
    jtp.velocities = [speed] * 6
    jtp.time_from_start = rospy.Duration(time)
    jt.points.append(jtp)

    for pub in pubs:
        pub.publish(jt)
        rospy.loginfo("%s: starting %.2f sec traj", "self.controller_name", time)


def parse_position_resp(pos_resp):
    if pos_resp.error_code.val == MoveItErrorCodes.SUCCESS:
        print('OK')
        return pos_resp.solution.joint_state.position
    elif pos_resp.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
        print('No inverse kinematis solution')
    else:
        print('IK Error: {}'.format(pos_resp.error_code.val))


def solve_ik(posestamped):
    rospy.wait_for_service('compute_ik')
    try:
        print("try to solve ik...")
        request = GetPositionIKRequest()
        request.ik_request.group_name = "manipulator"
        request.ik_request.ik_link_name = "tool0"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped = posestamped
        # request.ik_request.pose_stamped.header.frame_id = "/base_link"
        # request.ik_request.pose_stamped.pose.position.x = goal.x
        # request.ik_request.pose_stamped.pose.position.y = goal.y
        # request.ik_request.pose_stamped.pose.position.z = goal.z
        # request.ik_request.pose_stamped.pose.orientation.x = goal.q.x
        # request.ik_request.pose_stamped.pose.orientation.y = goal.q.y
        # request.ik_request.pose_stamped.pose.orientation.z = goal.q.z
        # request.ik_request.pose_stamped.pose.orientation.w = goal.q.w
        ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        resp = ik(request)
        return parse_position_resp(resp)

    except rospy.ServiceException as e:
        print('Service call failed - {}'.format(e))


def main():
    rospy.init_node('tester', anonymous=True)

    sim_pub = rospy.Publisher('/arm_controller/command', JointTrajectory,
                          queue_size=10)

    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)

    fanuc_pub = rospy.Publisher('/joint_trajectory_action/goal', JointTrajectory,
                          queue_size=10)
    rospy.sleep(0.5)

    #go to initial position 
    position = solve_ik(goal)
    if position != None:
        move_to_point([sim_pub, fanuc_pub], position, speed, time)

    jtp = JointTrajectoryActionGoal()
    print(jtp)
    c = ''
    while c != chr(27):
        c = raw_input('input\n')
        if c == 'd':
            goal.x += 0.01
        elif c == 'a':
            goal.x -= 0.01
        elif c == 'w':
            goal.y += 0.01
        elif c == 's':
            goal.y -= 0.01
        elif c == 'e':
            goal.z += 0.01
        elif c == 'q':
            goal.z -= 0.01
        elif c == 'D':
            goal.q.x += 0.01
        elif c == 'A':
            goal.q.x -= 0.01
        elif c == 'W':
            goal.q.y += 0.01
        elif c == 'S':
            goal.q.y -= 0.01
        elif c == 'E':
            goal.q.z += 0.01
        elif c == 'Q':
            goal.q.z -= 0.01
        elif c == 'C':
            goal.q.w += 0.01
        elif c == 'Z':
            goal.q.w -= 0.01


        print(goal)

        position = solve_ik(goal)
        if position != None:
            move_to_point([sim_pub, fanuc_pub], position, speed, time)

def main2():
    #initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    fanuc = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = 'manipulator'
    group = moveit_commander.MoveGroupCommander(group_name)

    #trajectories for RViz to visualize
    # display_trajectory_publisher = rospy.Publisher(
    #     '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
    #     queue_size=20)

    # #for debug
    # planning_frame = group.get_planning_frame()
    # print('Reference frame: {}\n'.format(planning_frame))
    # print('Fanuc state: {}\n'.format(fanuc.get_current_state))

    # #initial pose - all zeros
    # joint_goal = group.get_current_joint_values()
    # joint_goal[] = 0

    # group.go(joint_goal, wait=True)
    # group.stop()

    #planning to a pose goal
    c = ''
    while c != chr(27):
        c = raw_input('input\n')
        if c == 'd':
            goal.x += 0.01
        elif c == 'a':
            goal.x -= 0.01
        elif c == 'w':
            goal.y += 0.01
        elif c == 's':
            goal.y -= 0.01
        elif c == 'e':
            goal.z += 0.01
        elif c == 'q':
            goal.z -= 0.01
        elif c == 'D':
            goal.q.x += 0.01
        elif c == 'A':
            goal.q.x -= 0.01
        elif c == 'W':
            goal.q.y += 0.01
        elif c == 'S':
            goal.q.y -= 0.01
        elif c == 'E':
            goal.q.z += 0.01
        elif c == 'Q':
            goal.q.z -= 0.01
        elif c == 'C':
            goal.q.w += 0.01
        elif c == 'Z':
            goal.q.w -= 0.01

        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.frame_id = "/base_link"
        pose_goal.orientation.x = goal.q.x
        pose_goal.orientation.y = goal.q.y
        pose_goal.orientation.z = goal.q.z
        pose_goal.orientation.w = goal.q.w
        pose_goal.position.x = goal.x
        pose_goal.position.y = goal.y
        pose_goal.position.z = goal.z

        group.set_pose_target(pose_goal)

        print(goal)

        plan = group.go(wait=True)
        rospy.sleep(2)
        try:
            group.execute(plan, wait=True)
        except:
            print('Nope')
        rospy.sleep(2)
        group.stop()
        group.clear_pose_targets()


def main3():
    rospy.init_node('tester', anonymous=True)
    rospy.sleep(0.5)

    sim_pub = rospy.Publisher('/arm_controller/command', JointTrajectory,
                          queue_size=10)

    # fanuc_pub = rospy.Publisher('/move_group/goal', MoveGroupActionGoal,
    #                       queue_size=10)

    fanuc_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)

    fanuc_client.wait_for_server()

    move_group_goal = MoveGroupActionGoal()
    builder = MoveItGoalBuilder()
    builder.fixed_frame = '/base_link'
    builder.gripper_frame = 'tool0'
    builder.group_name = 'manipulator'

    c = ''
    while c != chr(27):
        c = raw_input('input\n')
        if c == 'd':
            goal.pose.position.x += 0.01
        elif c == 'a':
            goal.pose.position.x -= 0.01
        elif c == 'w':
            goal.pose.position.y += 0.01
        elif c == 's':
            goal.pose.position.y -= 0.01
        elif c == 'e':
            goal.pose.position.z += 0.01
        elif c == 'q':
            goal.pose.position.z -= 0.01
        elif c == 'D':
            goal.pose.orientation.x += 0.01
        elif c == 'A':
            goal.pose.orientation.x -= 0.01
        elif c == 'W':
            goal.pose.orientation.y += 0.01
        elif c == 'S':
            goal.pose.orientation.y -= 0.01
        elif c == 'E':
            goal.pose.orientation.z += 0.01
        elif c == 'Q':
            goal.pose.orientation.z -= 0.01
        elif c == 'C':
            goal.pose.orientation.w += 0.01
        elif c == 'Z':
            goal.pose.orientation.w -= 0.01


        goal.header = Header()
        goal.header.frame_id = "/base_link"
        print(goal)
        position = solve_ik(goal)
        if position != None:
            builder.set_joint_goal(joint_names, position)
            builder.allowed_planning_time = time
            #builder.plan_only = True
            # move_group_goal.goal = builder.build()

            #fanuc_client.send_goal(move_group_goal)
            js = rospy.wait_for_message('/joint_states', JointState)
            #builder.start_state.joint_state = js
            js.header.frame_id = "/base_link"
            #print(js)
            #builder.start_state.is_diff = False
            builder.start_state.multi_dof_joint_state.header.frame_id = "/base_link"

            tosend = builder.build()
            tosend.request.workspace_parameters.header = js.header

            fanuc_client.send_goal(tosend)
            fanuc_client.wait_for_result()


def main4():
    rospy.init_node('tester', anonymous=True)
    rospy.sleep(0.5)

    sim_pub = rospy.Publisher('/arm_controller/command', JointTrajectory,
                          queue_size=10)

    # fanuc_pub = rospy.Publisher('/move_group/goal', MoveGroupActionGoal,
    #                       queue_size=10)

    fanuc_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)

    fanuc_client.wait_for_server()

    move_group_goal = MoveGroupActionGoal()
    builder = MoveItGoalBuilder()
    builder.fixed_frame = '/base_link'
    builder.gripper_frame = 'tool0'
    builder.group_name = 'manipulator'

    c = ''
    while c != chr(27):
        c = raw_input('input\n')
        if c == 'd':
            goal.pose.position.x += 0.01
        elif c == 'a':
            goal.pose.position.x -= 0.01
        elif c == 'w':
            goal.pose.position.y += 0.01
        elif c == 's':
            goal.pose.position.y -= 0.01
        elif c == 'e':
            goal.pose.position.z += 0.01
        elif c == 'q':
            goal.pose.position.z -= 0.01
        elif c == 'D':
            goal.pose.orientation.x += 0.01
        elif c == 'A':
            goal.pose.orientation.x -= 0.01
        elif c == 'W':
            goal.pose.orientation.y += 0.01
        elif c == 'S':
            goal.pose.orientation.y -= 0.01
        elif c == 'E':
            goal.pose.orientation.z += 0.01
        elif c == 'Q':
            goal.pose.orientation.z -= 0.01
        elif c == 'C':
            goal.pose.orientation.w += 0.01
        elif c == 'Z':
            goal.pose.orientation.w -= 0.01


        goal.header = Header()
        goal.header.frame_id = "/base_link"
        print(goal)
        position = solve_ik(goal)
        if position != None:
            builder.set_joint_goal(joint_names, position)
            builder.allowed_planning_time = time
            #builder.plan_only = True
            # move_group_goal.goal = builder.build()

            #fanuc_client.send_goal(move_group_goal)
            js = rospy.wait_for_message('/joint_states', JointState)
            #builder.start_state.joint_state = js
            js.header.frame_id = "/base_link"
            #print(js)
            #builder.start_state.is_diff = False
            builder.start_state.multi_dof_joint_state.header.frame_id = "/base_link"

            tosend = builder.build()
            tosend.request.workspace_parameters.header = js.header

            fanuc_client.send_goal(tosend)
            fanuc_client.wait_for_result()

if __name__ == "__main__":
    main4()