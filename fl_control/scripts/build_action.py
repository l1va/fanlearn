# TODO: REMOVE such *** code
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction
from actionlib.simple_action_client import SimpleActionClient
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFKRequest, GetPositionFK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import rospy
from fl_control.scripts.moveit_builder import MoveItGoalBuilder

builder = MoveItGoalBuilder()
action_client = SimpleActionClient('move_group', MoveGroupAction)
sim_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
link_names = ["link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]


def get_pose():  # TODO: test me
    js = rospy.wait_for_message("/joint_states", JointState)
    pose = solve_fk(js)
    return pose


def execute_pose(pose, planning_time=0.2):
    positions = solve_ik(pose)
    if positions != None:
        # real
        builder.set_joint_goal(joint_names, positions)
        builder.allowed_planning_time = planning_time
        builder.plan_only = False
        to_send = builder.build()
        action_client.send_goal(to_send)
        action_client.wait_for_result()

        # simulation
        jt = JointTrajectory()
        jt.joint_names = joint_names
        jt.header.stamp = rospy.Time.now()
        jtp = JointTrajectoryPoint()
        jtp.positions = positions
        jtp.velocities = [0.5] * 6
        jtp.time_from_start = rospy.Duration(planning_time)
        jt.points.append(jtp)
        sim_pub.publish(jt)


def solve_fk(js):  # TODO: test me
    rospy.wait_for_service('compute_fk')
    try:
        print("try to solve fk...")
        request = GetPositionFKRequest()
        request.fk_link_names = link_names
        request.header.frame_id = "/base_link"
        request.robot_state.joint_state = js

        fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        resp = fk(request)
        return parse_fk_resp(resp)

    except rospy.ServiceException as e:
        print('Service call failed - {}'.format(e))


def parse_fk_resp(resp):
    if resp.error_code.val == MoveItErrorCodes.SUCCESS:
        print('OK')
        return resp.pose_stamped[-1].pose
    print('FK Error: {}'.format(resp.error_code.val))
    return None


def solve_ik(pose):
    rospy.wait_for_service('compute_ik')
    try:
        print("try to solve ik...")
        request = GetPositionIKRequest()
        request.ik_request.group_name = "manipulator"
        request.ik_request.ik_link_name = "link_6"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "/base_link"
        request.ik_request.pose_stamped.pose = pose

        ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        resp = ik(request)
        return parse_ik_resp(resp)

    except rospy.ServiceException as e:
        print('Service call failed - {}'.format(e))


def parse_ik_resp(pos_resp):
    if pos_resp.error_code.val == MoveItErrorCodes.SUCCESS:
        print('OK')
        return pos_resp.solution.joint_state.position
    if pos_resp.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
        print('No inverse kinematis solution')
        return None
    print('IK Error: {}'.format(pos_resp.error_code.val))
    return None
