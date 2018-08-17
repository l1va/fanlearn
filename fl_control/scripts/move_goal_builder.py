from geometry_msgs.msg import *
from moveit_msgs.msg import (Constraints, JointConstraint, PositionConstraint,
                             OrientationConstraint, BoundingVolume)
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from shape_msgs.msg import SolidPrimitive
from tf.listener import TransformListener
import actionlib
import copy
import moveit_msgs.msg
import rospy
import tf


class MoveItGoalBuilder(object):
    """Builds a MoveGroupGoal.
    Example::
        # To do a reachability check from the current robot pose.
        builder = MoveItGoalBuilder()
        builder.set_pose_goal(pose_stamped)
        builder.allowed_planning_time = 5
        builder.plan_only = True
        goal = builder.build()
        # To move to a current robot pose with a few options changed.
        builder = MoveItGoalBuilder()
        builder.set_pose_goal(pose_stamped)
        builder.replan = True
        builder.replan_attempts = 10
        goal = builder.build()
    Use ``set_pose_goal`` or ``set_joint_goal`` to set the actual goal. Pose
    goals and joint goals are mutually exclusive; setting one will overwrite the
    other.
    Most of the builder is configured by setting public members.
    Below is a list of all the public members you can set as well as their
    default values:
    allowed_planning_time = 10.0
        float. How much time to allow the planner, in seconds.
    fixed_frame = 'base_link'
        string. The MoveIt planning frame.
    gripper_frame = 'wrist_roll_link'
        string. The name of the end-effector link.
    group_name = 'arm'
        string. The name of the MoveIt group to plan for.
    planning_scene_diff
        moveit_msgs.msg.PlanningScene. Changes to the planning scene to apply.
        By default this is set to an empty planning scene.
    look_around = False
        bool. Whether or not the robot can look around during execution.
    max_acceleration_scaling_factor = 0
        float. The scaling factor for the maximum joint acceleration, in the
        range [0, 1]. A value of 0 means there is no scaling factor.
    max_velocity_scaling_factor = 0
        float. Used to slow the executed action, this specifies the scaling
        factor for the maximum velocity, in the range [0, 1]. A value of 0
        means there is no scaling factor.
    num_planning_attempts = 1
        int. How many times to compute the same plan (most planners are
        randomized). The shortest plan will be used.
    plan_only = False
        bool. Whether to only compute the plan but not execute it.
    planner_id = 'RRTConnectkConfigDefault'
        string. The name of the planner to use. A list of planner names can be
        found using the rviz MotionPlanning plugin.
    replan = False
        bool. Whether to come up with a new plan if there was an error
        executing the original plan.
    replan_attempts = 5
        int. How many times to do replanning.
    replan_delay = 1.0
        float. How long to wait in between replanning, in seconds.
    tolerance = 0.01:
        float. The tolerance radius, in meters, for the goal pose. If the
        builder is used for a joint angle action, then tolerance specifies the
        joint angle tolerance (both above and below) in radians.
    """

    def __init__(self):
        self.allowed_planning_time = 10.0
        self.fixed_frame = 'base_link'
        self.gripper_frame = 'wrist_roll_link'
        self.group_name = 'arm'
        self.planning_scene_diff = moveit_msgs.msg.PlanningScene()
        self.planning_scene_diff.is_diff = True
        self.planning_scene_diff.robot_state.is_diff = True
        self.look_around = False
        self.max_acceleration_scaling_factor = 0
        self.max_velocity_scaling_factor = 0
        self.num_planning_attempts = 1
        self.plan_only = False
        self.planner_id = 'RRTConnectkConfigDefault'
        self.replan = False
        self.replan_attempts = 5
        self.replan_delay = 1.0
        self.start_state = moveit_msgs.msg.RobotState()
        self.start_state.is_diff = True
        self.tolerance = 0.01
        self._orientation_constraints = []
        self._pose_goal = None
        self._joint_names = None
        self._joint_positions = None
        self._tf_listener = TransformListener()

    def set_pose_goal(self, pose_stamped):
        """Sets a pose goal.
        .. note:: The pose will be transformed into the planning frame when
            ``build`` is called.
        :param pose_stamped: The pose goal for the end-effector.
        :type pose_stamped: geometry_msgs.msg.PoseStamped
        """
        self._pose_goal = pose_stamped
        self._joint_names = None
        self._joint_positions = None

    def set_joint_goal(self, joint_names, joint_positions):
        """Sets a joint-space planning goal.
        :param joint_names: The names of the joints in the goal.
        :type joint_names: list of strings
        :param joint_positions: The joint angles to achieve.
        :type joint_positions: list of floats
        """
        self._joint_names = joint_names
        self._joint_positions = joint_positions
        self._pose_goal = None

    def add_path_orientation_constraint(self, o_constraint):
        """Adds an orientation constraint to the path.
        .. note:: Make more advanced edits to the orientation constraints by
            modifying _orientation_constraints directly.
        :param o_constraint: The orientation constraint to add.
        :type o_constraint: moveit_msgs.msg.OrientationConstraint
        """
        self._orientation_constraints.append(copy.deepcopy(o_constraint))

    def build(self, tf_timeout=rospy.Duration(5.0)):
        """Builds the goal message.
        To set a pose or joint goal, call ``set_pose_goal`` or
        ``set_joint_goal`` before calling ``build``. To add a path orientation
        constraint, call ``add_path_orientation_constraint`` first.
        :param tf_timeout: How long to wait for a TF message. Only used with
            pose goals.
        :type tf_timeout: rospy.Duration
        :return: The MoveGroup action goal.
        :rtype: moveit_msgs.msg.MoveGroupGoal
        """
        goal = MoveGroupGoal()

        # Set start state
        goal.request.start_state = copy.deepcopy(self.start_state)

        # Set goal constraint
        if self._pose_goal is not None:
            self._tf_listener.waitForTransform(self._pose_goal.header.frame_id,
                                               self.fixed_frame,
                                               rospy.Time.now(), tf_timeout)
            try:
                pose_transformed = self._tf_listener.transformPose(
                    self.fixed_frame, self._pose_goal)
            except (tf.LookupException, tf.ConnectivityException):
                return None
            c1 = Constraints()
            c1.position_constraints.append(PositionConstraint())
            c1.position_constraints[0].header.frame_id = self.fixed_frame
            c1.position_constraints[0].link_name = self.gripper_frame
            b = BoundingVolume()
            s = SolidPrimitive()
            s.dimensions = [self.tolerance * self.tolerance]
            s.type = s.SPHERE
            b.primitives.append(s)
            b.primitive_poses.append(pose_transformed.pose)
            c1.position_constraints[0].constraint_region = b
            c1.position_constraints[0].weight = 1.0

            c1.orientation_constraints.append(OrientationConstraint())
            c1.orientation_constraints[0].header.frame_id = self.fixed_frame
            c1.orientation_constraints[
                0].orientation = pose_transformed.pose.orientation
            c1.orientation_constraints[0].link_name = self.gripper_frame
            c1.orientation_constraints[
                0].absolute_x_axis_tolerance = self.tolerance
            c1.orientation_constraints[
                0].absolute_y_axis_tolerance = self.tolerance
            c1.orientation_constraints[
                0].absolute_z_axis_tolerance = self.tolerance
            c1.orientation_constraints[0].weight = 1.0

            goal.request.goal_constraints.append(c1)
        elif self._joint_names is not None:
            c1 = Constraints()
            for i in range(len(self._joint_names)):
                c1.joint_constraints.append(JointConstraint())
                c1.joint_constraints[i].joint_name = self._joint_names[i]
                c1.joint_constraints[i].position = self._joint_positions[i]
                c1.joint_constraints[i].tolerance_above = self.tolerance
                c1.joint_constraints[i].tolerance_below = self.tolerance
                c1.joint_constraints[i].weight = 1.0
            goal.request.goal_constraints.append(c1)

        # Set path constraints
        goal.request.path_constraints.orientation_constraints = self._orientation_constraints

        # Set trajectory constraints

        # Set planner ID (name of motion planner to use)
        goal.request.planner_id = self.planner_id

        # Set group name
        goal.request.group_name = self.group_name

        # Set planning attempts
        goal.request.num_planning_attempts = self.num_planning_attempts

        # Set planning time
        goal.request.allowed_planning_time = self.allowed_planning_time

        # Set scaling factors
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling_factor

        # Set planning scene diff
        goal.planning_options.planning_scene_diff = copy.deepcopy(
            self.planning_scene_diff)

        # Set is plan only
        goal.planning_options.plan_only = self.plan_only

        # Set look around
        goal.planning_options.look_around = self.look_around

        # Set replanning options
        goal.planning_options.replan = self.replan
        goal.planning_options.replan_attempts = self.replan_attempts
        goal.planning_options.replan_delay = self.replan_delay

        return goal