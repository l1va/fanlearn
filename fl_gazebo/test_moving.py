#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
#joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
#'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def move_to(pub, positions, velocities, times):
 
    jt = JointTrajectory()
    jt.joint_names = joint_names
    jt.header.stamp = rospy.Time.now()
 
    for (position, velocity, time) in zip(positions, velocities, times):
        jtp = JointTrajectoryPoint()
        jtp.positions = position
        jtp.velocities = velocity
        jtp.time_from_start = rospy.Duration(time)
        jt.points.append(jtp)
 
    pub.publish(jt)
    rospy.loginfo("%s: starting %.2f sec traj", "self.controller_name", times[-1])


def main():
	rospy.init_node('tester', anonymous=True)

	pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
	rospy.sleep(0.5)

	positions = [[0.1,0.1,0.1,0.1,0.1,0.1],
	[0.2,0.2,0.2,0.2,0.2,0.2]]
	velocities = [[1,1,1,1,1,1],[1,1,1,1,1,1]]
	times = [9,19]

	move_to(pub, positions, velocities, times)


main()