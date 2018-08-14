#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

pub = rospy.Publisher('control', Int32, queue_size=1)
def callback(msg):
	print(msg.data)
	pub.publish(2)


def talker():
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5)
    pub.publish(1)
    rospy.Subscriber("co",numpy_msg(Floats),callback)
    rospy.spin()

talker()

