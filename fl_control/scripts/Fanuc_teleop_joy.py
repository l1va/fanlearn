#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

# This ROS Node converts Joystick inputs from the joy node into commands for Fanuc

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into String commands
# axis 1 aka left stick vertical controls forward/backward
# axis 0 aka left stick horizonal controls right/left
def callback(data):
    print('callback')
    msg = String("0")

    if data.buttons[6] == 1:
        if data.buttons[0] == 1:
            msg.data = "qz+"
        elif data.buttons[1] == 1:
            msg.data = "qw-"
        elif data.buttons[2] == 1:
            msg.data = "qz-"
        elif data.buttons[3] == 1:
            msg.data = "qw+"
        elif data.axes[1] == -1:
            msg.data = "z-"
        elif data.axes[1] == 1:
            msg.data = "z+"
    else:
        if data.axes[0] == -1:
            msg.data = "y-"
        elif data.axes[0] == 1:
            msg.data = "y+"
        elif data.axes[1] == -1:
            msg.data = "x-"
        elif data.axes[1] == 1:
            msg.data = "x+"
	    print('x++')		

        elif data.buttons[0] == 1:
            msg.data = "qx+"
        elif data.buttons[1] == 1:
            msg.data = "qy-"
        elif data.buttons[2] == 1:
            msg.data = "qx-"
        elif data.buttons[3] == 1:
            msg.data = "qy+"
    if msg.data != '0':
        pub.publish(msg)



    # Intializes everything
def start():
     # publishing to 'Gamepad_wasd' to control Fanuc
    global pub
    rospy.init_node('Gamepad_node')
    pub = rospy.Publisher('Gamepad_wasd', String, queue_size=5)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.spin()

if __name__ == '__main__':
        start()
