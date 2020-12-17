#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist


class JoyToTwist():
    def __init__(self):

        rospy.init_node("joy_to_twist", log_level=rospy.INFO)
        rospy.loginfo("Started joy_to_twist node.")

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)

    def joy_callback(self, msg):
        """
        Convert joy  msg to twist msg

        axes[0] - joystick side to side (1.0 : -1.0)
        axes[1] - joystick up to down (1.0 : -1.0)
        """

        x_axis = msg.axes[0]
        y_axis = msg.axes[1]

        max_vel = 2.0
        max_yaw_rate = 1.0

        x = max_vel * y_axis
        yaw_rate = max_yaw_rate * x_axis

        vel = Twist()
        vel.linear.x = x
        vel.angular.z = yaw_rate

        self.vel_pub.publish(vel)


if __name__ == "__main__":

    translate = JoyToTwist()
    rospy.spin()
