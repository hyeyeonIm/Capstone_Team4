#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelHandler:
    def __init__(self):
        self.last_command = None
        self.pub = rospy.Publisher('motor_command', String, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        command = "stop"  # Default command

        if linear_x > 0.01:
            command = "go"
        elif linear_x < -0.01:
            command = "back"
        elif angular_z > 0.4:
            command = "left"
        elif angular_z < -0.4:
            command = "right"

        # Check if the command has changed from the last command
        if command != self.last_command:
            # rospy.loginfo(f"Received cmd_vel message: linear_x={linear_x}, angular_z={angular_z}")
            # rospy.loginfo(f"Publishing command: {command}")
            self.pub.publish(command)
            self.last_command = command  # Update the last command

if __name__ == '__main__':
    rospy.init_node('cmd_vel_subscriber')
    CmdVelHandler()
    rospy.spin()
