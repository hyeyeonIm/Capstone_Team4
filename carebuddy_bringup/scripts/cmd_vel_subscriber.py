#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def cmd_vel_callback(msg, pub):
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    command = "stop"  # Default command

    if linear_x > 0.05:
        command = "go"
    elif linear_x < -0.05:
        command = "back"
    elif angular_z > 0.05:
        command = "right"
    elif angular_z < -0.05:
        command = "left"

    # rospy.loginfo(f"Received cmd_vel message: linear_x={linear_x}, angular_z={angular_z}")
    # rospy.loginfo(f"Publishing command: {command}")
    pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_subscriber')
    pub = rospy.Publisher('motor_command', String, queue_size=1)
    sub = rospy.Subscriber('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, cmd_vel_callback, pub)
    rospy.spin()
