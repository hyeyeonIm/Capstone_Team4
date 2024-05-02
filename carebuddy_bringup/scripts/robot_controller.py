#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String


current_command = "stop"

rospy.init_node('robot_controller')

motor_pub = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

def handle_motor_command(command):
    global current_command
    current_command = command  

def publish_motor_state():
    global current_command
    twist = Twist()
    joint_state = JointState()
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    
    if current_command == "go":
        twist.linear.x = 5.0
        joint_state.velocity = [1.0, 1.0]
    elif current_command == "back":
        twist.linear.x = -5.0
        joint_state.velocity = [-1.0, -1.0]
    elif current_command == "left":
        twist.angular.z = 5.0
        joint_state.velocity = [0.5, -0.5]
    elif current_command == "right":
        twist.angular.z = -5.0
        joint_state.velocity = [-0.5, 0.5]
    elif current_command == "stop":
        twist.linear.x = 0
        twist.angular.z = 0
        joint_state.velocity = [0, 0]

    motor_pub.publish(twist)
    joint_pub.publish(joint_state)

def motor_command_listener():
    rospy.Subscriber('/motor_command', String, lambda msg: handle_motor_command(msg.data))
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        publish_motor_state()  
        rate.sleep()

if __name__ == '__main__':
    motor_command_listener()

