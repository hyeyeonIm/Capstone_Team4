#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i/k: increase/decrease forward speed
j/l: turn left/right
,: stop

CTRL-C to quit
"""

speed = 0.0144
turn = 0.411

moveBindings = {
    'i': (speed, 0),
    'k': (-speed, 0),
    'j': (0, turn),
    'l': (0, -turn),
    ',': (0, 0),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('keyboard_control_node')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    current_velocity = Twist()  # Initialize with zero velocity

    try:
        print(msg)
        while True:
            key = getKey()
            if key in moveBindings.keys():
                vel = moveBindings[key]
                current_velocity.linear.x = vel[0]
                current_velocity.angular.z = vel[1]
            elif key == '\x03':  # Handle CTRL-C
                break

            pub.publish(current_velocity)  # Publish the current velocity continuously

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
