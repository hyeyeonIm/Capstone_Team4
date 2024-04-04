#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i/k: forward/backward
j/l: left/right
,: stop

CTRL-C to quit
"""

moveBindings = {
        'i': 'go',
        'k': 'back',
        'j': 'left',
        'l': 'right',
        ',': 'stop',
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

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('keyboard_control_node')
    pub = rospy.Publisher('motor_command', String, queue_size=1)
    
    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                command = moveBindings[key]
                pub.publish(command)
            elif key == '\x03':
                break

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

