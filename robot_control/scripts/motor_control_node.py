#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

PWMA = 18
AIN1 = 22
AIN2 = 27
PWMB = 23
BIN1 = 25
BIN2 = 24

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)

L_Motor = GPIO.PWM(PWMA, 100)
R_Motor = GPIO.PWM(PWMB, 100)
L_Motor.start(0)
R_Motor.start(0)

def motor_control(command):
    speed = 50
    if command == "go":
        GPIO.output(AIN1, False)
        GPIO.output(AIN2, True)
        GPIO.output(BIN1, False)
        GPIO.output(BIN2, True)
        L_Motor.ChangeDutyCycle(speed)
        R_Motor.ChangeDutyCycle(speed)
    elif command == "back":
        GPIO.output(AIN1, True)
        GPIO.output(AIN2, False)
        GPIO.output(BIN1, True)
        GPIO.output(BIN2, False)
        L_Motor.ChangeDutyCycle(speed)
        R_Motor.ChangeDutyCycle(speed)
    elif command == "left":
        GPIO.output(AIN1, True)
        GPIO.output(AIN2, False)
        GPIO.output(BIN1, False)
        GPIO.output(BIN2, True)
        L_Motor.ChangeDutyCycle(speed)
        R_Motor.ChangeDutyCycle(speed)
    elif command == "right":
        GPIO.output(AIN1, False)
        GPIO.output(AIN2, True)
        GPIO.output(BIN1, True)
        GPIO.output(BIN2, False)
        L_Motor.ChangeDutyCycle(speed)
        R_Motor.ChangeDutyCycle(speed)
        
    elif command == "stop":
        L_Motor.ChangeDutyCycle(0)
        R_Motor.ChangeDutyCycle(0)

def callback(data):
    rospy.loginfo("Received command: %s", data.data)
    motor_control(data.data)

def listener():
    rospy.init_node('motor_control_node')
    rospy.Subscriber('motor_command', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        L_Motor.stop()
        R_Motor.stop()
        GPIO.cleanup()



