#!/usr/bin/env python

import roslib
import rospy
import time
import math
import PID as pid
import tf.transformations
from geometry_msgs.msg import Twist
from dual_mc33926_rpi import motors, MAX_SPEED


def limit(num, minimum=-480, maximum=480):
    return max(min(num, maximum), minimum)

def set_motor_vel(vr, vl):
    global left_vel
    global right_vel

    kp = 1.2
    ki = 1.0
    kd = 0.001
    sample_time = 0.01

    pid_left = pid.pid(kp, ki, kd)
    pid_right = pid.pid(kp, ki, kd)

    pid_left.SetPoint = vl
    pid_right.SetPoint = vr

    pid_left.setSampleTime(sample_time)
    pid_right.setSampleTime(sample_time)

    pid_left.update(left_vel)
    pid_right.update(right_vel)

    vl_out = limit(pid_left.output)
    vr_out = limit(pid_right.output)

    # Then set your wheel speeds (using wheel_left and wheel_right as examples)
    try:
        motors.setSpeeds(vl_out, vr_out)

    finally:
        motors.setSpeeds(0, 0)
        motors.disable()

def encoder_callback(msg):
    global left_vel
    global right_vel

    left_vel = msg.linear.x
    right_vel = msg.linear.y

def cmd_callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    robot_width = 0.17

    vr = (msg.angular.z * robot_width) / 2.0 + msg.linear.x;
    vl = msg.linear.x * 2.0 - vr;

    set_motor_vel(vr, vl)

def listener():
    rospy.init_node('cmd_vel_subscriber')
    
    global left_vel
    global right_vel

    motors.enable()
    motors.setSpeeds(0, 0)
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)
    rospy.Subscriber("/ard_odom", Twist, encoder_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
