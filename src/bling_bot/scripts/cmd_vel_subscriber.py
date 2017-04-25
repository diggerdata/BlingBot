#!/usr/bin/env python

import rospy
import time
import math
import PID as pid
import tf
from geometry_msgs.msg import Twist
from dual_mc33926_rpi import motors, MAX_SPEED


def limit(num, minimum=-480, maximum=480):
    return max(min(num, maximum), minimum)

def set_motor_vel():
    global left_vel
    global right_vel
    global last_time
    global vl
    global vr
    global pid_left
    global pid_right

    pid_left.SetPoint = vl
    pid_right.SetPoint = vr

    pid_left.update(left_vel)
    pid_right.update(right_vel)

    if vr == 0.0 and vl == 0.0:
        vl_out = 0.0
        vr_out = 0.0
    else:
        vl_out = -1 * limit(pid_left.output)
        vr_out = limit(pid_right.output)

    # rospy.loginfo("error: [{0}, {1}]".format(vr - right_vel, vl - left_vel))

    try:
        motors.setSpeeds(vl_out, vr_out)

    except:
        motors.setSpeeds(0, 0)
        motors.disable()

def encoder_callback(msg):
    global left_vel
    global right_vel

    left_vel = msg.linear.y
    right_vel = msg.linear.x

    set_motor_vel()

def cmd_callback(msg):
    global vl
    global vr

    robot_width = 0.17

    vr = (msg.angular.z * robot_width) / 2.0 + msg.linear.x;
    vl = msg.linear.x * 2.0 - vr;

    # rospy.loginfo("velocities: [{0}, {1}]".format(vr, vl))

def main():
    rospy.init_node('cmd_vel_subscriber')

    global first_time
    global left_vel
    global right_vel
    global last_time
    global vl
    global vr
    global pid_left
    global pid_right

    kp = 1.0
    ki = 1700.0
    kd = 0.5
    sample_time = 0.009

    pid_left = pid.pid(kp, ki, kd)
    pid_right = pid.pid(kp, ki, kd)

    # pid_left.setSampleTime(sample_time)
    # pid_right.setSampleTime(sample_time)

    vl = 0.0
    vr = 0.0

    left_vel = 0.0
    right_vel = 0.0

    first_time = True

    if first_time:
        rospy.loginfo("cmd_vel_subscriber setup!")

    motors.enable()
    motors.setSpeeds(0, 0)

    rospy.Subscriber("/ard_odom", Twist, encoder_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)

    first_time = False
    rospy.spin()
    #
    # r = rospy.Rate(100)
    # while not rospy.is_shutdown():
    #
    #     r.sleep()

if __name__ == '__main__':
    main()
