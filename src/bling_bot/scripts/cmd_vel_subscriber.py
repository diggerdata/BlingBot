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

    vl_out = -1 * limit(pid_left.output)
    vr_out = limit(pid_right.output)
    # vl_out = vl * -480
    # vr_out = vr * 480

    # Then set your wheel speeds (using wheel_left and wheel_right as examples)
    try:
        out_str = "Setting motors: {0}, {1}".format(vr_out, vl_out)
        rospy.logwarn(out_str)
        motors.setSpeeds(vl_out, vr_out)

    except:
        motors.setSpeeds(0, 0)
        motors.disable()

def encoder_callback(msg):
    global left_vel
    global right_vel

    # rospy.logwarn("encoder_callback")

    left_vel = msg.linear.x
    right_vel = msg.linear.y

def cmd_callback(msg):
    global vl
    global vr

    rospy.logwarn("Received a /cmd_vel message!")
    # rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    # rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    robot_width = 0.17

    vr = (msg.angular.z * robot_width) / 2.0 + msg.linear.x;
    vl = msg.linear.x * 2.0 - vr;

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

    kp = 500
    ki = 0.01
    kd = 0.001
    sample_time = 0.01

    pid_left = pid.pid(kp, ki, kd)
    pid_right = pid.pid(kp, ki, kd)

    pid_left.setSampleTime(sample_time)
    pid_right.setSampleTime(sample_time)

    vl = 0.0
    vr = 0.0

    left_vel = 0.0
    right_vel = 0.0

    first_time = True

    if first_time:
        rospy.loginfo("cmd_vel_subscriber setup!")

    motors.enable()
    motors.setSpeeds(0, 0)
    rospy.logwarn("Got past /cmd_vel Subscriber!")
    rospy.Subscriber("/ard_odom", Twist, encoder_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)

    first_time = False

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        set_motor_vel()
        r.sleep()

if __name__ == '__main__':
    main()
