#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


def getArduinoVels(msg):
    ael = msg.linear.x
    aer = msg.linear.y

    gyro_z = msg.angular.z
    generateMessage(aer, ael, gyro_z)

def generateMessage(aer, ael, gyro_z):
    global x
    global y
    global th

    global vx
    global vy
    global vth

    global prev_time

    # robot constants
    wheel_separation = 0.17  # meters
    odom_turn_multiplier = 1

    current_time = rospy.Time.now()
    dt = (current_time - prev_time).to_sec()

    prev_time = current_time

    delta_r = dt * aer
    delta_l = dt * ael
    delta_th = dt * gyro_z

    # wheel odometry
    delta_xy = (delta_r + delta_l) / 2.0
    # delta_th = ((delta_l - delta_r) / wheel_separation) * odom_turn_multiplier

    # imu odometry
    th =  th + delta_th

    # xy
    x = x + delta_xy * math.cos(th)
    y = y + delta_xy * math.sin(th)

    # velocity calculations
    vx = delta_xy/dt
    vth = delta_th/dt


def main():
    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    global x
    global y
    global th

    global vx
    global vth

    global prev_time

    aer = 0.0
    ael = 0.0

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0
    vy = 0.0
    vth = 0.0
    prev_time = rospy.Time.now()

    current_time = rospy.Time.now()

    ard_sub = rospy.Subscriber("/ard_odom", Twist, getArduinoVels, queue_size=100)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # compute odometry in a typical way given the velocities of the robot


        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))

        # print to console
        # console_out = "X {0} Y {1} Theta {2}".format(x, y, th)
        # rospy.loginfo(console_out)

        # publish the message
        odom_pub.publish(odom)

        r.sleep()

if __name__ == '__main__':
    main()
