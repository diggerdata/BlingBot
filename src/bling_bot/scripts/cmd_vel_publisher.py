#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

def main():
    rospy.init_node('cmd_vel_publisher')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(Twist(Vector3(0.5,0,0),Vector3(0,0,0)))
        r.sleep()

# Run the program
if __name__ == "__main__":
    main()
