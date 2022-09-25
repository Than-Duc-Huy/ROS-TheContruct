#! /usr/bin/env python


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


data = Twist()


def callback(msg):
    readings = msg.ranges  # List of data (right)0-719(left)
    middleReading = readings[360]
    print(middleReading)
    rightReading = readings[0]
    leftReading = readings[719]

    if (middleReading < 1.0):
        data.angular.z = 1.5
    else:
        data.linear.x = 0.2
        data.angular.z = 0

    if (rightReading < 1):
        data.angular.z = 1.5
    elif (leftReading < 1):
        data.angular.z = -1

    pub.publish(data)


rospy.init_node("node")
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber("/kobuki/laser/scan", LaserScan, callback)

rospy.spin()
