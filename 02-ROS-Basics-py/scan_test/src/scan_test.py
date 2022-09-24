#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan


def callback(msg):
    print(msg.ranges[0], msg.ranges[180], msg.ranges[360], msg.ranges[540])
    # 0 back 
    # 180 right
    # 360 front
    # 540 left

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
