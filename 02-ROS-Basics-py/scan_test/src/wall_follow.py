#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def go_straight(speed):
    speed = min(MAX_LINEAR, speed)
    data.linear.x = speed


def turn(clockwise, speed):
    speed = min(MAX_ANGULAR, speed)
    if clockwise == "cw":
        data.angular.z = speed
    elif clockwise == "acw":
        data.angular.z = -speed
    else:
        data.angular.z = 0


def pubnow():
    pub.publish(data)


MAX_LINEAR = 0.19
MAX_ANGULAR = 0.49
data = Twist()


def callback(msg):
    readings = msg.ranges
    # 0 back
    # 180 right
    # 360 front
    # 540 left
    # RIGHT side
    if (readings[360] > 0.5):
        go_straight(0.1)
        if (readings[180] > 0.3):
            go_straight(0.05)
            turn("acw", 0.15)
        if (readings[180] < 0.2):
            go_straight(0.05)
            turn("cw", 0.15)
    elif (readings[360] < 0.5):
        go_straight(0)
        turn("cw", 0.4)
    pubnow()


rospy.init_node("wall")
sub = rospy.Subscriber("/scan", LaserScan, callback)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rospy.spin()
