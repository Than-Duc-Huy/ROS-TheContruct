#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class Control():
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()

    def move_straight(self, speed=0.2):
        self.twist.linear.x = speed
        self.send()

    def turn(self, speed=0.2):
        self.twist.angular.z = speed
        self.send()

    def send(self):
        self.pub.publish(self.twist)
