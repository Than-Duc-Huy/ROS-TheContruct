#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist  # Type/Class Int32

rospy.init_node("pub_cmd_vel")  # The launch file name will overwrite this line
# Must follow ROS base name, words_underscore
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)  # 2Hz
value = Twist()  # UNDERSTAND the type of the message!!!
value.linear.x = 1
value.angular.z = 1

while not rospy.is_shutdown():
    pub.publish(value)
    rate.sleep()
