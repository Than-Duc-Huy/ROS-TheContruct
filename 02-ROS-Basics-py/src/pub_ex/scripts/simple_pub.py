#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32  # Type/Class Int32

rospy.init_node("this_pub")  # The launch file name will overwrite this line
# Must follow ROS base name, words_underscore
pub = rospy.Publisher('/counter', Int32, queue_size=1)
rate = rospy.Rate(2)  # 2Hz
count = Int32()
count.data = 0  # get data from Int32 class

while not rospy.is_shutdown():
    pub.publish(count)
    count.data += 1
    rate.sleep()
