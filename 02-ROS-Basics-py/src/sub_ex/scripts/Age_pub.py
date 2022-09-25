#! /usr/bin/env python
import rospy
from sub_ex.msg import Age

rospy.init_node("Age_pub")
pub = rospy.Publisher("/age", Age, queue_size=1)
data = Age()
data.years = 0
data.months = 0
data.days = 1
rate = rospy.Rate(2)  # 2 Hz
while not rospy.is_shutdown():
    pub.publish(data)
    rate.sleep()
