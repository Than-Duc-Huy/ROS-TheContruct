#! /usr/bin/env python
import rospy

rospy.init_node("this-node")  # The launch file name will overwrite this line
print("Started Test")
rate = rospy.Rate(2)  # 2 Hz
while not rospy.is_shutdown():
    print("running")
    rate.sleep()  # Sleep for 0.5 s
