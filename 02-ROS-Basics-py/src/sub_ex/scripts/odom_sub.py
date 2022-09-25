#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry


def callback(msg):
    print(msg.header)
    print(msg.child_frame_id)
    print(msg.pose)
    print(msg.twist)


rospy.init_node("Odom_sub")
sub = rospy.Subscriber("/odom", Odometry, callback)
rospy.spin()
