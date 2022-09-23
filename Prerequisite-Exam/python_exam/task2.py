#!/usr/bin/env python
import rospy
from robot_control_class import RobotControl


rb = RobotControl()
try:
    while (rb.get_front_laser() > 1.5):
        rb.move_straight()
        print(rb.get_front_laser())
    rb.stop_robot()
    rb.turn("clockwise", 1.60, 1)

except rospy.ROSInterruptException:
    pass
