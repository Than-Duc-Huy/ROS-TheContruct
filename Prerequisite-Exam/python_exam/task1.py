#!/usr/bin/env python
from robot_control_class import RobotControl


def get_highest_lowest():
    rb = RobotControl()
    laserReading = rb.get_laser_full()
    highest = 0
    lowest = 0
    for i in range(len(laserReading)):
        currentValue = laserReading[i]
        if currentValue == float("inf"):
            continue
        else:
            if (currentValue > highest):
                highest = i
            if (currentValue < lowest):
                lowest = i
    return [highest, lowest]
