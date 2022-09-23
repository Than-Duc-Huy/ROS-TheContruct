#!/usr/bin/env python

from robot_control_class import RobotControl


def get_highest_lowest():
    rb = RobotControl()
    laserReading = rb.get_laser_full()
    highest = 0
    lowest = 0
    print(laserReading)
    for i in range(len(laserReading)):
        currentValue = laserReading[i]
        if currentValue == float("inf"):
            continue
        if (currentValue > laserReading[highest]):
            print(currentValue)
            highest = i
        if (currentValue < laserReading[lowest]):
            lowest = i

    print(highest, lowest)
    print(laserReading[highest], laserReading[lowest])
    return [highest, lowest]
