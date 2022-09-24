#!/usr/bin/env python
from robot_control_class import RobotControl


class ExamControl():
    def __init__(self):
        self.rb = RobotControl()

    def get_laser_readings(self):
        laserReadings = self.rb.get_laser_full()
        print(laserReadings[0], laserReadings[719])
        return [laserReadings[719], laserReadings[0]]

    def main(self):
        while (self.get_laser_readings() != [float("inf"), float("inf")]):
            self.rb.move_straight()
        self.rb.stop_robot()
