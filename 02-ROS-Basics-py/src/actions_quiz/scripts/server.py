#! /usr/bin/env python

import rospy
import actionlib
from actions_quiz.msg import CustomActionMsgAction, CustomActionMsgResult
from std_msgs.msg import Empty
import subprocess
import signal
import os


class Action(object):
    _result = CustomActionMsgResult()
    publand = rospy.Publisher("/drone/land", Empty, queue_size=1)
    pubtake = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
    emp = Empty()

    def __init__(self):
        self._as = actionlib.SimpleActionServer(
            "action_custom_msg_as", CustomActionMsgAction, self.callback, False)
        self._as.start()

    def callback(self, goal):
        success = True
        if (goal.goal == "TAKEOFF"):
            rospy.loginfo("takeoff")
            self.pubtake.publish(self.emp)

        if (goal.goal == "LAND"):
            rospy.loginfo("Landed")
            self.publand.publish(self.emp)

        if self._as.is_preempt_requested():
            rospy.loginfo("Goal cancelled")
            self._as.set_preempted()  # SET PREEMPT
            success = False
        if success:
            rospy.loginfo("Success")
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    rospy.init_node("server")
    Action()
    rospy.spin()
