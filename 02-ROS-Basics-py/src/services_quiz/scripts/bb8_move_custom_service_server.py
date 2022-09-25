#! /usr/bin/env python
import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist


data = Twist()


def move_straight(time):
    i = 0
    while (i <= time):
        data.linear.x = 1
        pubb()
        i += 1
        rate.sleep()


def turn():
    stop()
    data.angular.z = 3.5
    pubb()
    rate.sleep()
    data.angular.z = 0
    pubb()


def stop():
    data.linear.x = 0
    pubb()


def pubb():
    pub.publish(data)


def callback(req):
    repetition = req.repetitions
    side = req.side

    for i in range(repetition):
        for i in range(4):
            move_straight(side)
            turn()
    # End
    stop()
    res = BB8CustomServiceMessageResponse()
    return res.success


rospy.init_node("server")
rate = rospy.Rate(1)
srv = rospy.Service("/move_bb8_in_square_custom",
                    BB8CustomServiceMessage, callback)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rospy.spin()
