#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from custom_srv.srv import Message, MessageResponse
import time

data = Twist()


def callback(request):
    print("Called")
    start = time.time()
    data.linear.x = 1
    data.angular.z = 1
    pub.publish(data)
    while(time.time() - start < request.duration):
        pass
    data.linear.x = 0
    data.angular.z = 0
    pub.publish(data)

    res = MessageResponse()
    res.success = 1
    return res


rospy.init_node("server")
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
service_server = rospy.Service("/my_service", Message, callback)
rospy.spin()
