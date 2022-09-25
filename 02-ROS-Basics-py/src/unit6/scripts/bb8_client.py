#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyRequest
from custom_srv.srv import Message, MessageRequest


rospy.init_node("call")
rospy.wait_for_service("/my_service")
srv = rospy.ServiceProxy("/my_service", Message)
req = MessageRequest()
req.duration = 1
res = srv(req)
print(res.success)
