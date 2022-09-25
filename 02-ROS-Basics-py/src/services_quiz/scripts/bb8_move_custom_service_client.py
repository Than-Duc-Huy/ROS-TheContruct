#! /usr/bin/env python
import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest

rospy.init_node("call")
rospy.wait_for_service("/move_bb8_in_square_custom")
srv = rospy.ServiceProxy("/move_bb8_in_square_custom", BB8CustomServiceMessage)
req = BB8CustomServiceMessageRequest()

req.side = 1
req.repetitions = 2
res = srv(req)

req.side = 2
req.repetitions = 1
res = srv(req)
