#! /usr/bin/env python

import rospy
from iri_wam_reproduce_trajectory.srv import ExecTraj, ExecTrajRequest

import rospkg
rospack = rospkg.RosPack()


rospy.init_node("serv_call")
rospy.wait_for_service("/execute_trajectory")
service = rospy.ServiceProxy("/execute_trajectory", ExecTraj)
serviceRequest = ExecTrajRequest()
serviceRequest.file = rospack.get_path(
    'iri_wam_reproduce_trajectory') + "/config/get_food.txt"
response = service(serviceRequest)
