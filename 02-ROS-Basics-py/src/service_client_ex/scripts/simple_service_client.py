#! /usr/bin/env python

import rospy
import sys
from trajectory_by_name_srv.srv import TrajByName, TrajByNameRequest # Get the service type

#trajectory_by_name_srv is a package
#trajcetory_by_name_srv.srv is a folder srv inside the package
#TrajByName.srv defines the structure 
    #After compile: TrajByNameRequest, TrajByNameResponse

rospy.init_node('service_client')
# Wait for the service client /trajectory_by_name to be running
rospy.wait_for_service('/trajectory_by_name')
traj_by_name_service = rospy.ServiceProxy('/trajectory_by_name', TrajByName)
#Create a request
traj_by_name_object = TrajByNameRequest()
#Attribute is based on the definition in .srv message type
traj_by_name_object.traj_name = "release_food"
# Pass the Request into the Proxy
result = traj_by_name_service(traj_by_name_object)
print(result)
