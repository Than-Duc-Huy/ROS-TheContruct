#! /usr/bin/env python

import rospy
# Import the service message used by the service /gazebo/delete_model
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
import sys

# Initialise a ROS node with the name service_client
rospy.init_node('remove_model_service_client')
print("Waiting for Service /gazebo/delete_model")
# Wait for the service client /gazebo/delete_model to be running
rospy.wait_for_service('/gazebo/delete_model')
delete_model_service = rospy.ServiceProxy(
    '/gazebo/delete_model', DeleteModel)  # Create the connection to the service
kk = DeleteModelRequest()  # Create an object of type DeleteModelRequest
# Fill the variable model_name of this object with the desired value
kk.model_name = "mira"
print("Deleting model ="+str(kk))
# Send through the connection the name of the object to be deleted by the service
status_message = delete_model_service(kk)
print(status_message)
