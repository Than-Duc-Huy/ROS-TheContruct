#! /usr/bin/env python

import rospy
# you import the service message python classes generated from Empty.srv.
from std_srvs.srv import Empty, EmptyResponse


def my_callback(request):
    print("My_callback has been called")
    return EmptyResponse()  # the service Response class, in this case EmptyResponse
    # return MyServiceResponse(len(request.words.split()))


rospy.init_node('service_server')
# create the Service called my_service with the defined callback
my_service = rospy.Service('/my_service', Empty, my_callback)
rospy.spin()  # maintain the service open.
