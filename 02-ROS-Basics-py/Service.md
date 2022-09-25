# ROS Services
- Service is synchronous
- Action is asynchronous
## Client
```bash
rosservice call <service> #TAB TAB to autofill the parameter structure
rosservice list | grep <service> # Check full service name
rosservice info <service> # get the info of the service

    #Node: It states the node that provides (has created) that service.
    #Type: It refers to the kind of message used by this service. It has the same structure as topics do. It's always made of package_where_the_service_message_is_defined / Name_of_the_File_where_Service_message_is_defined.
    #Args: Here you can find the arguments that this service takes when called.

rossrv show <Type_of_service> #The message 
```

### Python Example
```python
from package.srv import ServiceMessage, ServiceMessageRequest # Get the service type:

#package is a package
#package.srv is a folder srv inside the package
#package.srv is the folder after compilation
    #After compile: ServiceMessageRequest, ServiceMessageResponse, ServiceMessage (itself)

# Init
rospy.init_node('service_client')
rospy.wait_for_service('/service_name')
srv = rospy.ServiceProxy('/service_name', ServiceMessage) # Link the name with Message

req = ServiceMessageRequest() #Create a request
req.variable = value #Attribute is based on the definition in .srv message type

# Pass the Request into the Proxy
result = srv(req)

### Do more
```
### Include a launch in .launch
```xml
<include file="$(find package_with_the_launch_file)/launch/launch.launch>
```
### rospkg.RosPack.get_path()
## Server 
### Python Example
```python
import rospy
from Package.srv import ServiceMessage, ServiceMessageResponse

def callback(req): #req message type will be passed in 
  # Do things with req.variable
  res = ServiceMessageResponse
  res.variable = value
  return res

rospy.init_node("service_server")
srv = rospy.Service("/service_name", ServiceMessage, callback)
rospy.spin() #Run forever
```
## Message
```
package/type variable1 
---
package/type variable2
```

### CMake
```cmake
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation #generate message, srv, action
)
add_service_files(
  FILES
  Message.srv #Custom file
)
generate_messages(
  DEPENDENCIES
  std_msgs #From primitive
)
catkin_package(
      CATKIN_DEPENDS
      rospy #rospy is needed to generate message, if you don't have to generate message, rospy is not needed
)
```
### package.xml
```xml
<build_depend>message_generation</build_depend>
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```