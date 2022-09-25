# Basics
- Topics: constant stream of data
- Services: interrupt, start service, end service, continue
- Actions: concurrent

# Packages
```xml
    <node pkg="turtlebot_teleop" type="turtlebot_teleop_key.py" name="turtlebot_teleop_keyboard" output="screen">
    <!--
    pkg="package_name" # Name of the package that contains the code of the ROS program to execute
    type="python_file_name.py" # Name of the program file that we want to execute
    name="node_name" # Name of the ROS node that will launch our Python file
    output="type_of_output" # Through which channel you will print the output of the Python file
    -->
```

- What are the other valid parameter for "output"
### rospack
- Inspect installed packages
```bash
rospack list #list all the packages and location
rospack list | grep package_name # search for package_name
rospack profile # refresh the package list
```

### rosnode
- Inspect the running nodes
```bash
rosnode list #list all nodes
rosnode info <node_name> #more info about the running node
```

### build
```bash
catkin_make # if the workspace is not built from catkin_build
source devel/setup.bash

catkin_make --only-pkg-with-deps <package_name>
```

## Parameter Server
```bash
rosparam
rosparam get <param_name>
rosparam set <param_name> <value>
```

## Environment variables
```bash
export | grep ROS
ROS_MASTER_URI # Contains the url where the ROS Core is being executed. Usually, your own computer (localhost).
ROS_PACKAGE_PATH # Contains the paths in your Hard Drive where ROS has packages in it.

```

# ROS Topic
## Publisher
```python
rospy.init_node("use_a_ROS_base_name")
pub = rospy.Publisher('/topic_name', MessageClass/Type, queue_size = 2)

```

```bash
rostopic echo /topic_name -n1 #print only 1 message
rostopic hz /topic_name # hertz, check frequency
```

- *How do we define the structure of the topic (how are the messages type get formatted within one topic)*
- REMEBER TO MAkE PYTHON FILE EXECUTABLE
- UNDERSTAND the definition of the message
## Subscriber
```python

def callback(msg):
  var = msg.variable # Based on MessageType definition
  #do thing with the message

rospy.init_node("sub")
sub = rospy.Subscriber('/topic_name', MessageType, callback)
rospy.spin()

```

## Message
```bash
rosmsg show </message_type>
```
- We want to add `Custom.msg`
### CMAke
```cmake

find_package(catkin REQUIRED COMPONENTS
       rospy
       std_msgs
       message_generation   # Add message_generation here, after the other packages
)


add_message_files(
      FILES
      Custom.msg
    ) # Dont Forget to UNCOMENT the parenthesis and add_message_files TOO



generate_messages(
      DEPENDENCIES
      std_msgs
) # Dont Forget to uncoment here TOO


# All of the packages in here will have to be in package.xml as exec_depend
catkin_package(
      CATKIN_DEPENDS rospy message_runtime   # This will NOT be the only thing here
)
```
### Package.xml

```xml
<build_depend>message_generation</build_depend> 
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```

### Check
- Catkin make/build, source
```bash
rosmsg list | grep Custom #Find the msg
rosmsg info Custom
```

##### Don't name the python file as the same name of the pacakge
- Because the way python work, it will not find the .py file to import


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


# ROS Action 
- Asynchronous calls to services. Create 5 topics
  - cancel
  - feedback
  - goal
  - result
  - status
- Information exchange topics: goal, result, feedback

- Status 
  - Pending: 0
  - Active: 1
  - Done: 2
  - Warn: 3
  - Error: 4

## Client
- After build/make: Message.action will be built into
  - MessageAction
  - MessageGoal
  - MessageResult
  - MessageFeedback
  - in package_name.msg (the topic messages)
- The action message is the file `.action` but after compiling, it will be in package.msg folder when import to python
- Action Message has 3 parts
  - goal
  - result
  - feedback

### Python Example Code
```python
import actionlib
import action_package.msg import ActionAction, ActionGoal, ActionResult, ActionFeedback

def callback(feedback):

#Initialize
rospy.init_node("Client Action node")
action_server_name = "/action_server"
client = actionlib.SimpleActionClient(action_server_name, ActionAction)
client.wait_for_server()

#Set and send goal
goal = ActionGoal()
goal.variable = value

client.send_goal(goal, callback) # Callback function

#Get state & Do something else
state = client.get_state()
while state < 2: # Is not done
    #Do something else

#Result is after the action
done = client.wait_for_result() # Return boolean if the result is sent

```
### GUI
```bash
rosrun actionlib_tools axclient.py /action_server
```
## Server
### Python Example
```python
import actionlib
from Action.msg import ActionFeedback, ActionResult, ActionAction

class ActionServer():
  _feedback = ActionFeedback()
  _result = ActionResult()

  def __init__(self):
    self._as_ = actionlib.SimpleActionServer("action_server", ActionAction, self.goalCallback, auto_start = False)
    self._as_.start() #Start on class call

  def goalCallback(self,goal): # SimpleActionServer will pass in the goal
    success = True

    # Do things with goal.variable1, goal.variable2 as defined in the message
    self._as.publish_feedback(self._feedback) ## PUBLISH FEEDBACK
    ## Check preemp cancelation
    if self._as.is_preempt_requested():
        rospy.loginfo("Goal cancelled")
        self._as.set_preempted() # SET PREEMPT
        success = False
        break
    ## Check success
    if success:
        self._result.variable1 = self._feedback.variable1 #as defined
        rospy.loginfo("Success")
        self._as.set_succeeded(self._result) # SET SUCCEEDED
if __name__ == '__main__':
  rospy.init_node("ActionServer")
  ActionServer()
  rospy.spin() # Runs forever
```

### Note
- From `Action.action`, after compilation
  - Python: ActionAction, ActionFeedback, ActionResult, ActionGoal
  - Message type: ActionActionFeedback, ActionActionResult, ActionActionGoal

## Message
- Action file
```
#goal
package/type variable
---
#result
package/type variable
---
#feedback
package/type variable

```

### CMake
- `actionlib_msgs` is a must
```cmake
find_package(catkin REQUIRED COMPONENTS
      # your packages are listed here
      std_msgs
      actionlib_msgs
)
add_action_files(
      FILES
      Name.action
)
generate_messages(
      DEPENDENCIES
      std_msgs
      actionlib_msgs 
      # Your packages go here
)
catkin_package(
      CATKIN_DEPENDS
      rospy
      # Your package dependencies go here
)
```
### package.xml
- `actionlib_msgs` is a must
```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend> <!--Needed to compile the custom action msgs based on std_msgs-->
  <build_export_depend>actionlib</build_export_depend>
  <build_export_depend>actionlib_msgs</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <exec_depend>actionlib</exec_depend> <!--Needed in runtime to run action-->
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>rospy</exec_depend>  <!--Needed to run ROS python node-->
```
# Debug

# Appendix
