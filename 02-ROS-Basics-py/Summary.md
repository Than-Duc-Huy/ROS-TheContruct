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
## Message
```bash
rosmsg show </message_type>
```
### Custom
- We want to add `Custom.msg`
#### CMAke
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
#### Package.xml
```xml
<build_depend>message_generation</build_depend> 
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```

#### Check
- Catkin make/build, source
```bash
rosmsg list | grep Custom #Find the msg
rosmsg info Custom
```
# ROS Services
## Server 
## Cliet

# Python Classes
## Server
## Client

# ROS Action 
## Server
## Client

# Debug

# Appendix
