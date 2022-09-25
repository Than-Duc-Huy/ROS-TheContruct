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

# Debug, RQT, Rosbag

```bash
rosbag record -O name_of_bag_file.bag Topic1 Topic2 ...
rosbag info name_of_bag_file.bag
rosbag play name_of_bag_file.bag


rostopic echo /topic/variable_as_defined_in_msgs
rqt_plot /topic/variable_as_defined_in_msgs


rosservice call /gazebo/pause_physics "{}"
rosservice call /gazebo/unpause_physics "{}"

rosbag record -a

```
