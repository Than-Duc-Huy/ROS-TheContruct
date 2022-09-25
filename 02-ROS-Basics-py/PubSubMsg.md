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
