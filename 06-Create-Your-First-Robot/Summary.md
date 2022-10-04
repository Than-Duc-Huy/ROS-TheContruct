# Steps
1. Get the `.xacro`
    - URDF file, describe the connection
2. Create launch files
    - Launch for Rviz
        ```xml
        <?xml version="1.0"?>
        <launch>
        
        <param name="robot_description" command="cat '$(find rosbots_description)/urdf/rosbots.xacro'"/>
        
        <!-- send fake joint values -->
        <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
            <param name="use_gui" value="False"/>
        </node>
        
        <!-- Combine joint values -->
        <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
        
        <!-- Show in Rviz   -->
        <node name="rviz" pkg="rviz" type="rviz" />
        
        </launch>
        ```
    - Launch gazebo simulation
        <?xml version="1.0" encoding="UTF-8"?>
        <launch>
            <param name="robot_description" command="cat '$(find rosbots_description)/urdf/rosbots.xacro'" />
        
            <arg name="x" default="0"/>
            <arg name="y" default="0"/>
            <arg name="z" default="0.5"/>
        
            <node name="mybot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
                args="-urdf -param robot_description -model rosbots -x $(arg x) -y $(arg y) -z $(arg z)" />
        
        </launch>

    
# Useful
- Delete existing model: 
`rosservice call /gazebo/delete_model "model_name: 'rosbots'"`
- When you delete the model with `<gazebo>` plugin, the thing will crash

# Xacro Macro
- `cat` to show the content of the `.xacro` file
<param name="robot_description" command="cat '$(find rosbots_description)/urdf/rosbots.xacro'" />

- Need to parse the `.xacro` file using `xacro.py`
<param name="robot_description" command="$(find xacro)/xacro.py '$(find rosbots_description)/urdf/rosbots.xacro'" />

- Should make the gazebo plugin to a separate file
    - Then include the `gazebo` plugin file to the main xacro file



# Add sensing capability
- Add link to the `.xacro` file 
- Add `gazebo` plugin to make the camera works in gazebo
    - [Gazebo Plugin](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Camera)

# Connect to real robot
## Node 
```python
#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from motor_driver import MotorDriver

class RobotMover(object):

    def __init__(self, value_BASE_PWM, value_MULTIPLIER_STANDARD, value_MULTIPLIER_PIVOT, value_simple_mode):
        rospy.Subscriber("/morpheus_bot/cmd_vel", Twist, self.cmd_vel_callback)
        self.motor_driver = MotorDriver( i_BASE_PWM=value_BASE_PWM,
                                         i_MULTIPLIER_STANDARD=value_MULTIPLIER_STANDARD,
                                         i_MULTIPLIER_PIVOT=value_MULTIPLIER_PIVOT,
                                         simple_mode=value_simple_mode)
        rospy.loginfo("RobotMover Started...")


    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Decide Speed
        self.motor_driver.set_cmd_vel(linear_speed, angular_speed)


    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('morpheuschair_cmd_vel_listener', anonymous=True)

    if len(sys.argv) > 5:
        value_BASE_PWM = int(float(sys.argv[1]))
        value_MULTIPLIER_STANDARD = float(sys.argv[2])
        value_MULTIPLIER_PIVOT = float(sys.argv[3])
        value_simple_mode = sys.argv[4] == "true"

        robot_mover = RobotMover(value_BASE_PWM,
                                 value_MULTIPLIER_STANDARD,
                                 value_MULTIPLIER_PIVOT,
                                 value_simple_mode)
        robot_mover.listener()
```

## Launch
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>

        <arg name="BASE_PWM" default="50" />
        <arg name="MULTIPLIER_STANDARD" default="0.1" />
        <arg name="MULTIPLIER_PIVOT" default="1.0" />
        <arg name="simple_mode" default="true" />


        <node   name="motor_driver_start"
                pkg="my_robot_control"
                type="move_with_cmd_vel.py"
                respawn="false"
                output="screen"
                args="$(arg BASE_PWM) $(arg MULTIPLIER_STANDARD) $(arg MULTIPLIER_PIVOT) $(arg simple_mode)">
        </node>
</launch>

```



# Line Following
# CV_Bridge

# ORB2_SLAM
# Deep Learning
## Config yaml file 
- Can be loaded to the existing session using `rosparam command="load"` in a launch file
## Principle
- Task Environment: Line follower + Move robot (defined the basic)
- Robot environment: How the robot acts and receive outside information
- Training Parameter
    - publish data topics
    - Hyper-parameter as a separate yaml file, accessed by rosparam

# Issue
- Rviz flickering wheels
    - Due to the lack of tf from the body to the wheel (robot_state_publisher)
- Timer cannot be stopped from another step
- size.width > 0 && size.height > 0
    - 