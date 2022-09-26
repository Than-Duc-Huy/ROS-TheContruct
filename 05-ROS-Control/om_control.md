# Confifure (om_control)
- Define transmission. The transmission element are used to describe the relationship between a joint and an actuator
  - Define transmission in `.xacro`
- Add gazebo plugin to URDF

## `.launch` file
```xml
<launch>

  <!-- Load configuration file -->
  <rosparam file="$(find om_control)/config/om_control.yaml" command="load"/>

  <!-- Start joint state controller -->
  <node name="joint_state_controller_spawner" pkg="controller_manager" type="controller_manager" output="screen"
    args="spawn joint_state_controller" respawn="false"/>
   
  
  <!-- Start arm controller -->
  <node name="arm_controller_spawner" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="spawn arm_controller"/>

  <!-- start robot state publisher -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">    <param name="publish_frequency" type="double" value="100.0" />
    <param name="tf_prefix" type="string" value="" />
  </node>

  <!-- start fake calibration -->
  <node pkg="rostopic" type="rostopic" name="fake_joint_calibration"
        args="pub /calibrated std_msgs/Bool true" />

</launch>

```
- What is the difference of the `spawner` node and the `controller_manager` node?
- Node parameter
```xml
<node>
<param name="name" type="variable_type" value="variable_value"/>
```

## RQT
```bash
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller

```
