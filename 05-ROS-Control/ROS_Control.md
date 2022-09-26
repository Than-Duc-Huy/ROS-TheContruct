# Essentials
- Set of packages
    - controller interfaces
    - controller managers
    - transmissions
    - hardware_interfaces
    - control_toolbox
- controller plugins
    - effort_controllers
    - position_controllers
    - velocity_controllers
    - joint_state_controller
- `effort_controller` communicate with a joint that uses effort interface
- `joint_state_controller` ALWAYS have to be set

## Hardware interface
- Software abstraction of the hardware
- Intermediate between the controller plugin and the real or simulated robot
- Interfaces
    - Joint Command Interfaces
        - Effort
        - Velocity
        - Position
    - Joint State Interfaces
- Interface is defined in the `.urdf` file 

# Configure Controller
## Configuring the URDF (Transmission)
### `.xacro` file
```xml
  <transmission name="tran1">
    <type>transmission_interface/SimpleTransmission</type> 
    <joint name="joint1"> <!--Joint connected to the transmission. Must match the name in the URDF-->
      <hardwareInterface>EffortJointInterface</hardwareInterface> <!--Define the hardware interface-->
    </joint>
    <actuator name="motor1"> <!--The hardware actuator-->
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
```

### `.gazebo` file
```xml
<robot>

  <!-- ros_control plugin -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/rrbot</robotNamespace> <!--Name of the robot-->
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType> <!--for advanced user. Setting default allows us to use 
      JointStateInterface, EffortJointInterface, PositionJointInterface, VelocityJointInterface-->

      <!--Don't need to define leave as default>
      <robotParam>/robot_description</robotParam> Load the parameter on the parameter server. Don't have to defined and left as default
      <controlPeriod></controlPeriod> Period of the controller update
      -->
    </plugin>
  </gazebo>

</robot>

- Add the gazebo plugin read the transmission tag in the xacro and load them to gazebo accordingly

```
## Configure and launch controller
### `/config/.yaml` file
- Has the configuration of the joint_controller parameters
```yaml
rrbot: # Robot name / name space
# Joint state controller (ALWAYS NEEDED)
# Publish into /joint_states
  joint_state_controller:  
    type: joint_state_controller/JointStateController 
    publish_rate: 50  
# Other controller
  joint1_position_controller: 
    type: effort_controllers/JointPositionController # Type of plugin
    joint: joint1
    pid: {p: 100.0, i: 0.01, d: 10.0}
  joint2_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint2
    pid: {p: 100.0, i: 0.01, d: 10.0}

```
### `/launch/.launch` file
```xml
<launch>
<!--Why didn't we input the URDF file?-->
  <rosparam file="$(find my_robot_control)/config/my_robot_control.yaml" command="load"/> <!--Load the .yaml into the parameter server-->

<!--Controller manager, spawn controller -->
<!--Service call node, take arguments as the controllers need to be started -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" 
    output="screen" ns="/rrbot" args="joint1_position_controller joint2_position_controller joint_state_controller"/>

<!--robot state publisher-->
<!--Sub and Pub node. Subscribe and listen to the /joint_state and publish to /tf topics-->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    respawn="false" output="screen">
    <remap from="/joint_states" to="/rrbot/joint_states" />
  </node>

</launch>
```
### Question
- Why don't we need to include the urdf file?
- Does `.gazebo` file do anything? Or is it already imported as part of the environment? And it just subscribe to the topics
### RQT_GUI
- Dynamic Configure
- Plot
- Publisher
