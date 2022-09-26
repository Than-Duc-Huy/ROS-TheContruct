
# Robot State Publisher

## Launch file
```xml
<launch>

  <param name="robot_description" command="cat $(find pi_robot_pkg)/urdf/pi_robot_v2.urdf" />  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find pi_robot_pkg)/config/pirobot_control.yaml" command="load"/>

  <!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" ns="/pi_robot" args="head_pan_joint_position_controller head_tilt_joint_position_controller torso_joint_position_controller
    left_shoulder_forward_joint_position_controller right_shoulder_forward_joint_position_controller left_shoulder_up_joint_position_controller
    right_shoulder_up_joint_position_controller left_elbow_joint_position_controller right_elbow_joint_position_controller left_wrist_joint_position_controller
    right_wrist_joint_position_controller joint_state_controller"/>

</launch>
```

## State Publisher
- The RobotModel in Rviz was not working because the state publisher is not running, the links don't know their position relative to one another

```xml
<launch>

  <param name="robot_description" command="cat $(find pi_robot_pkg)/urdf/pi_robot_v2.urdf" />  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find pi_robot_pkg)/config/pirobot_control.yaml" command="load"/>

  <!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" ns="/pi_robot" args="head_pan_joint_position_controller head_tilt_joint_position_controller torso_joint_position_controller
    left_shoulder_forward_joint_position_controller right_shoulder_forward_joint_position_controller left_shoulder_up_joint_position_controller
    right_shoulder_up_joint_position_controller left_elbow_joint_position_controller right_elbow_joint_position_controller left_wrist_joint_position_controller
    right_wrist_joint_position_controller joint_state_controller"/>
    
  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    respawn="false" output="screen">
    <remap from="/joint_states" to="/pi_robot/joint_states" />
  </node>

</launch>
```
## Joint State Publisher
### Only connect the joints in the internally
```xml
<launch>
    <param name="robot_description" command="cat $(find pi_robot_pkg)/urdf/pi_robot_v2.urdf" />

    <!-- send fake joint values -->
    <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"/>

    <!-- Combine joint values -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

    <!-- Show in RVIZ, Robot has custom rviz file-->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find pi_robot_pkg)/launch/pi_robot.rviz"/>

</launch>

```

### Change internally and change the simulation
- In Gazebo simulation, you have `controller_manager` ROS package to simulate PID for Position or Effort control

#### Launch
```xml
<launch>

  <param name="robot_description" command="cat $(find pi_robot_pkg)/urdf/pi_robot_v2.urdf" />  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find pi_robot_pkg)/config/pirobot_control.yaml" command="load"/>

  <!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" ns="/pi_robot" args="torso_joint_position_controller joint_state_controller"/><!--ns = namespace = /pi_robot take all the joints of pi_robot-->
    
  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    respawn="false" output="screen">
    <remap from="/joint_states" to="/pi_robot/joint_states" />
  </node>

  <!-- Show in RVIZ   -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find pi_robot_pkg)/launch/pi_robot.rviz"/>

</launch>

```
#### URDF
```xml
<!-- Define the Joint-->
<joint name="left_shoulder_forward_joint" type="revolute">
    <parent link="left_shoulder_link"/>
    <child link="left_shoulder_forward_link"/>
    <origin xyz="0 0.025 0" rpy="0 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="3"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.7"/>
</joint>

<!--Defined the possible relative movement-->
<transmission name="tran3">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="torso_joint">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor3">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>

<!-- Gazebo plugin to control those joints in Gazebo simulation-->
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/pi_robot</robotNamespace>
        <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
</gazebo>

```
#### config/.yaml file (import as rosparam in the launch file)
- Config file will be loaded using `rosparam` and will update the `param` accordingly
```yaml
pi_robot:  #Name space /pi_robot
  # Publish all joint states -----------------------------------
  joint_state_controller:    # Change publish rate of type joint_state_controller/JointStateController
    type: joint_state_controller/JointStateController
    publish_rate: 50
   
  torso_joint_position_controller: # Change the pid values of type effort_controllers/JointPositionController on /torso_joint frame
    type: effort_controllers/JointPositionController
    joint: torso_joint
    pid: {p: 100.0, i: 0.01, d: 10.0}
```


#### GUI
```bash
rosrun rqt_gui rqt_gui
```

- Control topic `/pi_robot/torso_joint_position_controller/command` (where does this topics initiated? is it a tf node?)
    - How are the topics organize? What file, what definition gives rise to them?

## Exercise
- Be careful of typos : `<remap from="/joint_states" to="/pi_robot/joint_states"/>`

- Controlller manager will give 4 topics per controller
    - command
    - pid/parameter_descriptions
    - pid/parameter_updates
    - state

- When there is a controller (`controller_manager`), you don't need the `joint_state_publisher`
    - Because `controller_manager`(gazebo) is joint_state publishing
    - Real robot with encoders can use some other type of node to do joint_state publishing