# Gazebo

- Set inertias
- Set Gazebo Properties
- Set Collision
- Set Sensors
- Set Control System
## Intertia
```xml
<!-- * * * Link Definitions * * * -->
<link name="base_link">
    <inertial> <!--With and L-->
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.18" />
        <inertia ixx="0.0002835" ixy="0.0" ixz="0.0" iyy="0.0002835" iyz="0.0" izz="0.000324"/>
    </inertial>
    <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
            <cylinder radius="0.06" length="0.09"/>
        </geometry>
    </visual>
</link>  

```

## Gazebo Properties
```xml
<gazebo reference="base_link">
    <kp>100000.0</kp> <!--Static Contact Stiffness-->
    <kd>100000.0</kd> <!--Dynamic Contact Stiffness-->
    <mu1>10.0</mu1> <!--Static Friction Coeff-->
    <mu2>10.0</mu2> <!--Dynamic Friction Coeff-->
    <material>Gazebo/Grey</material>
</gazebo>

```

### Collision
```xml
<link name="base_link">
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.18" />
        <inertia ixx="0.0002835" ixy="0.0" ixz="0.0" iyy="0.0002835" iyz="0.0" izz="0.000324"/>
    </inertial>
    
    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <cylinder radius="0.06" length="0.09"/>
            <!--Can use 3D meshes as well-->
            <mesh filename="file"/> <!--Use primitive or low poly is enough-->
        </geometry>
    </collision>
    
    <visual>
        <origin rpy="0.0 0 0" xyz="0 0 0"/>
        <geometry>
            <cylinder radius="0.06" length="0.09"/>
        </geometry>
    </visual>
</link>
```

## Note
- The first link should not have inertia 
    - The first link is usually `base_link`

## Spawn in gazebo with the URDF (with gazebo tag)
### Spawn
```xml
    <arg name="x" default="0.0" />
    <arg name="y" default="0.0" />
    <arg name="z" default="0.0" />

    <arg name="urdf_robot_file" default="" />
    <arg name="robot_name" default="" />

    <!-- This version was created due to some errors seen in the V1 that crashed Gazebo or went too slow in spawn -->
    <!-- Load the URDF into the ROS Parameter Server -->
    <param name="robot_description" command="cat $(arg urdf_robot_file)" />

    <!-- Run a Python script to the send a service call to gazebo_ros to spawn a URDF robot -->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
    args="-urdf -x $(arg x) -y $(arg y) -z $(arg z)  -model $(arg robot_name) -param robot_description"/>
```

### Spawn with collision
```xml
    <include file="$(find my_mira_description)/launch/spawn_urdf.launch">
        <arg name="x" value="0.0" />
        <arg name="y" value="0.0" />
        <arg name="z" value="0.2" />
        <arg name="urdf_robot_file" value="$(find my_mira_description)/urdf/mira_simple_collisions_inertias.urdf" />
        <arg name="robot_name" value="mira" />
    </include>
```
### Start Gazebo before spawn

`roslaunch simulation_gazebo main.launch`

#### Get world property
`rosservice call /gazebo/get_world_properties "{}"`
- Show properties and spawned models in gazebo

`rosservice call /gazebo/delete_model "model_name: 'mira'"`
- Remove the model
- Delete using a ROS Node

## Add Control
- Add Gazebo `gazebo_ros_control` plugin
- Add Transmission
    - Type
    - Joint
    - Acutator
- Add YAML for PID parameter
### Transmission
```xml
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so"> <!--Ros control for the namespace-->
            <robotNamespace>/mira</robotNamespace> <!--This namespace has a slash in front-->
        </plugin>
    </gazebo>
    
    <joint name="roll_joint" type="revolute">
    	<parent link="base_link"/>
    	<child link="roll_M1_link"/>
        <origin xyz="0.0023 0 -0.0005" rpy="0 0 0"/>
        <limit lower="-0.2" upper="0.2" effort="0.1" velocity="0.005"/>
        <axis xyz="1 0 0"/>
	</joint>

    <transmission name="tran1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="roll_joint">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor1">
            <hardwareInterface>EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>

```

### Yaml Config file
```yaml
mira:
    # Publish all joint states -----------------------------------
    joint_state_controller:
      type: joint_state_controller/JointStateController
      publish_rate: 50

    # Position Controllers ---------------------------------------
    roll_joint_position_controller:
      type: effort_controllers/JointPositionController
      joint: roll_joint
      pid: {p: 1.0, i: 1.0, d: 0.0}
    # To add more just add them here as the first one

```

#### RQT_reconfigure
`rosrun rqt_reconfigure rqt_reconfigure`
### Launch file Controller Spawner
```xml
<launch>
  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find my_mira_description)/config/mira_onecontroller.yaml" command="load"/>

  <!-- load the controllers -->

  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" ns="/mira" args="roll_joint_position_controller joint_state_controller --shutdown-timeout 3"/> <!--When to remap?-->

  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    respawn="false" output="screen">
    <remap from="/joint_states" to="/mira/joint_states" /> <!--When to remap?-->
  </node>

</launch>
```
### Control with python


###