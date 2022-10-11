# Gazebo
`roslauch simulation_gazebo main.launch`

## Needed packages
```bash
rospy
rviz
controller_manager
gazebo_ros
joint_state_publisher
robot_state_publisher
```
## URDF File
```xml
<?xml version="1.0"?>
<robot name="mira">
    <!-- Link -->
    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/><!--Parent Pose-->
            <geometry>
                <cylinder radius="0.06" length="0.09"/>
            </geometry>
        </visual>
    </link>

    <link name="roll_M1_link">
        <visual>
            <!--Transform from the child joint-->
            <origin rpy="0 0 0" xyz="0 0 0"/> 
            <geometry>
                <cylinder radius="0.06" length="0.09"/>
            </geometry>
            <material name="color">
                <color rgba="0.5 0.5 0 1"/>
            </material> <!-- Follow the documentation -->
        </visual>
    </link>

    <joint name="roll_joint" type="revolute">
        <parent link="base_link"/>
        <child link="roll_M1_link"/>
        <!--Transform from parent pose to child joint-->
        <origin xyz="0.0023 0 -0.0005" rpy="0 0 0"/>
        <limit lower="-0.2" upper="0.2" effort="0.1" velocity="0.005"/>
        <axis xyz="1 0 0"/>
    </joint>
</robot>

```

### Format
- Type
    - Revolute
    - Continuous
    - Prismatic
    - Fixed
    - Floating
    - Planar
- Parent, Child: Who is connected to who?
- Origin
- Limit
- Axis
    - Child Axis that it will move

## Visualize Launch file
```xml
<launch>
    <!--Argument comes from the user-->
    <!--USE: roslaunch pkg launch.launch model:="$(find robot_pkg)/urdf/myrobot.urdf"-->

    <arg name="model" default=""/>
    <!--Load to Parameter Server-->
    <param name="robot_description" command="cat $(arg model)"/>

    <!--Send fake Joint value-->
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <!--What does this param do? Load or upload?-->
        <param name="use_gui" value="TRUE"/>
    </node>

    <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"/>

    <!--Combine and connect joint using TF-->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

    <!--Trigger Rviz-->
    <node name="rviz" pkg="rviz" type="rviz" args=""/>

</launch>
```

## Visualise the URDF
```bash
urdf_to_graphiz  <.urdf>  # Need to install?
```

- Save configuration into .rviz

```xml
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_mira_description)/rviz_config/mira.rviz"/>
```


# Morphology 

## 3D CAD to URDF
- `.stl` or `.dae` format
- Use Blender to add color
- Export and place in the folder `model` or `meshes`
```xml
<geometry>
    <mesh filename="package://packages/meshes/component.dae"/>
</geometry>
```



# Understanding
- The robot param will upload the param as the whole xml file
- Joint publisher is a scripts that will parse the xml file in `/robot_description`
    - What happened if there are more than 1 robots?
    - Then the `joint_state_publisher` has to be given a namespace'
- Need to have at least 2 link (with base_link) and at least 1 joint for visualization to work