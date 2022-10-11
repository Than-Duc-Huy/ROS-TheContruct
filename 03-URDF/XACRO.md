# Xacro
- Xacro is a macro language of urdf
## Protocol
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="gurdy">

</robot>
```
## Import Xacro in launch file
```xml
<param name="robot_description"
  command="$(find xacro)/xacro '$(find my_robot_description)/robots/myrobot.urdf.xacro'" />

```

## Convert `xacro` to `urdf`
```bash
xacro source.xacro > target.urdf  
```
## Check integrity
```bash
xacro gurdy.xacro > tmp.urdf && check_urdf tmp.urdf && rm tmp.urdf


check_urdf #Check urdf directly
```
## Generate urdf on the fly when needed
```xml
<param name="robot_description"
  command="$(find xacro)/xacro '$(find my_robot_description)/robots/myrobot.urdf.xacro'" />

```
## Convert urdf with xacro macro
```xml
<!-- Define the macro -->
<xacro:macro name="robot_part_macro" params="">
  <!--Put the urdf section here-->
</xacro:macro>

<!-- Use it-->
<xacro:robot_part_macro/>

```

## Xacro Parameter
```xml
<!--Macro definition-->
<xacro:macro name="insert_name" params="param1 param2 *block_param">
  <xacro:insert_block name="block1"/>
  <link name="$(param1)$(param2)">
  </link>
</xacro:macro>

<!--Macro Instance-->
<xacro:insert_name param1="1" param2="string">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:insert_name/>
```

## Xacro (global) properties
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="gurdy">

    <!-- Properties -->
	
	<xacro:property name="foot_kp" value="0.5"/>
	<xacro:property name="foot_kd" value="0.1"/>
	<xacro:property name="foot_mu1" value="1.0"/>
    <xacro:property name="foot_mu2" value="1.0"/>
    <xacro:property name="foot_radius" value="1.0"/>
        
    ... HEAD XACRO MACRO...
    ... LEG XACRO MACROS...
    ... Gazebo Control XACRO MACROS...
        
</robot>
```

### Xacro macro to calculate 
- Calculate the intertia etc withough calculate from a third party software and copy over
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="gurdy">

    ... All the other MACROS...
    
    <!-- Math Usefull MACROS-->
    <xacro:macro  name="cylinder_inertia" params="mass r l">
      <inertia  ixx="${mass*(3*r*r+l*l)/12}" ixy = "0" ixz = "0"
                iyy="${mass*(3*r*r+l*l)/12}" iyz = "0"
                izz="${mass*(r*r)/2}" />
    </xacro:macro >

    <xacro:macro name="box_inertia" params="mass x y z">
      <inertia  ixx="${mass*(y*y+z*z)/12}" ixy = "0" ixz = "0"
                iyy="${mass*(x*x+z*z)/12}" iyz = "0"
                izz="${mass*(x*x+z*z)/12}"
      />
    </xacro:macro>

    <xacro:macro name="sphere_inertia" params="mass r">
        <inertia  
            ixx="${2*mass*r*r/5}" ixy = "0" ixz = "0"
            iyy="${2*mass*r*r/5}" iyz = "0"
            izz="${2*mass*r*r/5}"
        />
    </xacro:macro>  

    
        
</robot>
```