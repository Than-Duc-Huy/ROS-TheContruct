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
