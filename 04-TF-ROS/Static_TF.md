# Static Transform
- Publisher publish changing TF
- RobotState URDF defined fixed TFs relative to another
- Use Static when for temporary TF that doesn't require changing the whole URDF (temporary mount)

```bash
rosrun tf static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
```

## Launch
```xml
<launch>
<node pkg="tf" type="static_transform_publish" name="name_of_static_publish_node"
args="x y z yaw pitch roll frame_id child_frame_id">
</node>
</launch>
```

## Exercise
```xml
<launch>
<node pkg="tf" type="static_transform_publisher" name="node1"
args=" 0 0 0 0 0 0 box_bot_1/odom box_bot_2/odom  100"/>
<node pkg="tf" type="static_transform_publisher" name="node2"
args=" 0 0 0 0 0 0 box_bot_2/odom box_bot_3/odom  100 "/>

<node pkg="rviz" type="rviz" name="rviz"/>
</launch>
```

- Don't make typo in the name `static_transform_publisher`