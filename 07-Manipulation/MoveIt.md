# Manipulation
- MoveIt
- Motion Planning
- Perception
- Grasping

# MoveIt Package
- Semantic Robot Description Format

## Setup
```bash
roslaunch moveit_setup_assistant setup_assistant
```

- planning scene monitor
- world geometry monitor (Octomap)
    - PointCloud occupancy map update plugin

- Inverse Kinematic Solver problem
- RobotState class
- CollisionWorld object (Flexible Collision Library)

## Config files to ROS param server
- ros_controllers.yaml
```yaml
controller_list:
  - name: robot/arm_controller
    action_ns: follow_joint_trajectory
    default: True
    type: FollowJointTrajectory
    joints:
      - rbkairos_ur10_shoulder_pan_joint
      - rbkairos_ur10_shoulder_lift_joint
      - rbkairos_ur10_elbow_joint
      - rbkairos_ur10_wrist_1_joint
      - rbkairos_ur10_wrist_2_joint
      - rbkairos_ur10_wrist_3_joint
```
- Action `/<robot_name>/<action_ns>/<action_topics>`
    - If there is a topic that is `robot/arm_controller/follow_joint_trajectory/goal`
    - Then the Action Server is `robot/arm_controller/follow_joint_trajectory`

- MoveIt during setup 
    - Does not allow introduction of namespace in the to namefield (text box) of Controller Name

**Note**: Why is the ROS architecture so confusing!
- What is the difference between param and arg

## launch file
```xml
<launch>

  <include file="$(find rbkairos_moveit_config)/launch/planning_context.launch" > <!--Load the description-->
    <arg name="load_robot_description" value="true" />
  </include>

  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"> <!--Publish joint state-->
    <param name="/use_gui" value="false"/>
    <rosparam param="/source_list">[/rbkairos/joint_states]</rosparam>
  </node>

  <include file="$(find rbkairos_moveit_config)/launch/move_group.launch">  <!--Launch the move group-->
    <arg name="publish_monitored_planning_scene" value="true" />
  </include>

  <include file="$(find rbkairos_moveit_config)/launch/moveit_rviz.launch"> <!--Launch the RVIZ-->
    <arg name="rviz_config" value="$(find rbkairos_moveit_config)/launch/moveit.rviz"/>
  </include>

</launch>

```

**Note**
- Joint state will publish the joints. Publish it in the `/source_list` parameter for Moveit
    - How does the joint state publisher know that it has to publish in `/rbkairos/joint_states` and not `joint_states`



# Workflow
- MoveIt -> tells Controllers how to move -> Gazebo subscribed to the action result, feedback messages to simulate -> Joint publisher publishes to /joint_state -> Picked up by MoveIt via /source_list

# Issues
- TF_REPEATED_DATA redundant timestamp
    - https://github.com/ros/geometry2/issues/467
    - https://github.com/ros-planning/navigation/issues/1125#issuecomment-1238647110 

- How get gazebo, real robot, moveit, rviz data to follow the same thing
- Remember to select the Kinematic plugin for your planning group
This kinematic plugin is important
- No Motion plan found in Programmatic motion planning
    - The constraints are too strict to find a solution

- How to pass value from launch file to node

# Spawn URDF Object
`rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/table.urdf -urdf -x 1.5 -model my_object`

# Summary 
- PointCloudUpdater
- OctoMap (Occupany Map Updator): Octree representation
- Default planner of MoveIt is OMPL
- move_group node will generate a suitable trajectory -> sent to robot joint trajectory controller

- The wizard create a pacakge with various helper `.launch` files. We still need to write a launch files to trigger the functionalities that we want

- The naming convention of MoveIt Setup wizard is the same, only need to change the robot name

# Motion planning programmatically

## Plan trajectory based on end effector pose
  
```python
#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)   # Initialize the commander module
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()  # Interface to the robot
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm") #Get Group
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1) # move_group namespace 

pose_target = geometry_msgs.msg.Pose() # Final pose of the robot
pose_target.orientation.w = 1.0
pose_target.position.x = 0.96
pose_target.position.y = 0
pose_target.position.z = 1.18
group.set_pose_target(pose_target)

plan1 = group.plan()

rospy.sleep(5)

moveit_commander.roscpp_shutdown()
```

### Get information
```python
print("Reference frame: %s" % group.get_planning_frame())
print("End effector: %s" % group.get_end_effector_link())
print("Robot Groups:")
print(robot.get_group_names())
print("Current Joint Values:")
print(group.get_current_joint_values())
print("Current Pose:")
print(group.get_current_pose())
print("Robot State:")
print(robot.get_current_state())
```

### Go!
```python
moveit_commander.MoveGroupCommander("group").go()

```

# Gazebo 
## Spawm
`rosrun gazebo_ros spawn_model`

# Moveit_python
https://github.com/mikeferguson/moveit_python

and moveit_commander are 2 python bindings for motion planning


# Grasping
## Adding End-effector
- Add `.yaml` file to configure the tool to planning frame


### Basic grasping perception
- Shape grasp planner
- Object support segmenatation
- Object detection will trigger the rest of the motion control

- Trigger segmentation of foreground and back ground
### Pick and place
- Start the motion planning and control


- Add detected object to the MoveIt scene? Is there a separate MoveIt workspace?  

### Adapt to other robot
- `moveit_simple_controller_manager_interface`
