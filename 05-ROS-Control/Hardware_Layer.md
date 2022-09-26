# Hardware interface
- Write a class that read data from hardware and write to the actuator
- Create a node to perform a ROS Control Loop

## Hardware interface
```cpp
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
 {
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_a);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_b);

   registerInterface(&jnt_pos_interface);
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

```

## ROS Control Loop
```cpp
main()
{
    MyRobot robot;
    controller_manager::ControllerManager cm(&robot);
    
    while (true)
    {
        robot.read();
        cm.update(robot.get_time(), robot.get_period());
        robot.write();
        sleep();
    }   
}
```
## Create a package
- Dependencies
    - controller_manager
    - hardware_interface

- When creating hardware interface
    - Put functions and variables definition in `.hpp` file in `include` 
    - Put the implementation in `.cpp`

### `.hpp` file (in package/include/package/.hpp)
```cpp
#ifndef RRBOT_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_
#define RRBOT_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

namespace rrbot_hardware_interface
{
class RRBotHardwareInterface : public hardware_interface::RobotHW
{
public:
  bool init(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh);

  bool read(const ros::Time time, const ros::Duration period);

  bool write(const ros::Time time, const ros::Duration period);

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_command_interface_;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;

  std::vector<std::string> joint_names_;
};

}  // namespace rrbot_hardware_interface

#endif  // RRBOT_HARDWARE_INTERFACE__RRBOT_HARDWARE_INTERFACE_HPP_

```

### `.cpp` implementation file
```cpp
#include "rrbot_hardware_interface/rrbot_hardware_interface.hpp"

#include <limits>
#include <vector>

namespace rrbot_hardware_interface
{

bool RRBotHardwareInterface::init(ros::NodeHandle & /*root_nh*/, ros::NodeHandle & robot_hw_nh)
{
  if (!robot_hw_nh.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'joint_names' on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
    "'joint_names' on the parameter server.");
  }

  size_t num_joints = joint_names_.size();
  ROS_INFO_NAMED("RRBotHardwareInterface", "Found %zu joints.", num_joints);

  hw_position_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_position_commands_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());

  // Create ros_control interfaces
  for (size_t i = 0; i < num_joints; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(
      hardware_interface::JointStateHandle(
        joint_names_[i], &hw_position_states_[i], &hw_velocity_states_[i], &hw_effort_states_[i]));

    // Create joint position control interface
    position_command_interface_.registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &hw_position_commands_[i]));
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_command_interface_);

  // stat execution on hardware
  ROS_INFO_NAMED("RRBotHardwareInterface", "Starting...");

  // in this simple example reset state to initial positions
  for (size_t i = 0; i < num_joints; ++i){
    hw_position_states_[i] = 0.0;  // INITIAL POSITION is ZERO
    hw_position_commands_[i] = hw_position_states_[i];
  }

  return true;
}

bool RRBotHardwareInterface::read(
  const ros::Time time, const ros::Duration period)
{
  // read robot states from hardware, in this example print only
  ROS_INFO_NAMED("RRBotHardwareInterface", "Reading...");

  // write command to hardware, in this example do mirror command to states
  for (size_t i = 0; i < hw_position_states_.size(); ++i) {
    ROS_INFO_NAMED("RRBotHardwareInterface",
                   "Got state %.2f for joint %zu!", hw_position_states_[i], i);
  }

  return true;
}

bool RRBotHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
  // write command to hardware, in this example do mirror command to states
  for (size_t i = 0; i < hw_position_commands_.size(); ++i) {
    hw_position_states_[i] = hw_position_states_[i] +
                             (hw_position_commands_[i] - hw_position_states_[i]) / 100.0;
  }

  return true;
}

}  // namespace rrbot_hardware_interface
```

### `.cpp` node handle file
```cpp
#include "controller_manager/controller_manager.h"
#include "rrbot_hardware_interface/rrbot_hardware_interface.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrbot_hardware_interface");

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::NodeHandle root_nh;
  ros::NodeHandle robot_nh("~");

  rrbot_hardware_interface::RRBotHardwareInterface rrbot_hardware_interface;
  controller_manager::ControllerManager controller_manager(&rrbot_hardware_interface, root_nh);

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  rrbot_hardware_interface.init(root_nh, robot_nh);

  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    // Receive current state from robot
    if (!rrbot_hardware_interface.read(timestamp, period)) {
      ROS_FATAL_NAMED("rrbot_hardware_interface",
                      "Failed to read state from robot. Shutting down!");
      ros::shutdown();
    }

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(
      stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;


    // Update the controllers
    controller_manager.update(timestamp, period);

    // Send new setpoint to robot
    rrbot_hardware_interface.write(timestamp, period);

    loop_rate.sleep();
  }

  spinner.stop();
  ROS_INFO_NAMED("rrbot_hardware_interface", "Shutting down.");

  return 0;
}

```

### Update CMake.txt
```cmake
# Uncomment
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# For the .cpp interface
add_library(${PROJECT_NAME}
  src/rrbot_hardware_interface.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# For the .cpp node
add_executable(${PROJECT_NAME}_node
  src/rrbot_hardware_interface_node.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Additional stuff

## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
```


### Launch file with the hardware interface
```xml
<?xml version="1.0" encoding="utf-8"?>
<launch>

  <arg name="controller_to_spawn" default="" />
  <arg name="controllers_yaml" default="rrbot_controllers" />

  <param name="robot_description" command="$(find xacro)/xacro '$(find rrbot_description)/urdf/rrbot_robot.urdf.xacro'" />

  <node name="rrbot_hardware_interface" pkg="rrbot_hardware_interface" type="rrbot_hardware_interface_node">
    <!-- Load standard controller joint names from YAML file to parameter server -->
    <rosparam command="load" file="$(find rrbot_bringup)/config/joint_names.yaml" />
  </node>

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam command="load" file="$(find rrbot_bringup)/config/$(arg controllers_yaml).yaml" />

  <!-- Load robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
    <param name="ignore_timestamp" type="bool" value="true"/>
  </node>

  <!-- Load controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"
      args="joint_state_controller $(arg controller_to_spawn)"/>

  <node name="view_rrbot" pkg="rviz" type="rviz" args="-d $(find rrbot_description)/rviz/rrbot.rviz" />

</launch>

```