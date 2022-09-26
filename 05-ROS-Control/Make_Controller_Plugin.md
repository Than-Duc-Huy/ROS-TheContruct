# Create a controller
- Create a pacakge with 
    - roscpp
    - pluginlib
    - controller_interface
    - hardware_interface
## Plugin Source file
```cpp
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace controller_ns {

class PositionController : public controller_interface::Controller<
                               hardware_interface::EffortJointInterface> {
public:
  bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
    std::string my_joint;
    if (!n.getParam("joint", my_joint)) {
      ROS_ERROR("Could not find joint name");
      return false;
    }

    joint_ = hw->getHandle(my_joint); // throws on failure
    return true;
  }

  void update(const ros::Time &time, const ros::Duration &period) {
    double error = setpoint_ - joint_.getPosition();
    joint_.setCommand(error * gain_);
  }

  void starting(const ros::Time &time) {}
  void stopping(const ros::Time &time) {}

private:
  hardware_interface::JointHandle joint_;
  static constexpr double gain_ = 2.25;
  static constexpr double setpoint_ = 1.00;
};
PLUGINLIB_EXPORT_CLASS(controller_ns::PositionController,
                       controller_interface::ControllerBase); // special macro plugin PLUGINLIB_EXPORT_CLASS to allow class to be dynamically loaded
} // namespace controller_ns

```
[Non-Integral Class initialization](https://stackoverflow.com/questions/9141950/initializing-const-member-within-class-declaration-in-c)


## Plugin Description file
```xml
<library path="lib/libmy_controller_lib">
  <class name="my_controller/PositionController" 
         type="controller_ns::PositionController"           
         base_class_type="controller_interface::ControllerBase" /> 
    <!--name = package/Class of the plugin type to be used in the config file
    type is where to get the class: inside controller_ns namespace, class Position Controller-->
</library>
```
## Update `package.xml`
- Indicate that there will be a plugin
```xml
<export>
<controller_interface plugin="${prefix}/controller_plugins.xml"/>
</export>
```

## Update `CMakeList.txt`
```cmake
add_compile_options(-std=c++11) #uncomment

add_library(my_controller_lib src/my_controller.cpp)   #Why is the there my_controller_lib? my package name is only my_controller

target_link_libraries(my_controller_lib ${catkin_LIBRARIES})
```

## Check rospack
```bash
rospack plugins --attrib=plugin controller_interface

rospack plugins --attrib=plugin controller_interface | grep my_controller
#Output: my_controller /home/user/catkin_ws/src/my_controller/controller_plugins.xml controller_plugin.xml is the plugin description

```

## Result
- Give you a controller plugin
- Can be used in the `config/.yaml` file
```yaml
robot:
    joint_controller:
        type: my_controller/PositionController 
```
