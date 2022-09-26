# Quiz workflow
1. Create package
2. Create launch folder and file
3. Go to `ur_description` to inspect
4. copy .xarco, .urdf
  - No .yaml file
  - Make .yaml file
  - `joint_states` MUST
  - Check .xarco for joints name
  - Choose controller (for example: position_controllers/JointPositionController)
  - Make sure the joints name are correct VERBATIM
5. write .launch file 
  - import param
  - rosparam: .yaml
  - spawn controller
  - robot state publisher



## Note
- Build target `<package_name>_lib`
```cmake
add_library(<package_name>_lib src/my_controller.cpp)
target_link_libraries(<package_name>_lib ${catkin_LIBRARIES})
```

- Still very confused about the custom controller