cmake_minimum_required(VERSION 3.16)
project(vacuum_mujoco)

find_package(ament_cmake REQUIRED)
# The optional simulator apt package provides the extension API and MuJoCo SDK.
find_package(mujoco_ros2_control_plugins 0.1.4 QUIET)
if(mujoco_ros2_control_plugins_FOUND)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(ros_gz_interfaces REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_library(vacuum_mujoco_sensors SHARED src/sensors.cpp)
  target_compile_features(vacuum_mujoco_sensors PRIVATE cxx_std_17)
  ament_target_dependencies(vacuum_mujoco_sensors mujoco_ros2_control_plugins pluginlib rclcpp
    geometry_msgs nav_msgs ros_gz_interfaces sensor_msgs std_msgs tf2_ros)
  target_link_libraries(vacuum_mujoco_sensors mujoco::mujoco)
  install(TARGETS vacuum_mujoco_sensors LIBRARY DESTINATION lib)
  pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugins.xml)
else()
  message(STATUS "Install ros-jazzy-mujoco-ros2-control and rebuild to enable MuJoCo")
endif()

install(DIRECTORY launch mjcf config DESTINATION share/${PROJECT_NAME})
ament_package()
