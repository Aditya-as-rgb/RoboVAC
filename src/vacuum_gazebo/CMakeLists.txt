cmake_minimum_required(VERSION 3.8)
project(vacuum_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
