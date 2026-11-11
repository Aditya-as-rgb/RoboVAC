cmake_minimum_required(VERSION 3.8)
project(vacuum_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models maps scripts
  DESTINATION share/${PROJECT_NAME}
)

# `ros2 run vacuum_gazebo dirt_world.py ...` (preview a dirt layout without starting Gazebo)
# and the dirt cleaner node that takes dirt off the floor as the robot drives over it
install(PROGRAMS scripts/dirt_world.py scripts/dirt_cleaner.py
  scripts/dirt_labels.py scripts/dirt_dataset.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
