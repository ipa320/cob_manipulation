cmake_minimum_required(VERSION 3.0.2)
project(cob_moveit_interface)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/load_scene_example.py scripts/move_group_example.py scripts/reactive_planning_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
