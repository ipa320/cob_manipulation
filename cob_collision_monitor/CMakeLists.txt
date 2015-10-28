cmake_minimum_required(VERSION 2.8.3)
project(cob_collision_monitor)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_move_group
  moveit_ros_planning
  pluginlib
  std_msgs
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cob_collision_monitor
#  CATKIN_DEPENDS moveit_ros_move_group moveit_ros_planning pluginlib std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(cob_collision_monitor_capability
  src/collision_monitor_capability.cpp
)

add_executable(cob_collision_monitor_node src/collision_monitor_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cob_collision_monitor_node
  ${catkin_LIBRARIES}
)
target_link_libraries(cob_collision_monitor_capability
  ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
install(TARGETS cob_collision_monitor_capability cob_collision_monitor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES move_group_capabilities.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

