cmake_minimum_required(VERSION 2.8.3)
project(cob_collision_monitor)

add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_move_group
  moveit_ros_planning
  pluginlib
  std_msgs
  tf
  tf2_ros
)
catkin_package(
  CATKIN_DEPENDS std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_capability
  src/collision_monitor_capability.cpp
)

add_executable(${PROJECT_NAME}_node src/collision_monitor_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_capability
  ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_capability ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES move_group_capabilities.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

