cmake_minimum_required(VERSION 2.8.3)
project(cob_lookat_action)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib control_msgs geometry_msgs message_generation moveit_msgs roscpp tf trajectory_msgs)

### Message Generation ###
add_action_files(
  FILES
  LookAt.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime
)

### BUILD ###
include_directories(${catkin_INCLUDE_DIRS})

add_executable(cob_lookat_action_server src/cob_lookat_action_server.cpp)
add_dependencies(cob_lookat_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_lookat_action_server ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS cob_lookat_action_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/cob_lookat_action_client.py src/cob_monitor_arm_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
