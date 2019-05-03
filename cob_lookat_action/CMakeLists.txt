cmake_minimum_required(VERSION 2.8.3)
project(cob_lookat_action)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  angles
  control_msgs
  geometry_msgs 
  kdl_conversions
  kdl_parser
  message_generation
  roscpp
  tf
  tf_conversions
  trajectory_msgs
)

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
  LIBRARIES cob_lookat_action_server
  INCLUDE_DIRS include
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cob_lookat_action_server src/cob_lookat_action_server.cpp)
add_dependencies(cob_lookat_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_lookat_action_server ${catkin_LIBRARIES})

add_executable(cob_lookat_action_server_node src/cob_lookat_action_server_node.cpp)
add_dependencies(cob_lookat_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_lookat_action_server_node cob_lookat_action_server ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS cob_lookat_action_server cob_lookat_action_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/cob_lookat_action_client.py scripts/cob_lookat_target_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

