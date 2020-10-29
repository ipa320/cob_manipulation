cmake_minimum_required(VERSION 2.8.3)
project(cob_lookat_action)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  angles
  control_msgs
  geometry_msgs 
  kdl_conversions
  kdl_parser
  message_generation
  move_base_msgs
  roscpp
  sensor_msgs
  tf
  tf2_ros
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
  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs geometry_msgs message_runtime move_base_msgs roscpp sensor_msgs tf trajectory_msgs
  LIBRARIES ${PROJECT_NAME}_server
  INCLUDE_DIRS include
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_server src/${PROJECT_NAME}_server.cpp)
add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_server_node src/${PROJECT_NAME}_server_node.cpp)
add_dependencies(${PROJECT_NAME}_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server_node ${PROJECT_NAME}_server ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME}_server ${PROJECT_NAME}_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}_client.py scripts/cob_lookat_target_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

