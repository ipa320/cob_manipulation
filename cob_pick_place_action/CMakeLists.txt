cmake_minimum_required(VERSION 2.8.3)
project(cob_pick_place_action)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib cob_grasp_generation cob_moveit_interface geometric_shapes geometry_msgs message_generation moveit_msgs moveit_ros_move_group moveit_ros_planning_interface roscpp shape_tools std_msgs tf)

### Message Generation ###
add_action_files(
  FILES
  CobPick.action
  CobPlace.action
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs actionlib cob_grasp_generation cob_moveit_interface geometric_shapes geometry_msgs message_runtime moveit_msgs moveit_ros_move_group moveit_ros_planning_interface roscpp shape_tools std_msgs tf
  INCLUDE_DIRS ros/include common/include
)

### BUILD ###
include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include ${catkin_INCLUDE_DIRS})

add_executable(cob_pick_place_action ros/src/cob_pick_place_action.cpp common/src/GraspTable.cpp)
add_dependencies(cob_pick_place_action ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_pick_place_action ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS cob_pick_place_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(PROGRAMS scripts/cob_multi_pick_place_client.py scripts/cob_pick_action_client.py scripts/cob_place_action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY files launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
