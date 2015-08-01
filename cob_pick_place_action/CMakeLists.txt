cmake_minimum_required(VERSION 2.8.3)
project(cob_pick_place_action)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs geometry_msgs moveit_msgs actionlib actionlib_msgs tf geometric_shapes shape_tools moveit_ros_planning_interface moveit_ros_move_group cob_moveit_interface cob_grasp_generation)
find_package(Boost REQUIRED COMPONENTS thread signals)


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
  CATKIN_DEPENDS roscpp rospy message_runtime std_msgs geometry_msgs moveit_msgs actionlib actionlib_msgs tf geometric_shapes shape_tools moveit_ros_planning_interface moveit_ros_move_group cob_moveit_interface cob_grasp_generation
  INCLUDE_DIRS ros/include common/include
)

### BUILD ###
include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(cob_pick_place_action ros/src/cob_pick_place_action.cpp common/src/GraspTable.cpp)
add_dependencies(cob_pick_place_action ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(cob_pick_place_action ${Boost_LIBRARIES})
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
