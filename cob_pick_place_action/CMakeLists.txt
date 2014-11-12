cmake_minimum_required(VERSION 2.8.3)
project(cob_pick_place_action)

find_package(catkin REQUIRED COMPONENTS rospy roscpp message_generation actionlib actionlib_msgs std_msgs geometry_msgs moveit_msgs tf moveit_ros_planning_interface  moveit_ros_move_group cob_moveit_interface cob_grasp_generation)
find_package(Boost REQUIRED COMPONENTS thread signals)


#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
  FILES
  CobPick.action
  CobPlace.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs geometry_msgs moveit_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS rospy message_runtime actionlib actionlib_msgs std_msgs geometry_msgs moveit_msgs tf moveit_ros_planning_interface  moveit_ros_move_group cob_moveit_interface cob_grasp_generation
    DEPENDS # TODO
    INCLUDE_DIRS ros/include common/include
    LIBRARIES # TODO
)



###########
## Build ##
###########

include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(cob_pick_place_action ros/src/cob_pick_place_action.cpp common/src/GraspTable.cpp)
add_dependencies(cob_pick_place_action ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(cob_pick_place_action ${Boost_LIBRARIES})
target_link_libraries(cob_pick_place_action ${catkin_LIBRARIES})



#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_pick_place_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY common/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark cpp header files for installation
install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(PROGRAMS scripts/cob_multi_pick_place_client.py scripts/cob_pick_action_client.py scripts/cob_place_action_client.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY files
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
