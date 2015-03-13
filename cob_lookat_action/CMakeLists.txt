cmake_minimum_required(VERSION 2.8.3)
project(cob_lookat_action)

find_package(catkin REQUIRED COMPONENTS rospy roscpp message_generation actionlib actionlib_msgs std_msgs geometry_msgs moveit_msgs control_msgs tf)
find_package(Boost REQUIRED COMPONENTS thread)

#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
  FILES
  LookAt.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs geometry_msgs moveit_msgs control_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS rospy message_runtime actionlib actionlib_msgs std_msgs geometry_msgs moveit_msgs control_msgs tf
)



###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(cob_lookat_action_server src/cob_lookat_action_server.cpp)
add_dependencies(cob_lookat_action_server ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(cob_lookat_action_server ${Boost_LIBRARIES})
target_link_libraries(cob_lookat_action_server ${catkin_LIBRARIES})



#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS cob_lookat_action_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/cob_lookat_action_client.py src/cob_monitor_arm_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
