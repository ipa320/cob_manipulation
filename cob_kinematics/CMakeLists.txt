cmake_minimum_required(VERSION 2.8.3)
project(cob_kinematics)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs geometry_msgs moveit_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  GetPositionIKExtended.srv 
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs moveit_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    DEPENDS roscpp message_runtime std_msgs geometry_msgs moveit_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO
    LIBRARIES # TODO
)



###########
## Build ##
###########

#include_directories(${PROJECT_SOURCE_DIR}/ros/include)

#add_executable(urdf_openrave ros/src/urdf_openrave.cpp)
add_subdirectory(ikfast)



#############
## Install ##
#############
## Mark executables and/or libraries for installation
#install(TARGETS cob_pick_place_action
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


