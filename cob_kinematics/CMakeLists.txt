cmake_minimum_required(VERSION 2.8.3)
project(cob_kinematics)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation moveit_core moveit_msgs pluginlib roscpp std_msgs urdf)

add_service_files(
  FILES
  GetPositionIKExtended.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs moveit_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime moveit_core moveit_msgs pluginlib roscpp std_msgs urdf
)

### BUILD ###
include_directories( ${catkin_INCLUDE_DIRS})

#add_executable(urdf_openrave ros/src/urdf_openrave.cpp)
add_subdirectory(ikfast)

### INSTALL ###
