cmake_minimum_required(VERSION 3.0.2)
project(cob_manipulation_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation moveit_msgs)
add_action_files(
  FILES
  GenerateGrasps.action
  QueryGrasps.action
  ShowGrasps.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime moveit_msgs
)

