cmake_minimum_required(VERSION 2.8.3)
project(cob_grasp_generation)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation moveit_msgs)

catkin_python_setup()

add_action_files(
  FILES
  GenerateGrasps.action
  QueryGrasps.action
  ShowGrasps.action
)

generate_messages(
  DEPENDENCIES moveit_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs actionlib geometry_msgs message_runtime moveit_msgs roslib rospy tf trajectory_msgs
)


### INSTALL ###
install(DIRECTORY files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/cob_grasp_generation_action_server.py scripts/generate_grasps_client.py scripts/query_grasps_client.py scripts/show_grasps_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
