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
  DEPENDENCIES actionlib_msgs moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime moveit_msgs
)


### INSTALL ###
install(DIRECTORY files launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
        scripts/generate_grasps_client.py
        scripts/generate_grasps_server.py
        scripts/query_grasps_client.py
        scripts/query_grasps_server.py
        scripts/show_grasps_or_client.py
        scripts/show_grasps_or_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
