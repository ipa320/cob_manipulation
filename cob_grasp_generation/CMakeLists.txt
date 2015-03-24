cmake_minimum_required(VERSION 2.8.3)
project(cob_grasp_generation)

find_package(catkin REQUIRED COMPONENTS rospy message_generation actionlib actionlib_msgs std_msgs moveit_msgs)

catkin_python_setup()



add_action_files(
  FILES
  GenerateGrasps.action
  QueryGrasps.action
  ShowGrasps.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs moveit_msgs
)



catkin_package(
    CATKIN_DEPENDS rospy message_runtime actionlib actionlib_msgs std_msgs moveit_msgs tf geometry_msgs
)



install(DIRECTORY files
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/cob_grasp_generation_action_server.py scripts/generate_grasps_client.py scripts/query_grasps_client.py scripts/show_grasps_client.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
