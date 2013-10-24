cmake_minimum_required(VERSION 2.8.3)
project(cob_grasp_generation)


find_package(catkin REQUIRED COMPONENTS rospy message_generation actionlib actionlib_msgs std_msgs moveit_msgs)

catkin_python_setup()


#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
  FILES
  GenerateGrasps.action
  QueryGrasps.action
  ShowGrasps.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs moveit_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS rospy message_runtime actionlib actionlib_msgs std_msgs moveit_msgs tf openrave geometry_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS 
    LIBRARIES # TODO
)
