cmake_minimum_required(VERSION 2.8.3)
project(cob_tray_monitor)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs sensor_msgs
)

install(DIRECTORY config ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS ros/src/tray_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)

