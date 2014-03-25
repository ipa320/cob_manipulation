cmake_minimum_required(VERSION 2.8.3)
project(cob_tray_monitor)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ros/launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
)

install(PROGRAMS ros/src/tray_monitor.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)

