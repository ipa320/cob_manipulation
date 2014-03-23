cmake_minimum_required(VERSION 2.8.3)
project(cob_tactiletools)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS nodes/tactile_data_gui.py nodes/tactile_filters.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
