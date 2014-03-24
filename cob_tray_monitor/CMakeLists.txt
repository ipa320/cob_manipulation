cmake_minimum_required(VERSION 2.8.3)
project(cob_tray_monitor)

find_package(catkin REQUIRED)

catkin_package()

install(FILES config/monitor_params.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES ros/launch/tray_monitor.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(PROGRAMS ros/src/tray_monitor.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

