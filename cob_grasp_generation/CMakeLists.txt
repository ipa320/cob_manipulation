cmake_minimum_required(VERSION 3.0.2)
project(cob_grasp_generation)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()


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
        scripts/show_grasps_rviz_client.py
        scripts/show_grasps_rviz_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
