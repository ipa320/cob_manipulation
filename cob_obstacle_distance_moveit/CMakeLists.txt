cmake_minimum_required(VERSION 3.0.2)
project(cob_obstacle_distance_moveit)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        cob_control_msgs
        cob_srvs
        eigen_conversions
        geometric_shapes
        geometry_msgs
        moveit_core
        moveit_msgs
        moveit_ros_perception
        moveit_ros_planning_interface
        roscpp
        tf
        tf2_ros
        tf_conversions
)

find_package(Boost REQUIRED COMPONENTS)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(fcl_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

catkin_package(
  CATKIN_DEPENDS cob_control_msgs cob_srvs geometry_msgs moveit_msgs roscpp tf
)


### Build ###
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(obstacle_distance_moveit src/${PROJECT_NAME}/obstacle_distance_moveit.cpp)
add_dependencies(obstacle_distance_moveit ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_distance_moveit ${catkin_LIBRARIES} ${fcl_LIBRARIES})

add_executable(obstacle_distance_node src/obstacle_distance_moveit_node.cpp)
add_dependencies(obstacle_distance_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_distance_node obstacle_distance_moveit ${catkin_LIBRARIES})

add_executable(visualize_obstacle_distance_node src/visualize_obstacle_distance_node.cpp)
add_dependencies(visualize_obstacle_distance_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_obstacle_distance_node ${catkin_LIBRARIES})

add_executable(test_obstacle_publisher_node src/test_obstacle_publisher_node.cpp)
add_dependencies(test_obstacle_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_obstacle_publisher_node ${catkin_LIBRARIES})


### INSTALL ###
install(TARGETS obstacle_distance_moveit obstacle_distance_node test_obstacle_publisher_node visualize_obstacle_distance_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/test_obstacle_publisher.py scripts/test_registration_client.py scripts/test_service_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY files launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
