cmake_minimum_required(VERSION 2.8.3)
project(obstacle_distance)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        cob_srvs
        eigen_conversions
        geometric_shapes
        geometry_msgs
        message_generation
        moveit_core
        moveit_ros_planning_interface
        roscpp
        tf
        tf_conversions
)

find_package(Boost REQUIRED COMPONENTS)

find_package(fcl REQUIRED)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
    FILES
        GetObstacleDistance.srv
)
 
add_message_files(
  FILES
  Chain.msg
  DistanceInfo.msg
  DistanceInfos.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

catkin_package()

###########
## Build ##
###########

include_directories(
    include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${FCL_INCLUDE_DIRS}
)
link_directories(
    ${catkin_LIBRARY_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${FCL_INCLUDE_DIRS}
)

add_executable(obstacle_distance_node src/${PROJECT_NAME}/obstacle_distance.cpp include/${PROJECT_NAME}/obstacle_distance.h)
add_dependencies(obstacle_distance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_distance_node ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES})

add_executable(visualize_obstacle_distance_node src/visualize_obstacle_distance_node.cpp)
add_dependencies(visualize_obstacle_distance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_obstacle_distance_node ${catkin_LIBRARIES})

add_executable(test_obstacle_publisher_node src/test_obstacle_publisher_node.cpp)
add_dependencies(test_obstacle_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_obstacle_publisher_node ${catkin_LIBRARIES})

