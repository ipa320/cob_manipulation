cmake_minimum_required(VERSION 2.8.3)
project(obstacle_distance)

find_package(catkin REQUIRED
    COMPONENTS
        atf_msgs
        message_generation
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        roscpp
        tf
)

find_package(Boost REQUIRED
    COMPONENTS
        filesystem
        system
        REQUIRED)
find_package(fcl REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
    FILES
        GetObstacleDistance.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
        atf_msgs
)

###################################
## catkin specific configuration ##
###################################

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

catkin_package(
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning_interface
        roscpp
        atf_msgs
    DEPENDS
        Boost
        fcl
    INCLUDE_DIRS
        include
)

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
add_dependencies(obstacle_distance_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_distance_node ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES})
