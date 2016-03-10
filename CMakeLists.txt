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
        moveit_ros_perception
        moveit_ros_planning_interface
        roscpp
        tf
        tf_conversions
)

find_package(Boost REQUIRED COMPONENTS)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})
find_package(fcl REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
    FILES
        GetObstacleDistance.srv
)
 
add_message_files(
    FILES
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

catkin_package()

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_distance_node src/${PROJECT_NAME}/obstacle_distance.cpp include/${PROJECT_NAME}/obstacle_distance.h)
add_dependencies(obstacle_distance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_distance_node ${catkin_LIBRARIES} ${fcl_LIBRARIES})

add_executable(visualize_obstacle_distance_node src/visualize_obstacle_distance_node.cpp)
add_dependencies(visualize_obstacle_distance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_obstacle_distance_node ${catkin_LIBRARIES})

add_executable(test_obstacle_publisher_node src/test_obstacle_publisher_node.cpp)
add_dependencies(test_obstacle_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_obstacle_publisher_node ${catkin_LIBRARIES})

