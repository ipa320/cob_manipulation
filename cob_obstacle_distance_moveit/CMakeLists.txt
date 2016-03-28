cmake_minimum_required(VERSION 2.8.3)
project(cob_obstacle_distance_moveit)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        cob_control_msgs
        cob_srvs
        eigen_conversions
        geometric_shapes
        geometry_msgs
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

catkin_package()

###########
## Build ##
###########

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

