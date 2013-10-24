set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)
include_directories(${PROJECT_SOURCE_DIR}/lookat_ik/include)

add_library(lookat_ik_kinematics_plugin src/lookat_ik_kinematics_plugin.cpp)
target_link_libraries(lookat_ik_kinematics_plugin moveit_kdl_kinematics_plugin)
