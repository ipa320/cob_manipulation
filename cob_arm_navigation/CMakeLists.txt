cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(cob_environment_server src/cob_environment_server.cpp)
rosbuild_link_boost(cob_environment_server thread)
target_link_libraries(cob_environment_server planning_environment)

rosbuild_add_executable(planning_description_generator src/planning_description_generator.cpp)
target_link_libraries(planning_description_generator planning_environment)

# rostest
rosbuild_add_roslaunch_check(launch ROBOT=cob3-1)
rosbuild_add_roslaunch_check(launch ROBOT=cob3-2)
rosbuild_add_roslaunch_check(launch ROBOT=cob3-3)
rosbuild_add_roslaunch_check(launch ROBOT=cob3-4)
rosbuild_add_roslaunch_check(launch ROBOT=cob3-6)
rosbuild_add_roslaunch_check(launch ROBOT=raw3-1)
