exec_program("rosversion -d; echo" OUTPUT_VARIABLE ROS_VERSION)

add_custom_target(ikfast_libs DEPENDS urdf_openrave)
include_directories(include)

file(GLOB yamls "config/*.yaml")
file(WRITE plugins.xml "<class_libraries>\n")

if(ROS_VERSION MATCHES "electric")
    add_definitions(-DROS_VERSION_ELECTRIC)
endif()
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM true)
foreach(fname ${yamls})
    GET_FILENAME_COMPONENT(bname ${fname} NAME_WE)
    set(cppout ${CMAKE_CURRENT_SOURCE_DIR}/src/ikfast_${bname}.cpp)
    set(tname ikfast_${bname})

    add_custom_command(
    OUTPUT ${cppout}
    COMMAND python ${EXECUTABLE_OUTPUT_PATH}/genikfast.py ${fname} ${cppout}
    DEPENDS ${fname}
    )    

    add_library(${tname} MODULE ${cppout} ../ros/src/ikfast_plugin.cpp)
    set_target_properties(${tname} PROPERTIES COMPILE_DEFINITIONS IKFAST_NAMESPACE=${bname})
    target_link_libraries(${tname} lapack)

    file(APPEND plugins.xml "<library path=\"ros/lib/lib${tname}\">\n")
    file(APPEND plugins.xml "<class name=\"cob_kinematics/IKFast_${bname}\" type=\"${bname}::IKFastPlugin\" base_class_type=\"kinematics::KinematicsBase\"/>\n")
    file(APPEND plugins.xml "</library>\n")
endforeach()
file(READ ../cob_kinematics_plugins.xml cob_kinematics_plugins)
file(APPEND plugins.xml "${cob_kinematics_plugins}")
file(APPEND plugins.xml "</class_libraries>\n")
