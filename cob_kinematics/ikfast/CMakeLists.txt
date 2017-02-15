exec_program("rosversion -d; echo" OUTPUT_VARIABLE ROS_VERSION)

add_custom_target(ikfast_libs DEPENDS urdf_openrave)
include_directories(include)

set(plugins_xml ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins.xml)
file(GLOB yamls "config/*.yaml")
file(WRITE ${plugins_xml} "<class_libraries>\n")

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
    set_target_properties(${tname} PROPERTIES COMPILE_DEFINITIONS IKFAST_NAMESPACE=${bname} )
    target_link_libraries(${tname} lapack)

    install(TARGETS ${tname} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

    file(APPEND ${plugins_xml} "<library path=\"ros/lib/lib${tname}\">\n")
    file(APPEND ${plugins_xml} "<class name=\"cob_kinematics/IKFast_${bname}\" type=\"${bname}::IKFastPlugin\" base_class_type=\"kinematics::KinematicsBase\"/>\n")
    file(APPEND ${plugins_xml} "</library>\n")
endforeach()
file(APPEND ${plugins_xml} "</class_libraries>\n")

install(FILES ${plugins_xml} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
