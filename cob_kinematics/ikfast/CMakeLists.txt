add_custom_target(ikfast_libs DEPENDS urdf_openrave)
include_directories(include)
file(GLOB yamls "config/*.yaml")
foreach(fname ${yamls})
    GET_FILENAME_COMPONENT(bname ${fname} NAME_WE)
    set(cppout ${CMAKE_CURRENT_SOURCE_DIR}/src/${bname}_ikfast.cpp)
    add_custom_command(
    OUTPUT ${cppout}
    COMMAND python ${EXECUTABLE_OUTPUT_PATH}/genikfast.py ${fname} ${cppout}
    DEPENDS ${fname}
    )    
    add_library(${bname}_ikfast MODULE ${cppout})
    target_link_libraries(${bname}_ikfast lapack)
endforeach()