if(pangolin_viewer_FOUND)
    message(STATUS "Viewer for examples: pangolin viewer")
endif()
if(socket_publisher_FOUND)
    message(STATUS "Viewer for examples: socket publisher")
endif()

set(EXECUTABLE_TARGETS "")

add_executable(run_slam run_slam.cc stella_vslam_ros.cc)
list(APPEND EXECUTABLE_TARGETS run_slam)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    add_dependencies(${EXECUTABLE_TARGET} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    # pangolin_viewer is used on a priority basis
    if(pangolin_viewer_FOUND)
        # set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE HAVE_PANGOLIN_VIEWER)
        # link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                pangolin_viewer::pangolin_viewer)
    endif()
    if(socket_publisher_FOUND)
        # set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE HAVE_SOCKET_PUBLISHER)
        # link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                socket_publisher::socket_publisher ${SIOCLIENT_LIBRARY} ${PROTOBUF_LIBRARIES})
    endif()

    # setup stack trace logger
    if(USE_STACK_TRACE_LOGGER)
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_STACK_TRACE_LOGGER)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${GLOG_LIBRARIES})
    endif()

    # setup google-perftools
    if(USE_GOOGLE_PERFTOOLS)
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_GOOGLE_PERFTOOLS)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${GPERFTOOLS_LIBRARIES})
    endif()

    # setup yaml-cpp
    target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
        yaml-cpp)

    if(spdlog_FOUND)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
        spdlog::spdlog)
    endif()
endforeach()
