set(EXECUTABLE_TARGETS "")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(run_slam run_slam.cc stella_vslam_ros.cc)
add_executable(run_slam_offline run_slam_offline.cc stella_vslam_ros.cc)
list(APPEND EXECUTABLE_TARGETS run_slam run_slam_offline)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
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
    if(iridescence_viewer_FOUND)
        # Set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE HAVE_IRIDESCENCE_VIEWER)
        # Link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                iridescence_viewer::iridescence_viewer)
    endif()

    # setup stack trace logger
    if(USE_STACK_TRACE_LOGGER)
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_STACK_TRACE_LOGGER)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE Backward::Backward)
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

    # build
    target_include_directories(${EXECUTABLE_TARGET}
                               PRIVATE ${rclcpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${EXECUTABLE_TARGET}
                          PRIVATE stella_vslam::stella_vslam ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})
    ament_target_dependencies(${EXECUTABLE_TARGET}
                              PUBLIC
                              rclcpp
                              cv_bridge
                              image_transport
                              message_filters
                              rcutils
                              geometry_msgs
                              nav_msgs
                              sensor_msgs
                              tf2
                              tf2_eigen
                              tf2_geometry_msgs
                              tf2_msgs
                              tf2_ros
                              rosbag2_cpp)
    install(TARGETS ${EXECUTABLE_TARGET}
            DESTINATION lib/${PROJECT_NAME})

endforeach()
