if (UNIX)
    set(SOURCE_DEST src/librealsense2)
    install(DIRECTORY
        $ENV{ROS_WRAPPER_PATH}/
        DESTINATION ${SOURCE_DEST}/wrappers/ros/
    )
endif()
