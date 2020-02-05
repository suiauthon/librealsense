if (UNIX)
    set(ROS_WRAPPER_DIR /opt/Projects/RS-D4-ETH/Software/RS-ROS-Wrapper)
    set(SOURCE_DEST src/librealsense2)
    install(DIRECTORY
        ${ROS_WRAPPER_DIR}/
        DESTINATION ${SOURCE_DEST}/wrappers/ros/
    )
endif()
