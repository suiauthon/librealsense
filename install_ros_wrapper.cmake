if (UNIX)
    if ( NOT DEFINED ENV{ROS_WRAPPER_PATH} )
        message( FATAL_ERROR "ROS_WRAPPER_PATH not defined" )
    endif()
    install(DIRECTORY
        $ENV{ROS_WRAPPER_PATH}/
        DESTINATION src/librealsense2/wrappers/ros/
		PATTERN ".git" EXCLUDE
    )
endif()
