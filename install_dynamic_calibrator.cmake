if ( NOT DEFINED ENV{DYNAMIC_CALIBRATOR_PATH} )
	message( FATAL_ERROR "DYNAMIC_CALIBRATOR_PATH not defined" )
endif()

if (UNIX)
    install(PROGRAMS
        $ENV{DYNAMIC_CALIBRATOR_PATH}/build/DynamicCalibrator
        DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(FILES
        $ENV{DYNAMIC_CALIBRATOR_PATH}/lib/libDSDynamicCalibrationAPI.so
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
elseif (WIN32)
	string (REPLACE "\\" "/" DYNAMIC_CALIBRATOR_DIR "$ENV{DYNAMIC_CALIBRATOR_PATH}")
	install(FILES 
		${DYNAMIC_CALIBRATOR_DIR}/build/Release/DynamicCalibrator.exe
		${DYNAMIC_CALIBRATOR_DIR}/lib/DSDynamicCalibrationAPI.dll
        ${DYNAMIC_CALIBRATOR_DIR}/3rdparty/glut/lib/freeglut.dll
		DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
endif()
