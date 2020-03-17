if ( NOT DEFINED ENV{DYNAMIC_CALIBRATOR_PATH} )
	message( FATAL_ERROR "DYNAMIC_CALIBRATOR_PATH not deinfed" )
endif()

if (UNIX)
    install(PROGRAMS
        $ENV{DYNAMIC_CALIBRATOR_PATH}/bin/DynamicCalibrator
        DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(FILES
        $ENV{DYNAMIC_CALIBRATOR_PATH}/lib/libDSDynamicCalibrationAPI.so
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
elseif (WIN32)
	set(DYNAMIC_CALIBRATOR_DIR $ENV{DYNAMIC_CALIBRATOR_PATH})
	install(FILES 
		${DYNAMIC_CALIBRATOR_DIR}/DynamicCalibrator.exe
		${DYNAMIC_CALIBRATOR_DIR}/DSDynamicCalibrationAPI.dll
		${DYNAMIC_CALIBRATOR_DIR}/freeglut.dll
		DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
endif()
