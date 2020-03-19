if (UNIX)
    if ( NOT DEFINED ENV{DYNAMIC_CALIBRATOR_PATH} )
        message( FATAL_ERROR "DYNAMIC_CALIBRATOR_PATH not deinfed" )
    endif()
    install(PROGRAMS
        $ENV{DYNAMIC_CALIBRATOR_PATH}/bin/DynamicCalibrator
        DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(FILES
        $ENV{DYNAMIC_CALIBRATOR_PATH}/lib/libDSDynamicCalibrationAPI.so
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
elseif (WIN32)
	set(DYNAMIC_CALIBRATOR_DIR C:/SmartekVision/Projects/RS-D4-ETH/Software/RS-CalibrationToolAPI/windows/bin)
	install(FILES 
		${DYNAMIC_CALIBRATOR_DIR}/DynamicCalibrator.exe
		${DYNAMIC_CALIBRATOR_DIR}/DSDynamicCalibrationAPI.dll
		${DYNAMIC_CALIBRATOR_DIR}/freeglut.dll
		DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
endif()
