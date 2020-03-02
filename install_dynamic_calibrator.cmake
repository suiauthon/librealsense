if (UNIX)
    set(DYNAMIC_CALIBRATOR_DIR /opt/Projects/RS-D4-ETH/Software/RS-CalibrationToolAPI/linux/usr)
    install(PROGRAMS
        ${DYNAMIC_CALIBRATOR_DIR}/bin/DynamicCalibrator
        DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(FILES
        ${DYNAMIC_CALIBRATOR_DIR}/lib/libDSDynamicCalibrationAPI.so
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
