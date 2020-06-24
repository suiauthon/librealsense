# CPACK_PACKAGE_INSTALL_DIRECTORY must be changed at CPack time and not at CMake time because it contains a backslash
set(CPACK_PACKAGE_INSTALL_DIRECTORY "FRAMOS\\librealsense2")