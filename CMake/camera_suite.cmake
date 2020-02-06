macro(add_camerasuite)
    list (APPEND CMAKE_MODULE_PATH "$ENV{CAMERA_SUITE_PACKAGE}")
    find_package(CameraSuite REQUIRED)
    include_directories(${CAMERASUITE_INCLUDE_DIRS})
    target_sources(${ARGV0} PRIVATE "${CAMERASUITE_INCLUDE_DIRS}/smcs_cpp/CameraSDK.cpp")
    target_include_directories(${ARGV0} PRIVATE ${CAMERASUITE_INCLUDE_DIRS})
    target_link_libraries(${ARGV0} PUBLIC ${CAMERASUITE_LIBRARY})
endmacro()
