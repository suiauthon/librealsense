macro(add_camerasuite)
    list (APPEND CMAKE_MODULE_PATH "$ENV{CAMERA_SUITE_PACKAGE}")
    find_package(CameraSuite REQUIRED)
    include_directories(${CAMERASUITE_INCLUDE_DIRS})
    target_sources(${LRS_TARGET} PRIVATE "${CAMERASUITE_INCLUDE_DIRS}/smcs_cpp/CameraSDK.cpp")
    target_include_directories(${LRS_TARGET} PRIVATE ${CAMERASUITE_INCLUDE_DIRS})
    target_link_libraries(${LRS_TARGET} PRIVATE ${CAMERASUITE_LIBRARY})
    if (UNIX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--unresolved-symbols=ignore-in-shared-libs")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--unresolved-symbols=ignore-in-shared-libs")
    endif()
endmacro()