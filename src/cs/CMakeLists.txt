list (APPEND CMAKE_MODULE_PATH "$ENV{CAMERA_SUITE_PACKAGE}")

find_package(CameraSuite REQUIRED)

include_directories(${CAMERASUITE_INCLUDE_DIRS})

target_sources(${LRS_TARGET}
        PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/cs.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs.h"
        "${CAMERASUITE_INCLUDE_DIRS}/smcs_cpp/CameraSDK.cpp"
        )