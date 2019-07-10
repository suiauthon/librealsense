list (APPEND CMAKE_MODULE_PATH "$ENV{CAMERA_SUITE_PACKAGE}")

find_package(CameraSuite REQUIRED)

include_directories(${CAMERASUITE_INCLUDE_DIRS})

target_sources(${LRS_TARGET}
        PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/cs-device.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-device.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-factory.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-factory.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-timestamp.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-timestamp.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-option.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-hw-monitor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-hw-monitor.h"
        "${CMAKE_CURRENT_LIST_DIR}/advanced_mode/cs-advanced-mode.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/advanced_mode/cs-advanced-mode.h"
        "${CAMERASUITE_INCLUDE_DIRS}/smcs_cpp/CameraSDK.cpp"
        )