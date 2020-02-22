#!/bin/bash -e 

set_default_variables() {

    if [[ -z $CAMERA_SUITE_PATH  ]]; then
        export CAMERA_SUITE_PATH=/opt/Projects/CameraSuite
    fi
    if [[ -z $CAMERA_SUITE_PACKAGE ]]; then
        export CAMERA_SUITE_PACKAGE=/opt/Projects/CameraSuite/CS_SDK/cmake_packages
    fi
}

build_platform () {

    case "$1" in
        "Linux64_x64" | "Linux64_ARM")
            PLATFORM="$1"
            ;;
        *)
            echo "Attempting to build unsupported platform $1" 
            exit
    esac

    (
        mkdir -p build/$PLATFORM
        cd build/$PLATFORM
        export LD_LIBRARY_PATH=/media/L/GenICam/V3_0_2/bin/$PLATFORM
        if [[ "$PLATFORM" = "Linux64_ARM" ]]; then
            export CAMERA_SUITE_TARGET_SYSTEM="$PLATFORM" 
            TOOLCHAIN="$CAMERA_SUITE_PATH/CS_SDK/toolchains/Linux64_ARM_HF_Toolchain.cmake"
        fi
        cmake ../../ -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" -DCPACK_SYSTEM_NAME=$PLATFORM
        make -j$(nproc)
        cpack -G DEB 
    )
}

set_default_variables
build_platform Linux64_x64
#build_platform Linux64_ARM
