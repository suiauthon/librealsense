#!/bin/bash -e 

set_build_environment() {

    if [[ -z $DOCKER_BUILD ]]; then
        export CAMERA_SUITE_PATH=/opt/Projects/CameraSuite
        export CAMERA_SUITE_PACKAGE=/opt/Projects/CameraSuite/CS_SDK/cmake_packages
        export LD_LIBRARY_PATH=/media/L/GenICam/V3_0_2/bin/$PLATFORM
    else
        DEBIAN_FRONTEND=noninteractive apt-get install --assume-yes /camerasuite_package/FRAMOS_CameraSuite_*-Linux64_x64.deb
        source /etc/profile.d/camerasuite.sh
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
        if [[ "$PLATFORM" = "Linux64_ARM" ]]; then
            export CAMERA_SUITE_TARGET_SYSTEM="$PLATFORM" 
            TOOLCHAIN="Linux64_ARM_HF_Toolchain.cmake"
        fi
        cmake ../../ -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" -DCPACK_SYSTEM_NAME=$PLATFORM
        make -j$(nproc)
        cpack -G DEB
    )
}

set_build_environment
build_platform "$1"