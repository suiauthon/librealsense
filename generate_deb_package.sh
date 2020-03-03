#!/bin/bash -e 

set_platform() {
    case "$1" in
    "Linux64_x64" | "Linux64_ARM")
        PLATFORM="$1"
        ;;
    "")
        PLATFORM="Linux64_x64"
        ;;
    *)
        echo "Attempting to build unsupported platform $1"
        exit
    esac
}

set_build_environment() {
    if [[ -z $DOCKER_BUILD ]]; then
        export CAMERA_SUITE_PATH=/opt/Projects/CameraSuite
        export CAMERA_SUITE_PACKAGE=/opt/Projects/CameraSuite/CS_SDK/cmake_packages
        export LD_LIBRARY_PATH=/media/L/GenICam/V3_0_2/bin/$PLATFORM
        export DYNAMIC_CALIBRATOR_PATH=/opt/Projects/RS-D4-ETH/Software/RS-CalibrationToolAPI/linux/usr
        export ROS_WRAPPER_PATH=/opt/Projects/RS-D4-ETH/Software/RS-ROS-Wrapper
    else
        DEBIAN_FRONTEND=noninteractive apt-get install --assume-yes /camerasuite_package/FRAMOS_CameraSuite_*-Linux64_x64.deb
        source /etc/profile.d/camerasuite.sh
    fi
}

build_platform () {    
    mkdir -p build/$PLATFORM
    pushd build/$PLATFORM
    if [[ "$PLATFORM" = "Linux64_ARM" ]]; then
        export CAMERA_SUITE_TARGET_SYSTEM="$PLATFORM"
        TOOLCHAIN="Linux64_ARM_HF_Toolchain.cmake"
        CMAKE_FLAGS="-DINSTALL_DYNAMIC_CALIBRATOR=OFF"
    fi
    cmake ../../ -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" -DCPACK_SYSTEM_NAME=$PLATFORM $CMAKE_FLAGS
    make -j$(nproc)
    cpack -G DEB
    popd
}

set_platform "$1"
set_build_environment
build_platform