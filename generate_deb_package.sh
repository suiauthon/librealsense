#!/bin/bash -e 

enter_script_folder() {
    cd $(dirname "$BASH_SOURCE[0]")
    BASEDIR=$(pwd)
}

set_platform() {
    case "$1" in
    "Linux64_x64" | "Linux64_ARM" | "Linux32_ARM")
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
    if [[ $DOCKER_BUILD = true ]]; then
        sudo DEBIAN_FRONTEND=noninteractive apt-get install --assume-yes /camerasuite_package/FRAMOS_CameraSuite_*-Linux64_x64.deb
        source /etc/profile.d/camerasuite.sh
    elif [[ -d /opt/Projects/CameraSuite ]]; then
        export CAMERA_SUITE_PATH=/opt/Projects/CameraSuite
        export CAMERA_SUITE_PACKAGE=/opt/Projects/CameraSuite/CS_SDK/cmake_packages
        export LD_LIBRARY_PATH=/media/L/GenICam/V3_0_2/bin/$PLATFORM
        export DYNAMIC_CALIBRATOR_PATH=/opt/Projects/RS-D4-ETH/Software/RS-CalibrationToolAPI/linux/usr
        export ROS_WRAPPER_PATH=/opt/Projects/RS-D4-ETH/Software/RS-ROS-Wrapper
    fi
}

build_realsense2 () {
    mkdir -p build/$PLATFORM
    pushd build/$PLATFORM
    CMAKE_FLAGS="$CMAKE_FLAGS -DROS_WRAPPER_PATH=$ROS_WRAPPER_PATH -DROS2_WRAPPER_PATH=$ROS2_WRAPPER_PATH"
    if [[ "$PLATFORM" = "Linux64_ARM" ]]; then
        export CAMERA_SUITE_TARGET_SYSTEM="$PLATFORM"
        TOOLCHAIN="Linux64_ARM_HF_Toolchain.cmake"
        CMAKE_FLAGS="$CMAKE_FLAGS -DCPACK_DEBIAN_PACKAGE_ARCHITECTURE=arm64"
    elif [[ "$PLATFORM" = "Linux32_ARM" ]]; then
        export CAMERA_SUITE_TARGET_SYSTEM="$PLATFORM"
        TOOLCHAIN="Linux32_ARM_HF_Toolchain.cmake"
        CMAKE_FLAGS="$CMAKE_FLAGS -DCPACK_DEBIAN_PACKAGE_ARCHITECTURE=armhf"
    else 
		CMAKE_FLAGS="$CMAKE_FLAGS -DDYNAMIC_CALIBRATOR_PATH=$DYNAMIC_CALIBRATOR_PATH"
	fi
    cmake ../../ -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" -DCPACK_SYSTEM_NAME=$PLATFORM $CMAKE_FLAGS
    make -j$(nproc)
    popd
}

build_dynamic_calibrator() {
    if [[ "$PLATFORM" = "Linux64_x64" ]]; then
        mkdir -p "$DYNAMIC_CALIBRATOR_PATH/build"
        pushd "$DYNAMIC_CALIBRATOR_PATH/build"
        cmake -DLIBRS_INCLUDE_DIR="$BASEDIR/include" -DLIBRS_LIBRARY_DIR="$BASEDIR/build/$PLATFORM" -DCMAKE_BUILD_TYPE=Release ..
        make -j$(nproc)
        popd
    fi
}

pack() {
    pushd build/$PLATFORM
    cpack -G DEB
    popd
}

enter_script_folder
set_platform "$1"
set_build_environment
build_realsense2
build_dynamic_calibrator
pack