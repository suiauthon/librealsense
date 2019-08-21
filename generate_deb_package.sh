#!/bin/bash -e 

(
    mkdir -p build
    cd build

    # for CS development machines
    if [[ -z $CAMERA_SUITE_PATH  ]]; then
        export CAMERA_SUITE_PATH=/opt/Projects/CameraSuite
    fi
    if [[ -z $CAMERA_SUITE_PACKAGE ]]; then
        export CAMERA_SUITE_PACKAGE=/opt/Projects/CameraSuite/CS_SDK/cmake_packages
    fi
    export LD_LIBRARY_PATH=/media/L/GenICam/V3_0_2/bin/Linux64_x64:/media/L/OpenCV/builds/Linux64_x64/lib

    cmake ../ -DCMAKE_BUILD_TYPE=Release -DCPACK_SYSTEM_NAME=Linux64_x64
    make -j4
    cpack -G DEB
)
