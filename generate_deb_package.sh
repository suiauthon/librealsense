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

    cmake ../ -DCMAKE_BUILD_TYPE=Release -DCPACK_SYSTEM_NAME=Linux64_x64
    make -j4
    cpack -G DEB
)
