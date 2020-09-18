#!/bin/bash -e

check_params() {
    if [[ ! -d $CAMERA_SUITE_PACKAGE_PATH ]]; then
        echo "Variable CAMERA_SUITE_PACKAGE_PATH does not point to a valid folder"
        exit 1
    fi
    if [[ ! -f $CAMERA_SUITE_CHANGELOG ]]; then
        echo "Variable CAMERA_SUITE_CHANGELOG does not point to a valid file"
        exit 1
    fi
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

generate_tar_package() {
    rm -rf build/$PLATFORM/FRAMOS_D400e_Software_Package
    mkdir build/$PLATFORM/FRAMOS_D400e_Software_Package
    pushd build/$PLATFORM/FRAMOS_D400e_Software_Package
    cp "$CAMERA_SUITE_PACKAGE_PATH"/*$PLATFORM.deb .
    cp ../*.deb .
    cp ../../../FRAMOS_D400e_Software_Package_Changelog.txt .
    cp ../../../FRAMOS_LibRealSense_Changelog.txt .
    cp "${CAMERA_SUITE_CHANGELOG}" FRAMOS_CameraSuite_Changelog.txt
	cp ../../../ReadMe_Linux.txt ReadMe.txt
    package_version=$(head -n 1 FRAMOS_D400e_Software_Package_Changelog.txt | cut -d " " -f1 | tr . -)
    popd
    pushd build/$PLATFORM
    tar czf FRAMOS_D400e_Software_Package_${package_version}_$PLATFORM.tar.gz FRAMOS_D400e_Software_Package --owner=0 --group=0
    popd
    cp build/$PLATFORM/*.tar.gz .
}

check_params
set_platform "$1"
generate_tar_package