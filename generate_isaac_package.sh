#!/bin/bash -e

if [[ ! -d $ISAAC_PATH ]]; then
    echo "Variable ISAAC_PATH does not point to a valid folder"
    exit 1
fi

for folder in $CAMERA_SUITE_UNPACKED_PATH; do
    if [[ -d $folder ]]; then
        expanded_camera_suite_unpacked_path="$folder"
        break
    fi
done
if [[ -z $expanded_camera_suite_unpacked_path ]]; then
    echo "Variable CAMERA_SUITE_UNPACKED_PATH does not point to a valid file"
    exit 1
fi

for folder in cp -r build/Linux64_x64/_CPack_Packages/Linux64_x64/DEB/FRAMOS-librealsense2-*-Linux64_x64; do
    if [[ -d $folder ]]; then
        expanded_realsense_unpacked_path=$(realpath "$folder")
        break
    fi
done
if [[ -z $expanded_realsense_unpacked_path ]]; then
    echo "librealsense2 not packed, unable to get source"
    exit 1
fi

rm -rf build/FRAMOS_D400e_Isaac_Package
mkdir build/FRAMOS_D400e_Isaac_Package

pushd build/FRAMOS_D400e_Isaac_Package
cp -r "$ISAAC_PATH"/framos_rs-2 .
cp "$ISAAC_PATH"/Readme.md .
cp "$ISAAC_PATH"/Changelog.txt .

mkdir -p librealsense2-for-isaac
pushd librealsense2-for-isaac
cp -r "$expanded_realsense_unpacked_path"/usr/src/librealsense2 .
mkdir -p librealsense2/third-party/camerasuite
pushd librealsense2/third-party/camerasuite
cp -r "$expanded_camera_suite_unpacked_path"/include .
mkdir -p lib/genicam
pushd lib/genicam
cp -r "$expanded_camera_suite_unpacked_path"/lib/genicam/Linux64_x64 .
cp -r "$expanded_camera_suite_unpacked_path"/lib/genicam/Linux64_ARM .
popd
mkdir -p lib/camerasuite
pushd lib/camerasuite
cp -r "$expanded_camera_suite_unpacked_path"/lib/camerasuite/Linux64_x64 .
cp -r "$expanded_camera_suite_unpacked_path"/lib/camerasuite/Linux64_ARM .
popd
popd
popd

package_version=$(head -n 1 Changelog.txt | cut -d " " -f1 | tr . -)
popd

pushd build
tar czf FRAMOS_D400e_Isaac_Package_${package_version}_Linux64_x64.tar.gz FRAMOS_D400e_Isaac_Package --owner=0 --group=0
popd