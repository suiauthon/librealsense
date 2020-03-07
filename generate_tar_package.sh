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

generate_tar_package() {
    rm -rf FRAMOS_D400e_Software_Package
    mkdir FRAMOS_D400e_Software_Package
    pushd FRAMOS_D400e_Software_Package
    cp "$CAMERA_SUITE_PACKAGE_PATH"/*$1.deb .
    cp ../build/$1/*.deb .
    cp ../FRAMOS_D400e_Software_Package_Changelog.txt .
    cp ../FRAMOS_LibRealSense_Changelog.txt .
    cp "${CAMERA_SUITE_CHANGELOG}" FRAMOS_CameraSuite_Changelog.txt
    package_version=$(head -n 1 FRAMOS_D400e_Software_Package_Changelog.txt | cut -d " " -f1 | tr . -)
    popd
    tar czf FRAMOS_D400e_Software_Package_${package_version}_$1.tar.gz FRAMOS_D400e_Software_Package
}

check_params
generate_tar_package Linux64_x64
generate_tar_package Linux64_ARM
