#!/bin/bash

export CAMERA_SUITE_PACKAGE_PATH=/opt/Projects/CameraSuite/CS_Build/setup/Linux/Output
export DYNAMIC_CALIBRATOR_PATH=/opt/Projects/RS-D4-ETH/Software/RS-CalibrationToolAPI/linux/usr
export ROS_WRAPPER_PATH=/opt/Projects/RS-D4-ETH/Software/RS-ROS-Wrapper
docker-compose run realsense ./generate_deb_package.sh Linux64_x64
