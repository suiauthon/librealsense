#!/bin/bash

export CAMERA_SUITE_PACKAGE_PATH=/opt/Projects/CameraSuite/CS_Build/setup/Linux/Output
docker-compose run realsense_arm64 ./generate_deb_package.sh Linux64_ARM
