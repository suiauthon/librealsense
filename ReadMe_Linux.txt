FRAMOS D400e Depth Camera Software Package for Linux
v1.8.0 (2020-03-31)
====================================================

Components:
-----------
- FRAMOS CameraSuite
- Intel® RealSense™ SDK with support for D400e cameras

Supported platforms:
--------------------
- Linux x86_64
- Linux ARM64

Prerequisites:
--------------
- D400e series camera

Installation:
-------------
1. Make sure that kernel headers are installed on the system

    sudo apt-get update && sudo apt-get install linux-headers-$(uname -r)

2. Install FRAMOS CameraSuite

    - please uninstall any previous version of CameraSuite before proceeding 
        with installation
    - make sure to install the FRAMOS GigEVision driver and add current user to 
        'video' group when prompted
    sudo apt install ./FRAMOS_CameraSuite*.deb

3. Reboot the system

4. Install Intel® RealSense™ SDK with support for D400e cameras
    - please uninstall other verions of Intel® RealSense™ SDK before proceeding 
        with installation
    sudo apt install ./FRAMOS-librealsense2*.deb

Starting the application:
-------------------------
After installation, launch Intel® RealSense™ Viewer from terminal:
    realsense-viewer
Other Intel® RealSense™ examples and tools are available from terminal as well.

Building Intel® RealSense™ library, examples and tools:
-------------------------------------------------------
Install required packages:
    sudo apt install gcc g++ cmake libglfw3-dev libgtk-3-dev git libssl-dev libusb-1.0-0-dev pkg-config

Additional packages are required on Ubuntu 18:
    sudo apt install libgl1-mesa-dev libglu1-mesa-dev

Sources for Intel® RealSense™ library, examples and tools are located in
    /usr/src/librealsense2

Copy the sources to a folder with user permissions, for example home:
    cp -r /usr/src/librealsense2 ~

Create a build folder:
    cd librealsense2
    mkdir build
    cd build

Generate Makefiles with cmake and specify build type:
    cmake .. -DCMAKE_BUILD_TYPE=Release

Build the project with make:
    make -j$(nproc)

D400e features in the librealsense2 API:
----------------------------------------
Intel® RealSense™ SDK with support for D400e cameras extends the standard librealsense2 API 
with features applicable only to the D400e cameras. List of all API additions is located in
    /usr/src/librealsense2/d400e_api_extensions.md

Python wrapper:
---------------
Instructions on how to use the Python wrapper are located in
    /usr/src/librealsense2/wrappers/python/d400e_readme.md

ROS wrapper:
------------
Instructions on how to use the ROS wrapper are located in
    /usr/src/librealsense2/wrappers/ros/d400e_readme.md

Multicam example:
-----------------
Notes on configuring a multi camera setup are located in
    /usr/src/librealsense2/examples/multicam/readme.md

Using Dynamic Calibration Tool:
-------------------------------
Launch the Dynamic Calibration Tool from terminal:
    DynamicCalibrator

Calibration of RGB imager is currently not supported.

Calibrating multiple cameras in a row is currently not supported.
Dynamic Calibration Tool needs to be restarted after each calibration.

Dynamic Calibration Tool requires additional libraries on Ubuntu 18.
Install the required libraries:
    wget http://security.ubuntu.com/ubuntu/pool/main/libp/libpng/libpng12-0_1.2.54-1ubuntu1.1_amd64.deb
    sudo apt install ./libpng12-0_1.2.54-1ubuntu1.1_amd64.deb

Dynamic Calibration Tool depends on the freeglut3. If the universe repository is not enabled in apt, 
it will not be installed automatically during installation. Enable the repository and install freeglut3 manually:
    sudo add-apt-repository universe
    sudo apt update
    sudo apt install freeglut3

Reverse path filtering:
-----------------------
Reverse path filtering is a security mechanism that drops incoming packets whose source address is not routable. 
It can help prevent IP address spoofing.
However, this security mechanism prevents discovery of cameras that are not on the same subnet as the network interface 
they are connected to. Because of that, it may be necessary to disable reverse path filtering during the initial setup 
of camera IP addresses and subnets. After the cameras are properly configured, it is recommended to enable reverse path 
filtering again.
Reverse path filtering can be disabled during installation of CameraSuite.
Scripts for disabling and reenabling reverse path filtering are also available:
    /usr/src/framos/camerasuite/Scripts/disable_rp_filter.sh
    /usr/src/framos/camerasuite/Scripts/revert_disable_rp_filter.sh
    
Troubleshooting:
----------------
Issue: Driver not installed
Cause: DKMS framework not installed
Solution:
- Install the DKMS framework manually:
    sudo apt install dkms
- Install the driver:
    cd /usr/src/framos/camerasuite/Scripts
    sudo ./install_gige_driver.sh
Workaround:
- Driver can also be installed without the DKMS framework.
- Installing without the DKMS framework has the following drawbacks:
    - Driver will not be able to load if Secure Boot is enabled because it is not signed.
    - Driver will not load after every kernel upgrade. It will have to be reinstalled manually every time.
- Install the driver without the DKMS framework:
    cd /usr/src/framos/camerasuite/GigeVisionDriver
    sudo make install-driver
- Do not attempt to install the driver using both the install_gige_driver script and the explicit call to make.

Issue: Driver not installed
Cause: Kernel headers not installed
Solution:
- Install kernel headers for current kernel:
    sudo apt install linux-headers-$(uname -r)
- Install the driver:
    cd /usr/src/framos/camerasuite/Scripts
    sudo ./install_gige_driver.sh

Issue: Driver not installed
Cause: Driver build failed
Solution: 
- Installing on an unsupported kernel. Contact support.

Issue: GigE filter driver not loaded! when starting the stream
Cause: Filter driver not available.
Solution:
- Make sure that the driver is installed and loaded:
    lsmod | grep gigedrvframos
- Make sure that the user running the application is a member of video group:
    groups | grep video
- Make sure that no other application that uses the GigE filter driver is running

Issue: GenICam_3_0::RuntimeException when starting an application
Cause: Environment variables not set properly
Solution:
- Make sure to reboot the system after installation to load the required environment variables.
- Do not run the application with sudo. This will hide the user environment variables. 
    - Alternatively, pass the -E flag to sudo command to preserve the environment variables.
