FRAMOS D400e Depth Camera Software Package for Windows
v1.8.0 (2020-03-31)
======================================================

Components:
-----------
- FRAMOS CameraSuite
- Intel® RealSense™ SDK with support for D400e cameras

Supported platforms:
--------------------
- Windows x86_64

Prerequisites:
--------------
- D400e series camera

Installation:
-------------
1. Install FRAMOS CameraSuite
    - please uninstall any previous version of CameraSuite before proceeding 
        with installation
    - make sure to install the FRAMOS GigEVision driver when prompted

2. Install Intel® RealSense™ SDK with support for D400e cameras
    - please uninstall other versions of Intel® RealSense™ SDK before proceeding 
        with installation

Starting the application:
-------------------------
After installation, launch Intel® RealSense™ Viewer application:
    realsense-viewer.exe, located in folder
    "C:\Program Files\FRAMOS-librealsense2 2.29.6\bin". 
    Other tools and examples are located in the same folder.
    
Building Intel® RealSense™ library, examples and tools:
-------------------------------------------------------
Following tools are required for building the Intel® RealSense™ library:
	Visual Studio 2015
	CMake

Sources for Intel® RealSense™ library, tools and examples are located in
	C:\Program Files\FRAMOS-librealsense2 2.29.6\src

This folder is owned by the administrator. Before proceeding with the build, 
copy it to a location owner by the user.

Launch the CMake GUI tool.
Set "Where is the source code" to the location of the copied src folder.
Set "Where to build the binaries" to the location of the copied src folder with the "/build" suffix.
Press the "Configure" button to update CMake values. 
Make sure to select the "Visual Studio 14 2015" generator and "x64" optional platform when prompted.
Press the "Generate" button to generate the Visual Studio Solution.
Press the "Open Project" button to open the generated Visual Studio Solution.
Inside the solution, set the build type to "Release" and build it by selecting "Build" -> "Build Solution".

D400e features in the librealsense2 API:
----------------------------------------
Intel® RealSense™ SDK with support for D400e cameras extends the standard librealsense2 API 
with features applicable only to the D400e cameras. List of all API additions is located in
    C:\Program Files\FRAMOS-librealsense2 2.29.6\src\d400e_api_extensions.md

Python wrapper:
---------------
Instructions on how to use the Python wrapper are located in
    C:\Program Files\FRAMOS-librealsense2 2.29.6\src\wrappers\python\d400e_readme.md

Multicam example:
-----------------
Notes on configuring a multi camera setup are located in
    C:\Program Files\FRAMOS-librealsense2 2.29.6\examples\multicam\readme.md

Using Dynamic Calibration Tool:
-------------------------------
Dynamic Calibration Tool is located in 
	"C:\Program Files\FRAMOS-librealsense2 2.29.6\bin\DynamicCalibrator"
	
Calibration of RGB imager is currently not supported.

Calibrating multiple cameras in a row is currently not supported.
Dynamic Calibration Tool needs to be restarted after each calibration.

Troubleshooting:
----------------
