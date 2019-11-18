# Python Wrapper for FRAMOS D435e camera

This readme file provides instructions on how to use the D435e camera with Python.
Building the wrapper from source is currently not supported on Linux.

## Supported platforms

Ubuntu 16.04 x86_64

Windows x86_64

## Prerequisites

FRAMOS CameraSuite version 4.1.2.0 or higher

Intel® RealSense™ SDK with support for D435e camera version 2.29.1 or higher

FRAMOS D435e Camera with firmware version 1.4.0.0 or higher

## Linux

This is a temporary workaround for using the Python wrapper on Linux.

1. Install Python development packages
   
   ```
   sudo apt install python3 python3-dev
   ```

2. Install `pyrealsense2` using `pip`

    ```
    sudo pip3 install pyrealsense2
    ```

3. Locate the folder in which `pyrealsense2` was installed

    ```
    cd /usr/local/lib/python3.X/dist-packages
    ```

4. Delete `pyrealsense2-*` folder

    ```
    sudo rm -r pyrealsense2-*
    ```

5. Delete `pyrealsense2.*` file

    ```
    sudo rm pyrealsense2/pyrealsense2.*
    ```

6. Clone and build Intel® RealSense™ SDK 2.29.0

    ```
    cd ~
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense
    git checkout v2.29.0
    mkdir build
    cd build
    cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3.X
    make -j4
    sudo make install
    ```

7. Copy the built 'pyrealsense.so' file

    ```
    sudo cp /usr/local/lib/pyrealsense2.* /usr/local/lib/python3.X/dist-packages/pyrealsense2/
    ```

8. Uninstall Intel® RealSense™ SDK 2.29.0

    ```
    sudo make uninstall
    ```

## Windows

Follow the official instructions.