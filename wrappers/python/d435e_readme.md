# Python Wrapper for FRAMOS D435e camera

This readme file provides instructions on how to use the D435e camera with Python.

## Supported platforms

Ubuntu 16.04 x86_64

Windows x86_64

## Prerequisites

FRAMOS CameraSuite version 4.1.2.0 or higher

Intel® RealSense™ SDK with support for D435e camera version 2.29.1 or higher

FRAMOS D435e Camera with firmware version 1.4.0.0 or higher

## Linux

Python3 wrapper for Intel® RealSense™ SDK with support for D435e camera is 
automatically installed as a part of the FRAMOS package.

Before launching one of the examples, make sure that required components are installed:

1. Install Python development packages
   
   ```
   sudo apt install python3 python3-dev
   ```

2. Update PYTHONPATH to add the path to the pyrealsense library
   
    ```
    export PYTHONPATH=$PYTHONPATH:/usr/lib
    ```

3. Install Python3 OpenCV package, required by some of the examples

    ```
    pip3 install --user opencv-python
    ```

4. Launch the examples

    ```
    cd /usr/src/librealsense2/wrappers/python/examples
    python3 align-depth2color.py
    ```

## Windows

Follow the official instructions.