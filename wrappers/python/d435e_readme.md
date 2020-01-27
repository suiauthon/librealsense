# Python Wrapper for FRAMOS D435e camera

This readme file provides instructions on how to use the D435e camera with Python.

## Supported platforms

Ubuntu 16.04 x86_64

Windows x86_64

## Prerequisites

FRAMOS CameraSuite version 4.1.2.0 or higher

Intel® RealSense™ SDK with support for D435e camera version 2.29.1 or higher

FRAMOS D435e Camera with firmware version 1.4.0.0 or higher

## Notes

The `pyrealsense2` wrapper that is part of this package supports the D435e camera.

The `pyrealsense2` wrapper available from `pip` does not support the D435e camera.

The `pybackend2` wrapper that is part of this package does not support the D435e camerea.

## Linux

Follow the official instructions to build and install the `pyrealsense` wrapper.

## Windows

Follow the official instructions to build the `pyrealsense` wrapper.

The wrapper must be copied next to the Python script along with required dependencies.

Copy the Python wrapper (name may be include additional information)

- `pyrealsense2.pyd`

Copy the RealSense library with suport for D435e camera from `Program Files\FRAMOS-librealsense2\bin`
- `realsense2.dll`

Copy the contents of `Program Files\FRAMOS\CameraSuite\bin`

- `CameraSuite.dll`

Copy the contents of `Program Files\FRAMOS\CameraSuite\GenICam_v3_0\bin\Win64_x64`

- `GCBase_MD_VC120_v3_0.dll`
- `GenApi_MD_VC120_v3_0.dll`
- `Log_MD_VC120_v3_0.dll`
- `MathParser_MD_VC120_v3_0.dll`
- `NodeMapData_MD_VC120_v3_0.dll`
- `XmlParser_MD_VC120_v3_0.dll`

Install Python3 OpenCV package, required by some examples

```
pip3 install --user opencv-python
```

