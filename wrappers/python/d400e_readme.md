# Python Wrapper for FRAMOS D400e camera series

This readme file provides instructions on how to use the D400e camera series with Python.

## Supported platforms

Ubuntu 16.04 x86_64

Windows x86_64

## Prerequisites

FRAMOS CameraSuite version 4.1.2.0 or higher

Intel® RealSense™ SDK with support for D400e cameras version 2.29.1 or higher

## Notes

The `pyrealsense2` wrapper that is part of this package supports the D400e camera series.

The `pyrealsense2` wrapper available from `pip` does not support the D400e camera series.

The `pybackend2` wrapper that is part of this package does not support the D400e camera series.

## Linux

Install required packages:
```
sudo apt install gcc g++ cmake libglfw3-dev libgtk-3-dev git libssl-dev libusb-1.0-0-dev pkg-config python3 python3-dev
```

Additional packages are required on Ubuntu 18:
```
sudo apt install libgl1-mesa-dev libglu1-mesa-dev
```

Install Python packages required by some of the Python examples:
```
pip3 install --user opencv-python
```

Copy the sources to a folder with user permissions, for example home:
```
cp -r /usr/src/librealsense2 ~
```

Create a build folder:
```
cd librealsense2
mkdir build
cd build
```

Generate Makefiles with cmake:
```
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
```

Build with make:
```
make -j$(nproc)
```

Copy the pyrealsense library next to the example and run it:
```
cd ~/librealsense2/wrappers/python/examples
cp ~/librealsense2/build/wrappers/python/pyrealsense2* .
python3 align-depth2color.py
```

## Windows

Follow the official instructions to build the `pyrealsense` wrapper.

The wrapper must be copied next to the Python script along with required dependencies.

Copy the Python wrapper (name may be include additional information)

- `pyrealsense2.pyd`

Copy the RealSense library with support for D400e camera series from `Program Files\FRAMOS-librealsense2\bin`

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

