# D400e librealsense2 API extensions

This readme file provides an overview of extensions to the librealsense2 API implemented to support D400e cameras.

## Heartbeat time

//TODO explain heartbat time

//TODO global, seconds, 4x, during stream

//C

//C++

//Python

## Controls

## Camera information

Available camera information in the librealsense2 API is listed in the `rs2_camera_info`  enumeration available in the `librealsense2/h/rs_sensor.h` header file. This enumeration was extended to provide information specific to the D400e cameras.

The `RS2_CAMERA_INFO_IP_ADDRESS` enumerator represents the IP address of a D400e camera.

The `RS2_CAMERA_INFO_SUBNET_MASK` enumerator represents the subnet mask of a D400e camera.

The `RS2_CAMERA_INFO_DEVICE_VERSION` enumerator represents FRAMOS firmware version on a D400e camera. This value is different from the `RS2_CAMERA_INFO_FIRMWARE_VERSION` enumerator which represents the Intel D4 firmware version.

Same API calls are used to obtain information from both extended and ordinary enumerators.

C++ example:

```c++
rs2::device device;
//obtain rs2::device from rs2::pipeline or rs2::context
const char* device_info = device.get_info(RS2_CAMERA_INFO_DEVICE_VERSION);
```

C example:

```c
rs2_device* dev;
rs2_error* e = 0;
const char* device_info;
//obtain rs2_device* from rs2_context*
device_info = rs2_get_device_info(device, RS2_CAMERA_INFO_DEVICE_VERSION, e);
```

Python example