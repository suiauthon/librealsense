# D400e librealsense2 API extensions

This readme file provides an overview of extensions to the librealsense2 API implemented to support D400e cameras.

## Heartbeat time

//TODO explain heartbat time

//TODO global, seconds, 4x, during stream

//C

//C++

//Python

## Sensor Options

Available sensor options in the librealsense2 API are listed in the `rs2_option` enumeration available in the `librealsense2/h/rs_option.h` header file. This enumeration was extended to provide options specific to D400e cameras.

The `RS2_OPTION_INTER_PACKET_DELAY` enumerator represents the delay in microseconds between packets that the camera sends to the host. The library automatically detects optimal value for this option on initialization.

The `RS2_OPTION_PACKET_SIZE` enumerator represents the size of Ethernet packets in bytes that the camera uses to stream images. The library automatically detects optimal value for this option on initialization. This option cannot be set while the sensor is streaming.

Same API calls are used to set both normal and extended options.

C++

```c++
rs2::sensor sensor; //obtain rs2::sensor from rs2::device or rs2::context
sensor.set_option(RS2_OPTION_INTER_PACKET_DELAY, 65.f);
```

C

```c
rs2_sensor* sensor; //obtain rs2_sensor* using rs2_create_sensor()
rs2_error* e = 0;
rs2_set_option(sensor, RS2_OPTION_INTER_PACKET_DELAY, 65.f, e);
```

Python

```python
#TODO
```



## Camera information

Available camera information in the librealsense2 API is listed in the `rs2_camera_info`  enumeration available in the `librealsense2/h/rs_sensor.h` header file. This enumeration was extended to provide information specific to D400e cameras.

The `RS2_CAMERA_INFO_DEVICE_VERSION` enumerator represents FRAMOS firmware version on a D400e camera. This value is different from the `RS2_CAMERA_INFO_FIRMWARE_VERSION` enumerator which represents the Intel D4 firmware version.

The `RS2_CAMERA_INFO_IP_ADDRESS` enumerator represents the IP address of a D400e camera.

The `RS2_CAMERA_INFO_SUBNET_MASK` enumerator represents the subnet mask of a D400e camera.

Same API calls are used to obtain information from both normal and extended enumerators.

C++

```c++
rs2::device device; //obtain rs2::device from rs2::context or rs2::pipeline_profile
const char* device_info = device.get_info(RS2_CAMERA_INFO_DEVICE_VERSION);
```

C

```c
rs2_device* dev; //obtain rs2_device* using rs2_create_device()
rs2_error* e = 0;
const char* device_info;
device_info = rs2_get_device_info(device, RS2_CAMERA_INFO_DEVICE_VERSION, e);
```

Python

```python
#TODO
```

