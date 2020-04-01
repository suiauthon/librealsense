# Getting Started with FRAMOS RealSense™ SDK2.0 for LabVIEW®

> **Note**:
>This document refers to RealSense™ SDK2.0 for LabVIEW® readme file found on https://github.com/IntelRealSense/librealsense/tree/master/wrappers/labview

## Introduction:
The latest generation of RealSense Depth Cameras can be controlled via the RealSense SDK2.0, also referred to as libRealSense 2.x.x, or LibRS for short. This includes stereo depth cameras D415 and D435 as well as their corresponding modules, such as D410 and D430. FRAMOS-librealsense2 extends the standard librealsense2 API with features applicable to the D400e cameras.


To get started controlling the RealSense Cameras with LabVIEW® in Windows 10, we have created a [VI-library](http://realsense-hw-public.s3.amazonaws.com/Releases/RS4xx/Windows/labview_2_8_1.zip) that wraps most of the core functions of the realsense2.dll, and we have also created some simple “Hello World” VI’s to get started capturing Color Depth Maps. This uses LabVIEW 2016 and requires a Windows 10 laptop with a USB3 port.   


## Getting Started:
Source files are located in FRAMOS-libreasesnse2 install folder inside of *wrappers* subfolder. It is neccessary to copy realsense2.dll file from *bin*  to folder with source files. There are three simple demos to help you get started with more to follow.




### RealSense Hello World: Monochrome + Depth

To run it, plug in a RealSense D415(e) or D435(e) depth camera, and press the run arrow. It should start streaming monochrome and depth. The monochrome is from the left camera of the stereo pair. 


You can stop the app and change the resolution in the pull-down menu and try different resolutions. However please note that for Stereo Depth Cameras the depth resolution (i.e. the Z-axis or ranging resolution) scales with resolution, so if you chose a smaller resolution expect not just smaller X,Y resolution, but also more error in the depth error.  We do note also that the minimum range, aka MinZ also scales with resolution. So while a D415 running at 1280x720 resolution will have a minZ of about 44cm, reducing the resolution to 640x360 will reduce the minZ to 22cm.   


In this app you can also turn the projector on/off. While the stereo depth cameras work great on naturally textured surfaces, objects that have only a single color (ex. A white wall) with no texture will not be resolved very well in depth range if the laser projector is not turned on.
   


### RealSense Hello World: Left-Right
You can also access directly the left and right channels of the stereo camera. We should note that only certain combinations are allowed, such as Color+Depth or Monochrome L+R. The monochrome images in this demo are 8 bit grey-scale and are after rectification by the RealSense ASIC. This is the input that will go to the stereo calculation engine inside the ASIC. It is therefore perfectly calibrated to the depth.

![Color + Depth](https://raw.githubusercontent.com/wiki/IntelRealSense/librealsense/res/labview/2.png)

### RealSense Hello World: Third Color Camera
This demos shows how to access external (rgb) sensor and get color stream. 

## Understanding the Programming:
While all the VI’s do have embedded documentation, it should in principle all be “self-explanatory”. It is however not completely straight forward and it requires some explanation about the LibRealSense (LibRS) architecture and in particular in what sequence the commands need to be executed, as well as what parameters are valid.

First we look at the hierarchy of devices, sensors, and streams. For further information about LibRS architecture, please refer to [API architecture](https://github.com/IntelRealSense/librealsense/blob/master/doc/api_arch.md)


![Color + Depth](https://raw.githubusercontent.com/wiki/IntelRealSense/librealsense/res/labview/6.png)

* LibRS supports being able to connect multiple <font color="orange">**DEVICES**</font>.  For example, multiple RealSense Cameras can be connected at once.
* Each DEVICE can have multiple <font color="green">**SENSORS**</font>. For example, the D415 has a Stereo Sensor and an extra RGB Sensor. Other devices may also have Fisheye sensors or IMU sensors.
* Each SENSOR can in turn have multiple <font color="blue">**STREAMS**</font>. These are, for example, the depth and color and monochrome streams. Each STREAM will be described by many different traits, such as frame rate, resolution, format, etc. Most importantly though is that each stream can be uniquely identified by a PROFILE number.  A single sensor can have hundreds of streams.


When initializing and preparing to stream specific STREAMs, it is necessary to find their specific PROFILE numbers.  So this means that the best way to start a RealSense Device to stream data, is to enumerate ALL device, sensors, and streams, and search for the one that has the features you are looking for, and remembering its PROFILE number. An array of these profiles numbers is input into the Configure VI, and the stream is officially started with the Start VI.


So now you are streaming data. To grab data you call the Capture VI inside a loop. This will give you a Frame pointer each time it is called. The VI will also tell you the type of frame captured, ie. What stream it belongs to. You can use the Get Frame Data to get a pointer to the payload.  So if it is a “depth frame” we will treat the data as depth data (in this case 16bit data), and if it is a “color frame” we will treat it as color (in the case R, G, B), or as a Left Monochrome, Right monochrome etc.    


Note that frames can arrive asynchronously. You can query their time stamp or counters to align them, or just grab the nearest frames in time.     


Finally, it is VERY important that you use the Release Frame VI to release each frame when it is no longer needed, or you will very quickly stop streaming. Also, be sure to CLOSE everything properly, or you will have problems streaming the next time, and LabVIEW may crash. 


-------------------


> **Note**: If you are looking for a different way to modify DLL file location, please refer to [#1947](https://github.com/IntelRealSense/librealsense/issues/1947) and consider trying   [github.com/ryannazaretian/librealsense/tree/master/wrappers/labview](https://github.com/ryannazaretian/librealsense/tree/master/wrappers/labview) as a possible solution. 
