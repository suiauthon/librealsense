# ROS Wrapper for FRAMOS D435e camera

This readme file provides instructions on how to use the D435e camera with ROS.

## Supported platforms

Ubuntu 16.04 x86_64

## Prerequisites

FRAMOS CameraSuite version 4.1.2.0 or higher

Intel® RealSense™ SDK with support for D435e camera version 2.29.1 or higher

FRAMOS D435e Camera with firmware version 1.4.0.0 or higher

## Notes

These instructions are based on the instructions available at the [ROS Wrapper for Intel® RealSense™ Devices GitHub repository](https://github.com/IntelRealSense/realsense-ros).

Intel® RealSense™ SDK with support for D435e camera is compatible with the ROS wrapper version 2.2.8.

## Installation

### Prerequisites

Make sure that `main`, `universe`, `restricted` and `multiverse` repositories are added

```
sudo add-apt-repository main
sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse
sudo apt update
```

### Install the ROS distribution

Install ROS Kinetic. See the [ROS wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu) for details.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
```
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full -y
```
```
sudo rosdep init
rosdep update
```
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
```
sudo apt-get install ros-kinetic-ddynamic-reconfigure -y
```

### Install Intel® RealSense™ ROS from Sources

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
```

```
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout 2.2.8
cd ..
```

```
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage Instructions

Before starting the camera node, make sure to modify the launch file for the D435e camera:

- set the `enable_infra1` argument to `false`
- set the `enable_infra2` argument to `false`
- set the `enable_pointcloud` argument to `true`

The launch file used in this example is `rs_camera.launch` located in `~/catkin_ws/src/realsense-ros/realsense2_camera/launch`

Start the camera node in ROS

```
roslaunch realsense2_camera rs_camera.launch
```

Launch `rviz` in another terminal

```
rosrun rviz rviz
```

In the `rviz` GUI 
- change the `Fixed Frame` from `map` to `camera_depth_frame`
- click `Add`, select `By topic` and choose `depth/color/points/PointCloud2`