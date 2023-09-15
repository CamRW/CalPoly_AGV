# California Polytechnic University, Pomona - Autonomous Vehicles Lab
# Advisor : Dr. Behnam Bahr

## Purpose of this document
This document serves as an introduction to the autonomous vehicles lab's codebase. This lab's main purpose is to provide an extensible system for students to research various topics in autonomous systems.

## Overview
This lab uses the ROS2 software architecture to manage all aspects of the autonomous vehicle. ROS2 Humble is used in this project with the default FastDDS middleware. Since ROS2 is the backbone of this project, the file structure will resemble that of a ROS2 workspace. Individual packages will contain their own readme when applicable.

There are currently three computing devices on the vehicle. The main computing device is an Nvidia AGX Orin, which handles high level robotics tasks and several sensor data collection modules. A raspberry pi board is used for communication with a microcontroller (Arduino Mega)
for sending control commands and recieving odometry data. Additionally, the raspberry pi interfaces with two usb cameras that are used for data collection and video streaming for remote control. The microcontroller currently used is an Arduino Mega which recieves control commands, collects encoder data, and transmits odometry information to the rest of the system.

The structure of this repository is set up so that each directory in the root of the repository corresponds to the code running on a specific device. A more detailed project structure is discussed below.

## Project Structure

### isaac_ros-dev
The code developed in this directory is set up to run on an Nvidia AGX Orin single board computer with various embedded systems for low-level control. The initial set up instructions for the AGX Orin will need to be followed when using a brand new AGX Orin (see https://developer.nvidia.com/embedded/learn/get-started-jetson-agx-orin-devkit). Device-specific settings such as network interfaces, ROS middleware profiles, serial ports, etc... will need configured based on your set-up. These changes will be outlined for each module.

This project utilizes a base docker image created through Nvidia's Isaac ROS packages. Setting up this image properly is critical to having a functional system. Follow Nvidia's documentation at https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common and https://github.com/NVIDIA-ISAAC-ROS

The bulk of this directory is laid out in standard ROS2 folder structures, the workspace directory isaac_ros-dev stores the source code for the ROS nodes in its src directory. A root level launch folder is used for storing a vehicle bringup launch file. A configs folder is used for storing camera configs that may be used across different nodes. Also within the root of the workspace, a utils folder is used for utility scripts such as updating camera device numbers for multiple identical cameras and a test script for trying out our road segmentation model using opencv. A high level overview of the project structure is listed below.

```
isaac_ros-dev
.
├── configs
│   └── camera_configs
├── launch
│   └── bringup_launch.py
├── src
│   ├── agv_bot_description
│   ├── async_web_server_cpp
│   ├── gps_package
│   ├── image_stitcher
│   ├── isaac_ros_common
│   ├── isaac_ros_nvblox
│   ├── realsense_obj_det
│   ├── realsense-ros
│   ├── teleop_twist_joy
│   ├── teleop_twist_keyboard
│   ├── twist_mux
│   ├── velodyne
│   └── web_video_server
└── utils
```
### Pi
The raspberry pi OS is 20.04 Ubuntu Server, but all modules are developed within a ROS2 Humble docker container. A dockerfile is provided to build the image and a start script is written to start the container. The workspace is mounted as a volume to allow development within the container.

```

workspaces
.
├── ros2_humble_ws 
|	├── launch
│   	└── bringup.py
|	├── src
│   	├── encoder_pkg
│   	└── camera_publishers
├── Dockerfile
└── start.sh
```


### Microcontrollers
One microcontroller is currently being used to handle control commands and transmit odometry information. The code was written for an Arduino Mega but should be easily adaptable to most boards. No directory structure is provided as it is a single .ino file.
# VPN Setup
This project uses the Husarnet VPN as it is a simple to use VPN specifically designed to work with ROS. Follow the documentation to set up networks and clients on Husarnet's website (see https://husarnet.com/ and https://github.com/husarnet/husarnet). Additionally, a tool provided by Husarnet, called husarnet-dds (see https://github.com/husarnet/husarnet-dds), can be used to automatically create the middleware profile necessary for ROS to communicate outside of the local network.

# Racing Simulator Chair Setup
One unique aspect of this lab is the integration of our system with a racing simulator chair. From the racing simulator, we can remotely control the vehicle, and with IMUs on the vehicle, we can send feedback to the chair to convey what a driver would experience. ROS nodes were developed for the chair to communicate with the vehicle remotely.

# Working with 3rd Party Libraries
Third-party packages are cloned and modified to fit our needs, some packages require minor changes to configurations to operate within the system. Each change required will be outlined.

## Current List of 3rd Party Libraries
The 3rd party libraries used in this project are listed below, if a specific branch is required, it will be listed. Otherwise assume the default branch is used.

- velodyne
	- https://github.com/ros-drivers/velodyne
	- branch: ros2
- twist_mux
	- https://github.com/ros-teleop/twist_mux
- web_video_server
	- https://github.com/RobotWebTools/web_video_server
	- branch: ros2
- async_web_server_cpp
	- https://github.com/fkie/async_web_server_cpp
- realsense-ros
	- https://github.com/IntelRealSense/realsense-ros
- isaac_ros_common
	- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
- teleop_twist_joy
	- https://github.com/ros2/teleop_twist_joy
	- branch: humble

## Velodyne
This package serves as the driver for accessing the Velodyne VLP16 puck lidar used on this vehicle. It publishes several ROS2 topics used for point cloud processing. The package assumes the default network address for the lidar is used. If there is a desire to change that address, the change must be reflected in this package.

## Twist_Mux
Twist_mux adds utilities to the twist messages used to control the vehicle's velocity and steering angle. It adds priority functionality so that things like remote control will supercede velocity commands from a controller working in autonomous mode. Topic names as well as their priority need to be adjusted to fit your needs.

## Web_video_server
This package is used to host a simple web video server to stream ROS2 image topics for remote control of the vehicle. The change necessary for this package is the IP address of the device must be specified. This can be accomplished via a ROS address parameter passed on startup.

## Async_web_server_cpp
Async web server is used in conjuction with the web_video_server package. No changes are necessary to this package.

## Realsense-ros
This package creates the functionality to add Intel Realsense devices to the ROS architecture. Currently, the only changes implemented are setting parameters in the launch script to publish IMU messages, as they are not set in the default launch file.

## Isaac_ros_common
Due to limitations in Nvidia's L4T OS, it's necessary to use a docker image to take advantage of many packages built for ROS. The isaac_ros_common package contains several dockerfiles and build scripts for building the base image used in this project. Middleware profiles are defined for communicating remotely with other devices within the VPN, as well as a custom dockerfile is created to add dependencies for libraries used throughout the system.

## Teleop_twist_joy
This package allows control of the vehicle via keyboard or controller commands, changes are necessary to the configuration files for certain controller set ups. These changes need to match the velocity command topics that are subscribed to by the controller/twist_mux.

# Custom ROS Packages
Many features for this lab are required to be implemented from scratch due to the uniqueness of our system. This section will outline the custom nodes developed, and a brief introduction to their functionality.

## agv_bot_description
This package is an ongoing effort in creating an accurate URDF model of the vehicle in order to properly simulate and visualize the vehicle in software such as RVIZ and Gazebo. Additionally, this package provides the transformations from sensor frames to the base frame of the vehicle which are necessary for tasks like SLAM.

## gps_package
This package communicates with the on-board Sierra Wireless Airlink MP70 (https://www.sierrawireless.com/router-solutions/mp70/) wireless modem to aquire GPS data. The node opens a serial port to the modem and receives telemetry data in the TAIP protocol format. This data is parsed and then published on a NavSatFix topic. Frequency of the messages can be changed within the modem's configuration page.

## image_stitcher
This package stitches multiple camera streams together to provide a 360 degree view around the vehicle for remote control. Since multiple identical cameras are used their /dev entries can change on boot and they cannot be symlinked through udev rules due to their identical properties, a helper script get_video_devices.py in isaac_ros-dev/utils will query the /dev entries based on usb bus and port and then write to a json file in isaac_ros-dev/configs/camera_configs/cameras.json which is used for parsing the camera /dev entries. The limitation to this fix is that the cameras must be plugged into their unique usb ports since the only way to differentiate identical devices is on which usb port/bus they are connected to.

## realsense_obj_det
This package uses the on-board realsense camera and the topics published from the realsense-ros nodes to detect objects close to the vehicle. By utilizing the depth sensor on the realsense camera, objects of a certain size within a "close" distance to the camera will change the value of the topic published from this node. The controller node is subscribed to this topic and will stop the drive motors when conditions are met.

## road_segmentation
Road segmentation uses the UNet architecture (https://arxiv.org/abs/1505.04597) trained on labeled images of roads provided by (https://github.com/aatiibutt/Drivable-Road-Region-Detection-and-Steering-Angle-Estimation-Method) as a baseline. The model is then fine-tuned with labeled images collected around Cal Poly Pomona's campus. This node subscribes to an optical camera topic /video/front_camera by default, and publishes an image masked with the detected road segment as well as a twist message to maintain a center position on the detected road segment.

## camera_publishers
This package runs on the raspberry pi and publishes two image topics for the optical front and back cameras. It uses OpenCV to access the devices, resize the images, and then publishes them on their appropriate ROS2 topics.

## motor_control_pkg
This package handles the motor controls and their associated encoders. This package also runs on the raspberry pi and communicates with an Arduino Mega over serial to send control commands and recieve odometry information.
