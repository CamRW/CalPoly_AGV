# California Polytechnic University, Pomona - Autonomous Vehicles Lab
# Advisor : Dr. Behnam Bahr

## Purpose of this document
This document serves as an introduction to the autonomous vehicles lab's codebase. This lab's main purpose is to provide an extensible system for students to research various topics in autonomous systems.

## Overview
This lab uses the ROS2 software architecture to manage all aspects of the autonomous vehicle. ROS2 Humble is used in this project with the default FastDDS middleware. Since ROS2 is the backbone of this project, the file structure will resemble that of a ROS2 workspace. Individual packages will contain their own readme when applicable.

The code developed in this repository is set up to run on an Nvidia AGX Orin single board computer with various embedded systems for low-level control. The initial set up instructions for the AGX Orin will need to be followed when using a brand new AGX Orin (see https://developer.nvidia.com/embedded/learn/get-started-jetson-agx-orin-devkit). Device-specific settings such as network interfaces, ROS middleware profiles, serial ports, etc... will need configured based on your set-up. These changes will be outlined for each module.

This project utilizes a base docker image created through Nvidia's Isaac ROS packages. Setting up this image properly is critical to having a functional system. Follow Nvidia's documentation at https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common and https://github.com/NVIDIA-ISAAC-ROS

# VPN Setup
This project uses the Husarnet VPN as it is a simple to use VPN specifically designed to work with ROS. Follow the documentation to set up networks and clients on Husarnet's website (see https://husarnet.com/ and https://github.com/husarnet/husarnet). Additionally, a tool provided by Husarnet, called husarnet-dds (see https://github.com/husarnet/husarnet-dds), can be used to automatically create the middleware profile necessary for ROS to communicate outside of the local network.

# Racing Simulator Chair Setup
One unique aspect of this lab is the integration of our system with a racing simulator chair. From the racing simulator, we can remotely control the vehicle, and with IMUs on the vehicle, we can send feedback to the chair to convey what a driver would experience. ROS nodes were developed for the chair to communicate with the vehicle remotely. Currently, the implementation of the haptic feedback (i.e. the chair shaking when the vehicle hits a bump) module is not ROS compatible. The haptic feedback module uses a simple websocket developed before ROS became the standard architecture. Future work should focus on integrating the haptic feedback module into the rest of the system. 

# Working with Submodules
Third-party packages are added as git submodules, some packages require minor changes to configurations to operate within the system. Future versions could incorporate these changes with forked versions of the submodules used. Each change required will be outlined.

## Current List of Submodules
The submodules used in this project are listed below, if a specific branch is required, it will be listed. Otherwise assume the default branch is used.

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

# Custom ROS Nodes
Many features for this lab are required to be implemented from scratch due to the uniqueness of our system. This section will outline the custom nodes developed, and a brief introduction to their functionality.

## agv_bot_description
This package is an ongoing effort in creating an accurate URDF model of the vehicle in order to properly simulate and visualize the vehicle in software such as RVIZ and Gazebo. Additionally, this package provides the transformations from sensor frames to the base frame of the vehicle which are necessary for tasks like SLAM.

## gps_package
This package communicates with the on-board Sierra Wireless Airlink MP70 (https://www.sierrawireless.com/router-solutions/mp70/) wireless modem to aquire GPS data. The node opens a serial port to the modem and receives telemetry data in the TAIP protocol format. This data is parsed and then published on a NavSatFix topic. Frequency of the messages can be changed within the modem's configuration page.

## lin_det_package
Currently in development, the final version of this package aims to provide some basic autonomy via line following.

## realsense_obj_det
This package uses the on-board realsense camera and the topics published from the realsense-ros nodes to detect objects close to the vehicle. By utilizing the depth sensor on the realsense camera, objects of a certain size within a "close" distance to the camera will change the value of the topic published from this node. The controller node is subscribed to this topic and will stop the drive motors when conditions are met.

## simulator_control
This is the current implementation of steering and velocity control. This node subscribes to twist messages and uses the x.linear field for linear acceleration and the z.angular field for steering angle. Two microcontrollers are used for the low-level motor control. This node opens up serial ports to these microcontrollers and send the appropriate message based on the twist messages.