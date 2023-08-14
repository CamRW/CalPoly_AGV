# California Polytechnic University, Pomona - Autonomous Vehicles Lab
# Advisor : Dr. Behnam Bahr

## Raspberry Pi
A raspberry pi on board the vehicle is used to receive driving commands in the form of ROS messages and communicate with the necessary microcontrollers to perform those commands. The same node also publishes encoder values to be used in for odometry.

## Overview
A docker container built on top of the ROS Humble docker container is used. The approprate volumes are mounted as well as network permissions and /dev entries. A middleware profile must be created to add the raspberry pi to the VPN.

## ROS2 Packages
Two packages are currently being used on the raspberry pi. 

### motor_control_pkg
The motor_control_pkg subscribes to the /twist_mux/cmd_vel topic to then send commands to the Arduino Mega microcontroller to control the motors. Additionally, this node publishes odometry information on /odom_encoder from the encoder data sent from the microcontroller.

### camera_publishers
This package publishes two Image topics /video/front_camera and /video/back_camera. These cameras were originally accessed through the Nvidia AGX Orin but there were issues with having 5 camera devices connected over USB.