# California Polytechnic University, Pomona - Autonomous Vehicles Lab
# Advisor : Dr. Behnam Bahr

## Raspberry Pi
A raspberry pi on board the vehicle is used to receive driving commands in the form of ROS messages and communicate with the necessary microcontrollers to perform those commands. The same node also publishes encoder values to be used in for odometry.

## Overview
A docker container built on top of the ROS Humble docker container is used. The approprate volumes are mounted as well as network permissions and /dev entries. A middleware profile must be created to add the raspberry pi to the VPN.

## ROS2 Packages
Currently a single ROS2 package, called encoder_pkg receives driving commands from the /twist_mux/cmd_vel topic and sends the correct message to the microcontroller for controlling velocity and steering. This same package also publishes encoder values read from the microcontroller which can be used for odometry.