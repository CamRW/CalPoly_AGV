FROM arm64v8/ros:humble-ros-base

RUN apt-get update && apt-get install -y \
	net-tools \
	iputils-ping \
	python3-pip \
	vim \
	ffmpeg \
	libsm6 \
	libxext6 \
	ros-humble-cv-bridge \
	python3-opencv

RUN pip install \
	pyserial \
	numpy

RUN pip install --upgrade numpy

WORKDIR /workspaces
# COPY ./ros2_humble_ws /workspaces/ros2_humble_ws

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/ros2_humble_ws/middleware_profile/husarnet-fastdds.xml

