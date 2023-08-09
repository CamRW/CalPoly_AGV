FROM arm64v8/ros:humble-ros-base

RUN apt-get update && apt-get install -y \
	net-tools \
	iputils-ping \
	python3-pip \
	vim

RUN pip install \
	pyserial
	

COPY . /workspaces

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/ros2_humble_ws/middleware_profile/husarnet-fastdds.xml

#ENTRYPOINT ["/workspaces/ros_entrypoint.sh"]

#CMD ["bash"]
