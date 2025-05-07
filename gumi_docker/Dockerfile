FROM nvidia/cuda:11.8.0-base-ubuntu20.04
ARG ROS_PKG=ros_base
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

# # nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

SHELL ["/bin/bash", "-c"]

RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
	python3.8 python3-pip \
	curl wget zip unzip tar git cmake make build-essential \
	gnupg2 \
	lsb-release \
	ca-certificates \
	ffmpeg \
    && rm -rf /var/lib/apt/lists/*

####################################################################################################
######################################### ROS INSTALLATION #########################################
####################################################################################################

# Add the ROS repository and the ROS key
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-${ROS_DISTRO}.list' \
     && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN apt-get update && apt-get install -y python3-rosdep \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Setup the environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install additional dependencies for building ROS packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
	ros-${ROS_DISTRO}-catkin \
	python3-catkin-tools \
    build-essential \
	ros-${ROS_DISTRO}-ros-numpy \
    && rm -rf /var/lib/apt/lists/*

####################################################################################################
###################################### ABB ROBOT DRIVER ############################################
####################################################################################################

WORKDIR /opt
# install poco
RUN git clone -b poco-1.13.3-release https://github.com/pocoproject/poco.git
RUN cd poco && mkdir cmake-build && cd cmake-build && cmake .. && cmake --build .

# install boost c++ library and Google Protocol Buffers
RUN apt-get update && \
	DEBIAN_FRONTEND=noninteractive apt-get install -y \
    libboost-all-dev \
    libprotobuf-dev protobuf-compiler \
	&& rm -rf /var/lib/apt/lists/*

####################################################################################################
###################################### DYNAMIXEL DRIVER ############################################
####################################################################################################
WORKDIR /opt
RUN git clone -b 3.8.1 --single-branch https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN python3 -m pip install -e DynamixelSDK/python

####################################################################################################
##################################### ROS PACKAGES INSTALLATION ####################################
####################################################################################################
RUN apt-get update && \
	DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-catkin-tools \
	ros-${ROS_DISTRO}-control-toolbox \
	ros-${ROS_DISTRO}-controller-interface \
	ros-${ROS_DISTRO}-controller-manager \
	ros-${ROS_DISTRO}-joint-limits-interface \
	ros-${ROS_DISTRO}-joint-state-publisher \
	ros-${ROS_DISTRO}-joint-state-controller \
	ros-${ROS_DISTRO}-joint-trajectory-controller \
	ros-${ROS_DISTRO}-position-controllers \
	ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
	ros-${ROS_DISTRO}-velocity-controllers \
	ros-${ROS_DISTRO}-simple-message \
	ros-${ROS_DISTRO}-robot-state-publisher \ 
	ros-${ROS_DISTRO}-tf-conversions \
	ros-${ROS_DISTRO}-hector-xacro-tools \
	ros-${ROS_DISTRO}-ros-numpy && \
    rm -rf /var/lib/apt/lists/*

####################################################################################################
######################################### PIP PACKAGES #############################################
####################################################################################################
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install -r requirements.txt --default-timeout=1000 --no-cache-dir
RUN python3 -m pip install --upgrade pip

####################################################################################################
########################################### FINALISATION ###########################################
####################################################################################################

ENTRYPOINT [ "" ]

WORKDIR /gumi_docker
CMD /bin/bash

