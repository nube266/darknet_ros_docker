# Pin the versions of the core tools and packages using global arguments for improved stability.
ARG nvidia_cudagl_version=9.0-devel-ubuntu16.04
ARG nvidia_cudnn_version=7.1.3.16-1+cuda9.0

FROM nvidia/cudagl:${nvidia_cudagl_version}

# Install the Nvidia cuDNN library missing in the parent image.
# https://gitlab.com/nvidia/cuda/blob/ubuntu16.04/10.1/devel/cudnn7/Dockerfile
ARG nvidia_cudnn_version
ENV NVIDIA_CUDNN_VERSION=${nvidia_cudnn_version}
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=${NVIDIA_CUDNN_VERSION} \
    libcudnn7-dev=${NVIDIA_CUDNN_VERSION} \
    && apt-mark hold libcudnn7 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS Kinetic Kame.
# http://wiki.ros.org/kinetic/Installation/Ubuntu

# Update the package list.
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# Add the package keys.
RUN apt-key adv --keyserver 'hkp://ha.pool.sks-keyservers.net:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install 'ros-kinetic-desktop-full' packages (including ROS, Rqt, Rviz, and more).
#ARG ros_desktop_version
#ENV ROS_DESKTOP_VERSION=${ros_desktop_version}
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update

# Install common optional packages/tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    nano \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Download_weights
WORKDIR  /root/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/
RUN wget http://pjreddie.com/media/files/yolov3.weights

# Install other packages/library
RUN apt-get update && apt-get install -y --no-install-recommends \
    python-catkin-tools \
    libboost-dev \
    ros-kinetic-vision-opencv \
    python-opencv \
    libopencv-dev \
    dbus \
    && rm -rf /var/lib/apt/lists/*

# build
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc
RUN echo "cd /root/catkin_ws; catkin build" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc