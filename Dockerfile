FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04

# ROS kinetic core
RUN apt-key adv --keyserver 'hkp://ha.pool.sks-keyservers.net:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# ROS desktop full
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update

RUN mkdir -p /home/catkin_ws/src
RUN apt-get update && apt-get -y install build-essential \
    git wget nano v4l-utils vim

WORKDIR /home/catkin_ws/src
RUN git clone --recursive -b cuda8_0-dev https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeMiake/darknet_ros_docker.git
RUN git clone https://aisl-serv6.aisl.cs.tut.ac.jp:20443/YusukeMiake/usb_cam.git

RUN mkdir /home/catkin_ws/src/darknet_ros/weights
WORKDIR /home/catkin_ws/src/darknet_ros/weights
RUN wget http://pjreddie.com/media/files/yolov3.weights

WORKDIR /home/catkin_ws

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV LD_LIBRARY_PATH = $LD_LIBRARY_PATH:/usr/local/cuda/lib64
ENV ROS_DISTRO kinetic

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
