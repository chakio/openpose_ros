# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04
ARG USER=initial
ARG GROUP=initial
ARG UID=1000
ARG GID=${UID}
ARG SHELL=/bin/bash
ARG workspace=/
#FROM $base_image
RUN echo base image: ${base_image}

#######################################################################
##                            Speeding up                            ##
#######################################################################
RUN sed -i 's@archive.ubuntu.com@ftp.jaist.ac.jp/pub/Linux@g' /etc/apt/sources.list

#######################################################################
##                          openpose install                         ##
#######################################################################
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
   pkg-config \
   apt-utils \
   wget \
   git \
   build-essential \ 
   net-tools \
   gedit \
   terminator \
   nautilus \
   software-properties-common \
   apt-transport-https \
   libopencv-dev \
   ffmpeg \
   x264 \
   libx264-dev \
   zip \
   unzip \
   usbutils \
   sudo \
   python3-pip \
   libusb-1.0-0-dev

#######################################################################
##                           install font                            ##
#######################################################################
RUN echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections 
RUN apt-get install -y ttf-mscorefonts-installer \
   msttcorefonts -qq

RUN python3 -m pip install --upgrade pip
RUN apt-get install  -y python3-ruamel.yaml

#######################################################################
##                       install nvidia docker                       ##
#######################################################################
RUN apt-get install -y --no-install-recommends \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libxext-dev \
    libx11-dev \
    mesa-utils \
    x11-apps

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

#######################################################################
##                            install ros                            ##
#######################################################################

RUN apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    curl


# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
# setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update 
RUN apt install -y \
 ros-noetic-desktop-full
 # install ros packages
ENV ROS_DISTRO noetic
COPY ./ros_entrypoint.sh /

RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \ 
    python3-rosinstall-generator \ 
    python3-wstool \
    build-essential 
#######################################################################
##                   install additional packages                     ##
#######################################################################

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

#######################################################################
##                         install openpose                          ##
#######################################################################

RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose -b v1.7.0
RUN cd openpose/ && \
    git submodule update --init --recursive --remote

COPY ./CMakeLists.txt /openpose

RUN apt-get install -y \
    cmake-qt-gui \
    libssl-dev \
    qtbase5-dev \ 
    libopencv-dev \
    libeigen3-dev

RUN cd openpose/ && \
    bash ./scripts/ubuntu/install_deps.sh

RUN cd openpose/ && \
    mkdir build && \
    cd build && \
    cmake ..

RUN cd openpose/build && \
    cmake .. -LA

RUN cd openpose/build && \
    make -j`nproc`

RUN rm -rf ~/.nv/

RUN apt-get install -y \
    ros-noetic-ros-numpy

#######################################################################
##                            delete cash                            ##
#######################################################################
RUN rm -rf /var/lib/apt/lists/*

#######################################################################
##                         make sudoer user                          ##
#######################################################################

ARG USER
ARG USER_ID
 
RUN groupadd -g 1000 developer && \
    useradd  -g      developer -G sudo -m -u $USER_ID -s /bin/bash ${USER} && \
    echo "${USER}:${USER}" | chpasswd

RUN echo "Defaults visiblepw" >> /etc/sudoers
RUN echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

#######################################################################
##                    make user accessible to usb                    ##
#######################################################################
RUN adduser ${USER} dialout


# USER root
USER ${USER}

RUN echo "export PS1='\[\e[1;31;40m\]OPENPOSE_ROS\[\e[0m\] \u:\w\$ '">> ~/.bashrc
RUN echo "source /ros_setting.sh">> ~/.bashrc

WORKDIR ${workspace}