# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM nvidia/cuda:8.0-cudnn5-devel

#######################################################################
##                          openpose install                         ##
#######################################################################
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y \
    && apt-get install -y \
    build-essential \
    apt-utils \
    openssl \
    git \
    cmake \
    git \
    curl \
    vim \
    ca-certificates \
    libjpeg-dev \
    libfreetype6-dev \
    zlib1g-dev \
    openssh-client \
    bash \
    g++ \
    tzdata \
    mariadb-client \
    gcc \
    xz-utils \
    file \
    sudo \
    tmux \
    libmecab-dev \
    wget \
    unzip \
    lsof \
    lsb-core \
    libatlas-base-dev \
    libopencv-dev \
    python-opencv \
    python-pip \
    python3-pip \
    ffmpeg \
    zip \
    cython \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONPATH /usr/local/python/openpose
RUN export PATH="/usr/local/cuda-8.0/bin:$PATH"
RUN export LD_LIBRARY_PATH="/usr/local/cuda-8.0/lib64:$LD_LIBRARY_PATH"


# openpose
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git
WORKDIR /openpose
RUN git checkout tags/v1.4.0 \
    && git submodule update --init --recursive \
    && sed -i -e "s/ubuntu\/Makefile.example/ubuntu_deprecated\/Makefile.example/g" ubuntu_deprecated/install_openpose_if_cuda8.sh \
    && sed -i -e "s/ubuntu\/Makefile.config/ubuntu_deprecated\/Makefile.config/g" ubuntu_deprecated/install_openpose_if_cuda8.sh \
    # && sed -i -e "s/git submodule update --init --recursive//g" ubuntu/install_caffe_and_openpose_JetsonTX2_JetPack3.1.sh \
    && sed -i -e "s/ubuntu/ubuntu_deprecated/g" ubuntu_deprecated/install_caffe_and_openpose_if_cuda8.sh \
    && sync \
    && sleep 1 \
    && bash ./ubuntu_deprecated/install_caffe_and_openpose_if_cuda8.sh

# python API
RUN cd build && cmake -DBUILD_PYTHON=true ..
RUN cd build && make -j `nproc`
RUN cd build && sudo make install
#######################################################################
##                            ros install                            ##
#######################################################################
WORKDIR /
# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.2-0* \
    ros-kinetic-desktop-full=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./openpose_ros/dockerfile/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

#######################################################################
##                       install nvidia docker                       ##
#######################################################################

RUN apt-get update && apt-get install -y --no-install-recommends \
       pkg-config \
       libxau-dev \
       libxdmcp-dev \
       libxcb1-dev \
       libxext-dev \
       libx11-dev && \
   rm -rf /var/lib/apt/lists/*
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
 /usr/local/lib/x86_64-linux-gnu \
 /usr/local/lib/x86_64-linux-gnu
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
 /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json \
 /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json
RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
   ldconfig && \
   echo '/usr/local/$LIB/libGL.so.1' >> /etc/ld.so.preload && \
   echo '/usr/local/$LIB/libEGL.so.1' >> /etc/ld.so.preload
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
   ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
   ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

#######################################################################
##                   install additional packages                     ##
#######################################################################
# install opencv
RUN apt-get update && apt-get install -y
RUN apt-get install -y libopencv-dev \
      ros-kinetic-map-server \
      ros-kinetic-amcl \
      ros-kinetic-teleop-twist-joy \
      ros-kinetic-ros-numpy
# install openni2
RUN apt-get install -y ros-kinetic-rgbd-launch \
   ros-kinetic-openni2-camera \
   ros-kinetic-openni2-launch

#######################################################################
##                          make catkin_ws                           ##
#######################################################################
RUN mkdir -p /catkin_ws/src && \
   cd /catkin_ws/src && \
   /bin/bash -c 'source /opt/ros/kinetic/setup.bash;catkin_init_workspace'  && \
   cd /catkin_ws/ && \
   /bin/bash -c 'source /opt/ros/kinetic/setup.bash;catkin_make'

RUN cd /catkin_ws && \
   /bin/bash -c 'source /opt/ros/kinetic/setup.bash;catkin_make'

RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
COPY ./openpose_ros/dockerfile/catkin_entrypoint.sh /
ENTRYPOINT ["/catkin_entrypoint.sh"]
CMD ["bash"]

#######################################################################
##                   install additional packages                     ##
#######################################################################



#######################################################################
##                           ros settings                            ##
#######################################################################
#RUN echo "export ROS_IP=192.168.1.218.">> ~/.bashrc
#RUN echo "export ROS_HOME=~/.ros">> ~/.bashrc
#RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
#RUN echo "export ROS_MASTER_URI=http://192.168.1.221:11311" >> ~/.bashrc