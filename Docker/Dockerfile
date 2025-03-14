FROM nvidia/cuda:12.4.0-devel-ubuntu20.04


ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic
ARG NPROC=4

WORKDIR /root


RUN apt-get update && apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y \
    wget unzip zip tar vim curl \
    libboost-all-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libeigen3-dev \
    libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev\
    libvtk7-dev libgtk-3-dev libcanberra-gtk-module libcanberra-gtk3-module 

RUN apt-get update && apt-get install -y libatlas-base-dev gfortran \
    libparmetis-dev \
    libtbb-dev \
    software-properties-common \
    git \
    gdb \
    build-essential \
    cmake \
    python3-pip \
    libopencv-contrib-dev \
    libopencv-dev && \
    rm -rf /var/lib/apt/lists/*



RUN python3 -m pip install evo

# pytorch and libtorch
RUN python3 -m pip install torch torchvision torchaudio
RUN wget https://download.pytorch.org/libtorch/cu124/libtorch-cxx11-abi-shared-with-deps-2.6.0%2Bcu124.zip && \
    unzip libtorch-cxx11-abi-shared-with-deps-2.6.0+cu124.zip




# install DBoW2
WORKDIR /root
RUN git clone https://github.com/dorian3d/DBoW2.git
RUN mkdir DBoW2/build
WORKDIR /root/DBoW2/build
RUN cmake .. && make -j$(NPROC) && make install

# install gtsam
WORKDIR /root
RUN wget -O gtsam-4.2.zip https://github.com/borglab/gtsam/archive/refs/tags/4.2.zip && \
    unzip gtsam-4.2.zip
RUN mkdir gtsam-4.2/build
WORKDIR /root/gtsam-4.2/build
RUN cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. && make && make install && cd ../..
    


# install ros packages
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-perception && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-vcstool python3-catkin-tools python3-osrf-pycommon


RUN rosdep init &&  rosdep update --rosdistro $ROS_DISTRO

RUN apt-get install -y \
    ros-${ROS_DISTRO}-hector-trajectory-server \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-image-geometry ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-cv-bridge pkg-config autoconf

    


RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc



# ultralytics library for YOLO
RUN python3 -m pip install \
    ultralytics \
    timm \
    transform \
    opencv-python 
    



# build openvins
RUN apt-get update &&  apt install -y \
    python3-dev \ 
    python3-matplotlib \
    python3-numpy \
    python3-psutil \
    python3-tk

WORKDIR /root
RUN git clone https://github.com/rpng/open_vins/ 
RUN mkdir open_vins/ov_core/build 
WORKDIR /root/open_vins/ov_core/build
RUN cmake .. && make -j$(NPROC) && make install

# WORKDIR /root
# RUN mkdir open_vins/ov_eval/build 
# WORKDIR /root/open_vins/ov_eval/build
# RUN cmake .. && make -j$(NPROC) && make install

WORKDIR /root
RUN mkdir open_vins/ov_init/build 
WORKDIR /root/open_vins/ov_init/build
RUN cmake .. && make -j$(NPROC) && make install

WORKDIR /root
RUN mkdir open_vins/ov_msckf/build 
WORKDIR /root/open_vins/ov_msckf/build
RUN cmake .. && make -j$(NPROC) && make install

# glog
WORKDIR /root
RUN git clone https://github.com/google/glog.git
WORKDIR /root/glog
RUN git checkout tags/v0.6.0
RUN cmake -S . -B build -G "Unix Makefiles" && cmake --build build --target install


WORKDIR /root
