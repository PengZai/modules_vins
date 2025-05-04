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
    libopencv-dev \
    libpcl-dev && \
    rm -rf /var/lib/apt/lists/*



RUN python3 -m pip install evo

# pytorch and libtorch
RUN python3 -m pip install torch torchvision torchaudio
RUN wget https://download.pytorch.org/libtorch/cu124/libtorch-cxx11-abi-shared-with-deps-2.6.0%2Bcu124.zip && \
    unzip libtorch-cxx11-abi-shared-with-deps-2.6.0+cu124.zip



# nvidia-cuda-toolkit and cudnn
# WORKDIR /root
# RUN rm -f /etc/apt/sources.list.d/cuda*.list
# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
# RUN dpkg -i cuda-keyring_1.1-1_all.deb
# RUN apt-get update && apt-get install -y zlib1g  nvidia-cuda-toolkit cuda-toolkit cudnn9-cuda-12

# OpenCV 4.11
# WORKDIR /root
# RUN git clone https://github.com/opencv/opencv_contrib.git
# RUN cd opencv_contrib && git checkout tags/4.2.0 && cd ..
# RUN git clone https://github.com/opencv/opencv.git
# RUN cd opencv && git checkout tags/4.2.0 && mkdir build
# WORKDIR /root/opencv/build
# RUN cmake \
#     -D WITH_CUDA=ON \
#     -D CUDA_ARCH_BIN=6.1 \
#     -D OPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules \
#     -D WITH_VTK=ON \
#     -D BUILD_EXAMPLES=OFF \
#     -D CMAKE_BUILD_TYPE=Release \
#     -D CMAKE_INSTALL_PREFIX=/usr/local \
#     .. && \
#     make -j$(NPROC) && make install

# OpenCV 4.11
# WORKDIR /root
# RUN git clone https://github.com/opencv/opencv_contrib.git
# RUN cd opencv_contrib && git checkout 4.x && cd ..
# RUN git clone https://github.com/opencv/opencv.git
# RUN cd opencv && git checkout 4.x && mkdir build
# WORKDIR /root/opencv/build
# RUN cmake \
#     -D OPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules \
#     -D WITH_VTK=ON \
#     -D CMAKE_BUILD_TYPE=Release \
#     -D CMAKE_INSTALL_PREFIX=/usr/local \
#     .. && \
#     make -j$(NPROC) && make install


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
    timm==0.5.4 \
    opencv-python \
    matplotlib \
    tqdm \
    tensorboard \
    scikit-image

    


# glog
WORKDIR /root
RUN git clone https://github.com/google/glog.git
WORKDIR /root/glog
RUN git checkout tags/v0.6.0
RUN cmake -S . -B build -G "Unix Makefiles" && cmake --build build --target install



# Sophus
WORKDIR /root
RUN git clone https://github.com/strasdat/Sophus.git
RUN cd Sophus && git checkout tags/1.22.4 && mkdir build
WORKDIR /root/Sophus/build
RUN cmake .. && make -j$(NPROC) && make install


RUN apt-get update &&  apt install -y \
        libgl1-mesa-dev \
        libglew-dev 



#Pangolin
WORKDIR /root
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
RUN cd Pangolin && git checkout tags/v0.6 && mkdir build
WORKDIR /root/Pangolin/build
RUN cmake .. && make -j$(NPROC) && make install





WORKDIR /root
