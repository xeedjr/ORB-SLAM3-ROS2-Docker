# Image taken from https://github.com/turlucode/ros-docker-gui
FROM osrf/ros:jazzy-desktop-full-noble
# FROM ubuntu:24.04

RUN apt-get update

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y gnupg2 curl vim wget python3-pip libpng16-16 libjpeg-turbo8 libtiff5-dev

RUN apt-get install -y \
    # Base tools
    cmake \
    build-essential \
    git \
    unzip \
    pkg-config \
    python3-dev \
    # OpenCV dependencies
    python3-numpy \
    # Pangolin dependencies
    libgl1-mesa-dev \
    libglew-dev \
    libpython3-dev \
    libeigen3-dev \
    mc \
    apt-transport-https \
    ca-certificates\
    software-properties-common

RUN apt update


#ROS2 Jazzy install 
# RUN locale  # check for UTF-8

# RUN apt update &&  apt install locales -y
# RUN locale-gen en_US en_US.UTF-8
# RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# RUN export LANG=en_US.UTF-8

# RUN locale  # verify settings

# RUN apt install software-properties-common -y
# RUN add-apt-repository universe

# RUN apt update &&  apt install curl -y
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

# RUN apt update &&  apt install ros-dev-tools -y

# RUN apt update

# RUN apt upgrade

# RUN apt install ros-jazzy-desktop -y


# boost
RUN apt-get install libboost-all-dev -y


# Build OpenCV
RUN apt-get install -y python3-dev python3-numpy
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
RUN apt-get install -y libgtk-3-dev

RUN cd /tmp && git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 4.10.0 && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF  -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j8 && make install && \
    cd / && rm -rf /tmp/opencv

# Build Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.9.2 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j8 && make install && \
    cd / && rm -rf /tmp/Pangolin

# Get Pangolin
# RUN apt-get install curl zip unzip tar nasm -y
# WORKDIR "/tmp"
# RUN git clone https://github.com/Microsoft/vcpkg.git
# WORKDIR "./vcpkg"
# RUN ./bootstrap-vcpkg.sh
# RUN ./vcpkg integrate install
# RUN ./vcpkg install pangolin

# WORKDIR "/tmp"
# RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
# WORKDIR "./Pangolin"

# # Install dependencies (as described above, or your preferred method)
# RUN apt update
# RUN apt install -y libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev libc++-dev libepoxy-dev libglew-dev libeigen3-dev cmake g++ ninja-build libjpeg-dev libpng-dev catch2 libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev libdc1394-dev libraw1394-dev libopenni-dev python3-dev

# # Configure and build
# RUN cmake -B build
# RUN cmake --build build
# RUN cmake install

# WORKDIR "/"



# # Build vscode (can be removed later for deployment)
# COPY ./container_root/shell_scripts/vscode_install.sh /root/
# RUN cd /root/ &&  chmod +x * && ./vscode_install.sh && rm -rf vscode_install.sh

# Build ORB-SLAM3 with its dependencies.
RUN apt-get update && apt-get install ros-jazzy-pcl-ros tmux -y
RUN apt-get install ros-jazzy-nav2-common x11-apps nano -y
COPY ORB_SLAM3 /home/orb/ORB_SLAM3
RUN . /opt/ros/jazzy/setup.sh && cd /home/orb/ORB_SLAM3 && mkdir -p build && ./build.sh
COPY orb_slam3_ros2_wrapper /root/colcon_ws/src/orb_slam3_ros2_wrapper
COPY slam_msgs /root/colcon_ws/src/slam_msgs
RUN apt install libpcl-dev -y
RUN . /opt/ros/jazzy/setup.sh && cd /root/colcon_ws/ && colcon build --symlink-install