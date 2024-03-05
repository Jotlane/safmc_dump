FROM ros:humble as base

# Install prerequisites
RUN apt update && apt install -y \
    cmake \
    g++ \
    wget \
    unzip \
    libgtk2.0-dev \
    pkg-config

# Download and unpack sources
WORKDIR /home
RUN mkdir build_opencv
WORKDIR /home/build_opencv
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip

# Create build directory and switch into it
RUN mkdir -p /build
WORKDIR /build

# Configure
RUN cmake -DWITH_GTK=ON -DOPENCV_EXTRA_MODULES_PATH=/home/build_opencv/opencv_contrib-4.x/modules /home/build_opencv/opencv-4.x

# Build
RUN cmake --build . -j3

RUN sudo make -j2 install

RUN sudo ldconfig

WORKDIR /root/aruco_detection
