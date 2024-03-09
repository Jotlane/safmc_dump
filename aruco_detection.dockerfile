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
RUN cmake -DWITH_GTK=ON -DWITH_GSTREAMER=ON -DOPENCV_EXTRA_MODULES_PATH=/home/build_opencv/opencv_contrib-4.x/modules /home/build_opencv/opencv-4.x

# Build
RUN cmake --build . -j3

RUN sudo make -j2 install

RUN sudo ldconfig

RUN apt install -y ros-humble-cv-bridge ros-humble-vision-opencv ros-humble-image-common

RUN apt install vim -y

RUN apt install v4l-utils -y

RUN apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

WORKDIR /root/aruco_detection
