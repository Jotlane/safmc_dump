
FROM ros:humble as base

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install --no-install-recommends -y \
    software-properties-common 
RUN add-apt-repository universe

ENV UDEV=on

# Install vim
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y vim \
    cmake \
    g++ \
    wget \
    unzip \
    libgtk2.0-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*RUN apt update \
    && apt-get install -y python3-pip

RUN source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

RUN pip install --user -U empy==3.3.4 pyros-genmsg setuptools

#RUN apt-get install -y python3-libcamera python3-kms++
RUN apt-get install -y python3-prctl libatlas-base-dev ffmpeg python3-pip
RUN apt-get install -y python3-pyqt5 python3-opengl # only if you want GUI features
RUN pip3 install numpy --upgrade
RUN pip3 install picamera2

#RUN apt-get install -y python3-picamera2

RUN pip3 install meson ninja jinja2 ply
RUN apt install -y libyaml-dev python3-yaml python3-ply python3-jinja2
RUN apt install -y libdw-dev libunwind-dev
RUN apt install -y libudev-dev
RUN apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN apt install -y libpython3-dev pybind11-dev
RUN apt install -y libevent-dev libdrm-dev libjpeg-dev libsdl2-dev
RUN git clone https://github.com/raspberrypi/libcamera.git /opt/libcamera

WORKDIR /opt/libcamera
RUN meson setup build -Dcam=enabled -Dpycamera=enabled -Dpipelines=rpi/vc4,rpi/pisp
RUN sudo ninja -C build install

RUN apt install -y libopencv-dev
RUN git clone https://github.com/kbarni/LCCV.git /opt/LCCV
RUN mkdir /opt/LCCV/build
WORKDIR /opt/LCCV/build
RUN cmake ..
RUN sudo make install

RUN apt-get install -y \
    ros-humble-libcamera \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-common

# Add entrypoint to configure ros package and node with environmental variables
COPY aruco_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
