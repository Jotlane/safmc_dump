Run this on the rpi

```
cd ~/safmc_dump

git clone https://github.com/Jotlane/safmc_dump.git

source /opt/ros/humble/setup.bash

colcon build

mv aruco_detection.dockerfile DOCKERFILE

sudo docker build -t aruco_detection .

xhost +
```
This next line still problematic, change /dev/video0 to something that works. By default the working camera will be this, take note of the number
```
sudo docker run --name aruco_detection --rm -it --network=host --ipc=host --device=/dev/video0 -e DISPLAY=unix$DISPLAY -v /home/pi/safmc_dump:/root/aruco_detection -v /tmp/.X11-unix:/tmp/.X11-unix aruco_detection:latest
```

Run in the container
```
source /opt/ros/humble/setup.bash

source install/setup.bash
```

Change ci=0 to the same number as your dev/video*. ci stands for camera index
```
ros2 run aruco_detection detect_markers -d=16 -c=caliboutput.yaml -l=0.07 -ci=0
```
