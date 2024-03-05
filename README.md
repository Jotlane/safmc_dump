cd ~/safmc_dump

git clone https://github.com/Jotlane/safmc_dump.git

source /opt/ros/humble/setup.bash

colcon build

mv aruco_detection.dockerfile DOCKERFILE

sudo docker build -t aruco_detection .
xhosts +

sudo docker run --name aruco_detection --rm -it --network=host --ipc=host --device=/dev/video0 -e DISPLAY=unix$DISPLAY -v /home/binux/github/safmc_dump:/root/aruco_detection -v /tmp/.X11-unix:/tmp/.X11-unix aruco_detection:latest

source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 run aruco_detection detect_markers -d=16 -c=caliboutput.yaml -l=0.07
