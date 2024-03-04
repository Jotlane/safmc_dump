Stuff to take note of
install libgtk2.0-dev and pkg-config
Install opencv all modules, refer to website
cmake -DWITH_GTK=ON ../opencv-4.x

(not needed for docker I think) add to the cmake file <<<set(OpenCV_DIR /usr/local/lib/cmake/opencv)>>>, add this above the find dependencies portion

(not needed for docker I think) maybe sudo ldconfig
(not needed for docker I think) uninstall the python pip opencv thing with pip pip uninstall opencv-contrib-python then pip cache purge
(not needed for docker I think) uninstall opencv with sudo apt remove libopencv*

rmb to modify the cmake file for each cpp file you add. refer to existing cmake file


ros2 run my_opencv_demo detect_markers -d=16 -c=caliboutput.yaml -l=0.07




todo: make it easier to setup the docker

sudo docker run --device=/dev/video0 -it 1a8f97f015e6 bash

