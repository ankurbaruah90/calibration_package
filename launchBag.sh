#!/bin/bash

cd /home/ankur/detect_ws/
source devel/setup.bash
roslaunch calibration_package example.launch
rostopic pub --once /bagdone std_msgs/Bool True
