#!/bin/bash -i

set -e

source /opt/ros/humble/setup.bash
#mkdir -p ${ROS2WS}/src



cd ~/ros2ws/src
git clone https://github.com/Vanvitelli-Robotics/DOPE.git dope -b ros2_humble

cd ~/ros2ws/src/dope
python3 -m pip install -r requirements.txt
cd ~/ros2ws
colcon build --packages-select dope





