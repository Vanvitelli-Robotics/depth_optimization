#!/bin/bash
#

# Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
# Full license terms provided in LICENSE.md file.

# Stop in case of any error.
set -e

source /opt/ros/humble/setup.bash



# setup depth_optimizer
mkdir -p ${ROS2WS}/src
cd ${ROS2WS}/src/depth_optimization/depth_optimization


#source setup_environment.sh


#cd ${ROS2WS}/
#colcon build
