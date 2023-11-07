#!/bin/bash

# Import conda environment from YAML
conda env create -f environment.yaml

# Activate the conda environment "py_depth_opt"
conda activate py_depth_opt

# Python version update for ROS2 compatibility 
conda install python=3.10

# Export python3.8 modules - This is integrated in the bashrc with the last line
export PYTHONPATH="$CONDA_PREFIX/lib/python3.8/site-packages:$PYTHONPATH"

# Alias for depth optimizer execution
echo "alias run_depth_optimizer_ros2='source $PWD/run_depth_opt.sh'" >> ~/.bashrc



