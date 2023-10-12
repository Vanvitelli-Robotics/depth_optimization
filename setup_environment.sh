#!/bin/bash

# Import conda environment from YAML
conda env create -f environment.yaml

# little pause
sleep 1
# Activate the conda environment "py_depth_opt"
conda activate py_depth_opt

# Python version update for ROS2 compatibility 
conda install python=3.10

# Export python3.8 modules - This should be integrated in the bashrc
export PYTHONPATH="$CONDA_PREFIX/lib/python3.8/site-packages:$PYTHONPATH"



