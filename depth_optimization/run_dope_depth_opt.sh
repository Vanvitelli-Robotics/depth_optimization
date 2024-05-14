#!/bin/bash

# Activate the conda environment "py_depth_opt"
conda activate py_depth_opt

# Export python3.8 modules - This should be integrated in the bashrc
export PYTHONPATH="$CONDA_PREFIX/lib/python3.8/site-packages:$PYTHONPATH"

# Run depth_optimizer node
 ros2 launch depth_optimization dope_depth_optimizer.launch.py 


