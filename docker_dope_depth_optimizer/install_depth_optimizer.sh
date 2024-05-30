#!/bin/bash -i

set -e

source /opt/ros/humble/setup.bash
mkdir -p ${ROS2WS}/src



cd ~/ros2ws/src
git clone https://github.com/Vanvitelli-Robotics/depth_optimization.git

cd ~/ros2ws/src/depth_optimization/depth_optimization/

~/miniconda3/bin/conda init bash
source ~/.bashrc

# Import conda environment from YAML
~/miniconda3/bin/conda env create -f environment.yaml

# Activate the conda environment "py_depth_opt"
export PATH="${HOME}/miniconda3/bin:$PATH"
source activate py_depth_opt


# Python version update for ROS2 compatibility 
~/miniconda3/bin/conda install python=3.10

# Export python3.8 modules - This is integrated in the bashrc with the last line
export PYTHONPATH="$CONDA_PREFIX/lib/python3.8/site-packages:$PYTHONPATH"

# Alias for depth optimizer execution
echo "alias run_depth_optimizer_ros2='source /root/ros2ws/src/depth_optimization/depth_optimization/run_depth_opt.sh'" >> ~/.bashrc
echo "alias run_dope_depth_optimizer_ros2='source /root/ros2ws/src/depth_optimization/depth_optimization/run_dope_depth_opt.sh'" >> ~/.bashrc

# deactivate conda auto-initialization
/bin/bash -c "conda config --set auto_activate_base false"






