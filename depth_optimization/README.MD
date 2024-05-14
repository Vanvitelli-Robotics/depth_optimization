# Depth Optimizer
In this repository there are different ROS2 implementations of an algorithm that refines the 6D pose coming from any pose estimation algorithm by exploiting depth measurements. It addresses the _dimension variability problem_ which affects the RGB-only based neural networks when objects of variable dimension have to be inferred. It solves the problem by setting up a geometric optimization problem. For more details, see our [paper](https://arxiv.org/abs/2305.15856).

## Description
The node "depth_optimizer.py" exposes a python ROS2 service with the following interface:  
* ***inputs***: estimated object pose, depth frame values from rgb-d camera, cad model of recongized object (models folder)
* ***outputs***: refined object pose, scaling factor of the target object

The node "dope_depth_optimizer.py" was implementad to work directly with [DOPE](https://github.com/Vanvitelli-Robotics/DOPE.git).
It exposes a python ROS2 service with the following interface:  
* ***inputs***: class_id (int), n_max_poses (int), optimize (bool)
* ***outputs***: refined poses, scaling factos, sucesses

## Getting Started

### Dependencies
The ROS2 nodes(python 3.10) require the python module NVISII (https://github.com/owl-project/NVISII), which is available only for python <= 3.8. For compatibility reasons, you can use a conda environment as described below, else skip to 3) after installing NVISII.

1) Install conda [instructions](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html).

2) Setup the conda environment and all the required packages. 
You can directly create the conda environment using the environment.yaml file:
  ```diff
  source setup_environment.sh
  ```
In the same folder you can run these commands to add some aliases to run the nodes in the right conda environment
  ```diff
  echo "alias run_depth_optimizer_ros2='source $PWD/run_depth_opt.sh'" >> ~/.bashrc
  echo "alias run_dope_depth_optimizer_ros2='source $PWD/run_dope_depth_opt.sh'" >> ~/.bashrc
  ```
Alternativly, follow these steps:

2.1) Setup conda environment 
```diff
conda create --name py_depth_opt python=3.8
conda activate py_depth_opt
```

2.2) Install some required packages on python38 env
```diff
conda install -n py_depth_opt pip numpy scipy matplotlib scikit-learn
```

2.3) Install NVISII
```diff
pip install nvisii
```

2.4) Update python
```diff
conda update python
```


---
***NOTE***

If you use a virtual environment as conda, note that you have to correctly set the ROS2 variable $PYTHONPATH. Following the procedure described in https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html#saving-environment-variables, export the variable in the virtual environment by using
```diff
export PYTHONPATH="$CONDA_PREFIX/lib/python3.8/site-packages:$PYTHONPATH"
```

---

3) Run the node

Once the conda environment is created and the PYTHONPATH correctly set (if necessary), you can run the node:
```diff
conda activate py_depth_opt
export PYTHONPATH="$CONDA_PREFIX/lib/python3.8/site-packages:$PYTHONPATH"
ros2 launch depth_optimization depth_optimizer.launch.py 
```

---
The "depth_optimizer.py" depends on several parameters that can be set through the parameters.yaml file in the config folder (in the /depth_optimizer section). Moreover, the two parameters "mesh_path" and "mesh_scale", can be dinamically changed through the cli (ros2 param set) or by code. 

You can launch a simple client node for "depth_optimizer.py" with 
```diff
ros2 run depth_optimization simple_depth_optimizer_client 
```

The "dope_depth_optimizer.py" also depends on several parameters defined in config/parameters.yaml in the /dope_depth_optimizer section. Here you have to set the camera parameters, the optimization parameters and the class id list, mesh paths and mesh scales defined in the config/config_dope.yaml of [DOPE](https://github.com/Vanvitelli-Robotics/DOPE.git).

You can launch a simple client node for "dope_depth_optimizer.py" with 
```diff
ros2 run depth_optimization dope_depth_opt_client 
```



## How to cite 

If you use this software in a research project, please cite as follows:
```
@INPROCEEDINGS{10284072,
  author={Costanzo, Marco and De Simone, Marco and Federico, Sara and Natale, Ciro and Pirozzi, Salvatore},
  booktitle={2023 9th International Conference on Control, Decision and Information Technologies (CoDIT)}, 
  title={Enhanced 6D Pose Estimation for Robotic Fruit Picking}, 
  year={2023},
  volume={},
  number={},
  pages={901-906},
  keywords={Training;Visualization;Pose estimation;Force;Europe;Artificial neural networks;Sensors},
  doi={10.1109/CoDIT58514.2023.10284072}}

```








