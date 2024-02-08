# Depth Optimizer
This is a ROS2 node to refine the 6D pose coming from any pose estimation algorithm by exploiting depth measurements. It addresses the _dimension variability problem_ which affects the RGB-only based neural networks when objects of variable dimension have to be inferred. It solves the problem by setting up a geometric optimization problem. For more details, see our [paper](https://arxiv.org/abs/2305.15856).

## Description
The node exposes a python ROS2 service with the following interface:  
* ***inputs***: estimated object pose, depth frame values from rgb-d camera, cad model of recongized object (models folder)
* ***outputs***: refined object pose, scaling factor of the target object

## Getting Started

### Dependencies
The ROS2 node (python 3.10) requires the python module NVISII (https://github.com/owl-project/NVISII), which is available only for python <= 3.8. For compatibility reasons, you can use a conda environment as described below, else skip to 4).

1) Install conda [instructions](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html).


2.a) Run the bash script to create the conda environment from the environment.yaml file 
```diff
source setup_environment.sh
```

Alternativly you can directly create the environment

2.b) Setup conda environment 
```diff
conda create --name py_depth_opt python=3.8
conda activate py_depth_opt
```

2.b.1) Install some required packages on python38 env
```diff
conda install -n py_depth_opt pip numpy scipy matplotlib scikit-learn
```

2.b.2) Install NVISII
```diff
pip install nvisii
```

2.b.3) Update python
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

Once the conda environemnt is created and the PYTHONPATH correctly set, you can run the node:
```diff
conda activate py_depth_opt
ros2 launch depth_optimization depth_optimizer.launch.py 
```

---
The depth optimizer node depends on several parameters that can be set through the parameters.yaml file in the config folder. Moreover, the two parameters "mesh_path" and "mesh_scale", can be dinamically changed through the cli (ros2 param set) or bu code. 

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








