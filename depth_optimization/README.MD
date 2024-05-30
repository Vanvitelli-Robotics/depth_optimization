# Depth Optimizer
Here you can find two implementations of the pose refinement algorithm 

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

3) Run the nodes

Once the conda environment is created and the PYTHONPATH correctly set (if necessary), you can run the node "depth_optimizer.py":
```diff
conda activate py_depth_opt
export PYTHONPATH="$CONDA_PREFIX/lib/python3.8/site-packages:$PYTHONPATH"
ros2 launch depth_optimization depth_optimizer.launch.py 
```
It is suggested to use the the alias 
```diff
run_depth_optimizer_ros2
```

---
The "depth_optimizer.py" depends on several parameters that can be set through the parameters.yaml file in the config folder (in the /depth_optimizer section). Moreover, the two parameters "mesh_path" and "mesh_scale", can be dinamically changed through the cli (ros2 param set) or by code. 

You can run a simple client node for "depth_optimizer.py" with 
```diff
ros2 run depth_optimization simple_depth_optimizer_client 
```

You can run "dope_depth_optimizer.py" using the alias 
```diff
run_dope_depth_optimizer_ros2
```

The "dope_depth_optimizer.py" also depends on several parameters defined in config/parameters.yaml in the /dope_depth_optimizer section. Here you have to set the camera parameters, the optimization parameters and the class id list, mesh paths and mesh scales defined in the config/config_dope.yaml of [DOPE](https://github.com/Vanvitelli-Robotics/DOPE.git).

You can run a simple client node for "dope_depth_optimizer.py" with 
```diff
ros2 run depth_optimization dope_optimization_client class_id n_max_poses optimize
```
Example for the client: 'ros2 run depth_optimization dope_optimization_client 5 3 True'.
Once the simple client has received the response to the service, it will publish the received poses on the topic /refined_poses. 











