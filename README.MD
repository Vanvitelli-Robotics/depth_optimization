# Depth Optimizer
ROS2 service server that implements a pose refinement alghoritm using depth data.

## Description
A solution to improve the pose estimation results getted from only rgb data for objects with variable dimensions (like an apple), using depth measures.  
* ***input***: estimated object pose, depth frame values from rgb-d camera, cad model of recongized object (models folder)
* ***output***: refined object pose 

## Getting Started

### Dependencies
You need to install NVISII under python <=3.8 and numpy library. If your pyhton version is higher than the required one, you can use a conda env as described below, else skip to 4).

1) Install conda [instructions](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html).


2.a) Run the bash script to create the conda environment from the environment.yaml file 
```diff
source setup_environment.sh
```

Alternativly you can directly create the environment

2.b) Setup conda environment 
```diff
conda create --name python38 python=3.8
conda activate python38
```

2.b.1) Install some required packages on python38 env
```diff
conda install -n python38 pip numpy scipy matplotlib scikit-learn
```

2.b.2) Install NVISII
```diff
pip install nvisii
```

---
***NOTE***

If you use a virtual environment as conda, note that you have to correctly set the ROS2 variable $PYTHONPATH so that python 3.8 will be used. Following the procedure described in https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html#saving-environment-variables, export the variable in the virtual environment by using
```diff
export PYTHONPATH="your_envs_path/python38/lib/python3.8/site-packages:$PYTHONPATH"
```

---






