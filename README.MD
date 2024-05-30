# Depth Optimizer
In this repository there are different ROS2 implementations of an algorithm that refines the 6D pose coming from any pose estimation algorithm by exploiting depth measurements. It addresses the _dimension variability problem_ which affects the RGB-only based neural networks when objects of variable dimension have to be inferred. It solves the problem by setting up a geometric optimization problem. For more details, see our [paper](https://arxiv.org/abs/2305.15856).

## Repository structure
* **depth_optimization** : this folder contains the ros2 nodes that implement the depth_optimization algorithm. Follow the guide inside it for more details
* **depth_optimization_interfaces** : this folder contains the ros2 service definitions
* **docker** :  go in this folder if you want to run the depth_optimizer inside in a docker container. Follow the guide inside it for more details 
* **docker_dope_depth_optimizer** : go in this folder if you want to run the depth_optimizer inside a docker container in tandem with the 6D pose estimation algorithm [DOPE](https://github.com/Vanvitelli-Robotics/DOPE.git).
Follow the guide inside it for more details


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








