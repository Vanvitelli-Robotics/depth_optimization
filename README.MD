# Depth Optimizer
ROS2 service server that implements a pose refinement alghoritm using depth data.

## Description
A solution to improve the pose estimation results getted from only rgb data for objects with variable dimensions (like an apple), using depth measures.  
* ***input***: estimated object pose, depth frame values from rgb-d camera, cad model of recongized object (models folder)
* ***output***: refined object pose 

## Getting Started

### Dependencies

* NVISII
```diff
pip install nvisii
```

