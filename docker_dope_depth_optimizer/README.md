## DOPE + DEPTH OPTIMIZER in a Docker Container

A docker image containing both [DOPE](https://github.com/Vanvitelli-Robotics/DOPE.git) and Depth Optimizer can be created with the following guide. In the image will be installed also the ros2 pkg for running the [realsense](https://github.com/IntelRealSense/realsense-ros.git) camera.   

### Steps

1. **Download the depth optimzer repo**
   ```
   git clone https://github.com/Vanvitelli-Robotics/depth_optimization.git depth_optimization
   ```


2. **Build the docker image**
   ```
   cd depth_optimization/docker_dope_depth_optimizer/
   chmod +x build_docker.sh
   ./build_docker.sh
   ```
   This will take several minutes and requires an internet connection. If it fails, try first to run it again. 

3. **Run the container**
   First, add the weigths and the meshes in ```depth_optimization/docker_dope_depth_optimizer/recognition_files/meshes``` and ```depth_optimization/docker_dope_depth_optimizer/recognition_files/weights```. The meshes need to be in the obj format. 

   Run the container with
   ```
   xhost +local:root
   docker compose -f docker-compose_depth_opt_dope.yaml up -d
   docker exec -it ros2_vision_component bash
   ```


   Then, in the ```ros2_vision_component``` container run
   ```
   # make sure to deactivate every conda environment with 'conda deactivate' before building (otherwise you will have compilation errors)
   cd ~/ros2ws
   colcon build --symlink-install
   ```
   If accidentally you run the colcon build without deactivating the conda environment, deactivate the conda environment and run
   ```
   cd ~/ros2ws
   rm -r build/ install/ log/
   colcon build --symlink-install
   ```


4. **Run the inference**

   In the ```ros2_vision_component``` container, first run the realsense camera with
   ```
   ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
   ```
   or your specific camera node (make sure to have a topic streaming the aligned_depth_image).
   
   After updating the ```dope/config/config_dope.yaml```, the ```depth_optimization/config/parameters.yaml```, and, uploading the weights and meshes (weights and meshes), you can run the neural network DOPE with:
   ```
   ros2 launch dope_ros2 dope.launch.py 
   ```
   Then, in another terminal you can run the depth_optimization node with:
   ```
   run_dope_depth_optimizer_ros2
   ```
   A simple client to test the vision component can be run with:
   ```
   ros2 run depth_optimization dope_optimization_client class_id n_max_poses optimize
   ```
   Example for the client: 'ros2 run depth_optimization dope_optimization_client 5 3 True'.
   Once the simple client has received the response to the service, it will publish the received poses on the topic /refined_poses. 
   

 


   

