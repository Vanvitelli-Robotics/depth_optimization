## DEPTH OPTIMIZER in a Docker Container

You can run the depth optimization directly in a docker container with Ubuntu 22.04 with ROS2 Humble and Python 3.10.

### Steps

1. **Download the depth optimzer code**
   ```
   $ git clone https://github.com/Vanvitelli-Robotics/depth_optimization.git depth_optimization
   ```

2. **Build the docker image**
   ```
   $ cd depth_optimization/docker
   $ docker build -t depth-optimizer:humble-v1 -f Dokerfile.humble ..
   ```
   This will take several minutes and requires an internet connection.

3. **Run the container**
   ```
   $ ./run_docker.sh [name] [host dir] [container dir]
   ```
   Parameters:
   - `name` is an optional field that specifies the name of this image. By default, it is `depth-optimizer-v2`.  By using different names, you can create multiple containers from the same image.
   - `host dir` and `container dir` are a pair of optional fields that allow you to specify a mapping between a directory on your host machine and a location inside the container.  This is useful for sharing code and data between the two systems.  By default, it maps the directory containing dope to `/root/ros2ws/src/depth_optimization` in the container.

      Only the first invocation of this script with a given name will create a container. Subsequent executions will attach to the running container allowing you -- in effect -- to have multiple terminal sessions into a single container.

4. **Install conda environment**
   Return to the installation instructions(../depth_optimization/readme.md).
   Inside the container
   ```
   $ cd ~/ros2ws
   $ colcon build
   $ source install/setup.bash
   $ ~/miniconda3/bin/conda init bash
   $ source ~/.bashrc
   $ cd ~/ros2ws/src/depth_optimization/depth_optimization
   $ source setup_environment.sh
   $ source ~/.bashrc
   $ conda deactivate 
   $ pip install -U scikit-learn
   ```
   This will install the py_depth_opt environment and add in the ~/.bashrc the alias
   ```
   run_depth_optimizer_ros2
   ```
   that you can use to activate in the correct way the conda environment and launch the ros2 server node for the depth optimization. 


   

