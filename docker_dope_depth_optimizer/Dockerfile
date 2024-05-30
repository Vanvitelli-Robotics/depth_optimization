
FROM osrf/ros:humble-desktop


ENV HOME /root

# Install system and development components
RUN apt-get update && apt-get -y --no-install-recommends install \
    apt-utils \
    software-properties-common \
    build-essential \
    cmake \
    git \
    python3-pip \
    libxext6 \
    libx11-6 \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    freeglut3-dev \
    wget \
    && apt-get -y autoremove \
    && apt-get clean

# Install required ROS components
RUN apt-get update && apt-get -y --no-install-recommends install \
  ros-humble-cv-bridge \
  ros-humble-geometry-msgs \
  ros-humble-message-filters \
  ros-humble-resource-retriever \
  ros-humble-rclpy \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-tf2 \
  ros-humble-vision-msgs \
  ros-humble-visualization-msgs \
  ros-humble-rviz2 \
  && apt-get -y autoremove \
  && apt-get clean

RUN pip install -U scikit-learn


# install microconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh 
RUN bash Miniconda3-latest-Linux-x86_64.sh -b -u -p ${HOME}/miniconda3
RUN rm Miniconda3-latest-Linux-x86_64.sh

# Setup colcon workspace
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
ENV ROS2WS ${HOME}/ros2ws
COPY install_depth_optimizer.sh ${HOME}
RUN source ${HOME}/install_depth_optimizer.sh
RUN echo "source ${ROS2WS}/install/setup.bash" >> ${HOME}/.bashrc

# install dope 
COPY install_dope.sh ${HOME}
RUN source ${HOME}/install_dope.sh 

# install realsense camera pkg (optional)
SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
 && rm -rf /var/lib/apt/lists/*

ARG DEBIAN_FRONTEND=dialog


ENV DISPLAY :0
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TERM=xterm
# Some QT-Apps don't show controls without this
ENV QT_X11_NO_MITSHM 1
