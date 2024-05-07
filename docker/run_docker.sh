#!/bin/bash


CONTAINER_NAME=$1
if [[ -z "${CONTAINER_NAME}" ]]; then
    CONTAINER_NAME=depth-optimizer-v2
fi

# This specifies a mapping between a host directory and a directory in the
# docker container. This mapping should be changed if you wish to have access to
# a different directory
HOST_DIR=$2
if [[ -z "${HOST_DIR}" ]]; then
    HOST_DIR=`realpath ${PWD}/..`
fi

CONTAINER_DIR=$3
if [[ -z "${CONTAINER_DIR}" ]]; then
    CONTAINER_DIR=/root/ros2ws/src/depth_optimization
fi

echo "Container name     : ${CONTAINER_NAME}"
echo "Host directory     : ${HOST_DIR}"
echo "Container directory: ${CONTAINER_DIR}"
DEPTH_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`
if [ -z "${DEPTH_ID}" ]; then
    echo "Creating new depth_optimizer docker container."
    xhost +local:root
    docker run --gpus all  -it --privileged -v ${HOST_DIR}:${CONTAINER_DIR}:rw -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env="DISPLAY" --name=${CONTAINER_NAME} depth-optimizer:humble-v1 bash
    #--network=host ros2 not work with this
else
    echo "Found depth_optimizer docker container: ${DEPTH_ID}."
    # Check if the container is already running and start if necessary.
    if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then
        xhost +local:${DEPTH_ID}
        echo "Starting and attaching to ${CONTAINER_NAME} container..."
        docker start ${DEPTH_ID}
        docker attach ${DEPTH_ID}
    else
        echo "Found running ${CONTAINER_NAME} container, attaching bash..."
        docker exec -it ${DEPTH_ID} bash
    fi
fi
