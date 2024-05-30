#!/bin/bash

DOCKERBUILD_PATH=$(pwd)

cd $DOCKERBUILD_PATH
docker build -t intelliman-uc2-vision-component:humble -f Dockerfile .