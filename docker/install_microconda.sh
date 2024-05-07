#!/bin/bash

set -e

mkdir -p ${HOME}/miniconda3
cd ${HOME}/miniconda3

wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh

bash ${HOME}/miniconda3/miniconda.sh -b -u -p ${HOME}/miniconda3
rm -rf ${HOME}/miniconda3/miniconda.sh

${HOME}/miniconda3/bin/conda init bash


source ${HOME}/.bashrc