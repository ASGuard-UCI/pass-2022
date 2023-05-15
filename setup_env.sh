#!/bin/bash

# Install SVL Python API
git clone https://github.com/lgsvl/PythonAPI.git
cd PythonAPI
git checkout tags/2021.3
cd ..

pip install -e PythonAPI

# basedir=$(dirname "$0")
# if [ -d ${basedir}/plant_models ]; then
#     echo "SVL simulator and PythonAPI are already installed at ${basedir}/3rd_party"
#     exit
# fi
#
# mkdir -p ${basedir}/plant_models
# cd ${basedir}/plant_models
#
# # Install SVL simulator
# wget https://github.com/lgsvl/simulator/releases/download/2021.3/svlsimulator-linux64-2021.3.zip
# unzip svlsimulator-linux64-2021.3.zip
# rm -f svlsimulator-linux64-2021.3.zip
