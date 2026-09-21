#!/bin/bash

source /opt/ros/humble/setup.bash

export GZ_SIM_RESOURCE_PATH="$HOME/skyscrub_sim/skyscrub_models:${GZ_SIM_RESOURCE_PATH}"

echo "======================================"
echo " SkyScrub simulation environment"
echo "======================================"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
