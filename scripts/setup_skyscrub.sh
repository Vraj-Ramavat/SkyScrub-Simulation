#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

case ":${GZ_SIM_RESOURCE_PATH:-}:" in
  *":$REPO_ROOT/models:"*)
    ;;
  *)
    export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
    ;;
esac

echo "======================================"
echo " SkyScrub simulation environment"
echo "======================================"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "SKYSCRUB_ROOT=$REPO_ROOT"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
