#!/bin/sh

source ~/.bashrc
ENVNAME=rpi-hardware-monitor-ros

# use conda env if available and not already activated
if command -v conda >/dev/null 2>&1; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  if [ "$CONDA_DEFAULT_ENV" != "$ENVNAME" ]; then
    if conda env list | grep -Pq "^\s*$ENVNAME\s"; then
      conda activate $ENVNAME
    fi
  fi
fi

# source ROS2
if [ -d /opt/ros/rolling ]; then
    source /opt/ros/rolling/setup.bash
elif [ -d /opt/ros/humble ]; then
    source /opt/ros/humble/setup.bash
elif [ -d ~/ros2_rolling/install ]; then
    source ~/ros2_rolling/install/setup.bash
fi

if [ -d install/ ]; then
    source install/setup.sh
fi

export PYTHONPATH="${CONDA_PREFIX}/lib/python3.12/site-packages:$PYTHONPATH"
