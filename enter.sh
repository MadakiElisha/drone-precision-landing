#!/bin/bash
# Enter the precision landing environment.
# Source this file (do not run it) so it affects your current shell.

# Override the global Zenoh setting with CycloneDDS for this session only
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/jazzy/setup.bash
source /home/madakie/precision_landing/ros2_ws/install/setup.bash

if [ -d "/home/madakie/precision_landing/venv" ]; then
    source /home/madakie/precision_landing/venv/bin/activate
fi

echo "[precision_landing] Environment loaded. RMW is now: $RMW_IMPLEMENTATION"
