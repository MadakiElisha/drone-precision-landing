#!/bin/bash
# Enter the precision landing environment.
# Source this file (do not run it) so it affects your current shell.

source /opt/ros/jazzy/setup.bash
source /home/madakie/precision_landing/ros2_ws/install/setup.bash

if [ -d "/home/madakie/precision_landing/venv" ]; then
    source /home/madakie/precision_landing/venv/bin/activate
fi

echo "[precision_landing] Environment loaded."
