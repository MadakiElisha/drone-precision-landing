#!/bin/bash
#
# Enter the precision landing environment.
# Source this file (do not run it) so it affects your current shell.
#

# Load project configuration
source /home/madakie/precision_landing/config.sh

# Isolate from any other ROS workspaces sourced in this shell
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH

# Use CycloneDDS for this session
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS 2, then only our workspace
source /opt/ros/jazzy/setup.bash
source ${PROJECT_ROOT}/ros2_ws/install/setup.bash

# Activate Python virtual environment (system-site-packages)
if [ -d "${PROJECT_ROOT}/venv" ]; then
    source ${PROJECT_ROOT}/venv/bin/activate
fi

echo "[precision_landing] Environment loaded."
echo "  RMW:       ${RMW_IMPLEMENTATION}"
echo "  PX4_DIR:   ${PX4_DIR}"
echo "  MODEL:     ${PX4_SIM_MODEL}"
