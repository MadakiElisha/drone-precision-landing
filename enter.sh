#!/bin/bash
#
# Enter the precision landing environment.
# Source this file (do not run it) so it affects your current shell.
#

# Load project configuration
source /home/madakie/precision_landing/config.sh

# Override the global Zenoh setting with CycloneDDS for this session
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Source our workspace
source ${PROJECT_ROOT}/ros2_ws/install/setup.bash

# Activate Python virtual environment
if [ -d "${PROJECT_ROOT}/venv" ]; then
    source ${PROJECT_ROOT}/venv/bin/activate
fi

echo "[precision_landing] Environment loaded."
echo "  RMW:       ${RMW_IMPLEMENTATION}"
echo "  PX4_DIR:   ${PX4_DIR}"
echo "  MODEL:     ${PX4_SIM_MODEL}"
