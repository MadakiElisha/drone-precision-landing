#!/bin/bash
#
# Central configuration for the precision landing project.
#

# PX4-Autopilot installation path
export PX4_DIR="/home/madakie/PX4-Autopilot"

# Simulation model
export PX4_SIM_MODEL="gz_x500_pl"

# XRCE-DDS Agent port
export XRCE_PORT=8888

# Project root
export PROJECT_ROOT="/home/madakie/precision_landing"

# Custom world name (File must be symlinked into PX4/Tools/simulation/gz/worlds/)
export PX4_GZ_WORLD="precision_landing"
export PX4_GZ_MODEL_POSE="4,0,0.3,0,0,0"
