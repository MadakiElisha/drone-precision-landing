#!/bin/bash
#
# Central configuration for the precision landing project.
# Source this file from other scripts to get consistent paths.
#

# PX4-Autopilot installation path
export PX4_DIR="/home/madakie/PX4-Autopilot"

# Simulation model to use
export PX4_SIM_MODEL="gz_x500"

# XRCE-DDS Agent port
export XRCE_PORT=8888

# Project root
export PROJECT_ROOT="/home/madakie/precision_landing"
