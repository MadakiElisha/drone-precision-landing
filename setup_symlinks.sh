#!/bin/bash
#
# setup_symlinks.sh
# PX4 v1.15 hardcodes the Gazebo Sim world and model paths.
# This script symlinks our custom assets into the PX4 directory so they can be found.
# Run this once after cloning or moving the project.
#

set -e

# Load project paths
source "$(dirname "$0")/config.sh"

echo "[setup] Ensuring PX4 Gazebo Sim directories exist..."
mkdir -p "${PX4_DIR}/Tools/simulation/gz/worlds"
mkdir -p "${PX4_DIR}/Tools/simulation/gz/models"

echo "[setup] Linking custom world..."
ln -sf "${PROJECT_ROOT}/sim/worlds/precision_landing.sdf" "${PX4_DIR}/Tools/simulation/gz/worlds/precision_landing.sdf"

echo "[setup] Linking custom models..."
ln -sf "${PROJECT_ROOT}/sim/models/landing_pad" "${PX4_DIR}/Tools/simulation/gz/models/landing_pad"

echo "[setup] Done. Symlinks created successfully."
