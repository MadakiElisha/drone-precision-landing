#!/usr/bin/env bash
#
# Generate sim/worlds/<PX4_GZ_WORLD>.sdf from PX4's vendor world.
#
# Overlays applied:
#   1. Inner <world name> renamed to match PX4_GZ_WORLD (gz_bridge requirement).
#   2. Scene grid re-enabled: the vendor world sets <scene><grid>false</grid>;
#      this flag (default true) is what draws the visual ground grid.
#   3. Landing pad model included at (4, 0, 0.01).
#
# Note: We do not run `gz sdf` validation here because the standalone parser
# cannot resolve `model://` URIs without the full Gazebo runtime environment.
# The simulation itself validates the file successfully at runtime.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../../config.sh"

BASE_WORLD="${PX4_DIR}/Tools/simulation/gz/worlds/default.sdf"
OUT_WORLD="${SCRIPT_DIR}/${PX4_GZ_WORLD}.sdf"

[ -f "${BASE_WORLD}" ] || { echo "[error] base world missing: ${BASE_WORLD}"; exit 1; }

sed -e "s|<world name=\"default\">|<world name=\"${PX4_GZ_WORLD}\">|" \
    -e "s|<grid>false</grid>|<grid>true</grid>|" \
    "${BASE_WORLD}" > "${OUT_WORLD}"

python3 - "${OUT_WORLD}" << 'PY'
import sys
path = sys.argv[1]

PAD_OVERLAY = """
    <!-- ==== Precision Landing project overlay ==== -->
    <include>
      <uri>model://landing_pad</uri>
      <pose>4 0 0.01 0 0 0</pose>
    </include>
"""

src = open(path).read()
i = src.rfind("</world>")
assert i != -1, "closing </world> tag not found"
open(path, "w").write(src[:i] + PAD_OVERLAY + src[i:])
print("[generate] landing pad overlay inserted:", path)
PY
