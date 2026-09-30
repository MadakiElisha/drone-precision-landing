#!/usr/bin/env bash
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
import sys, os, re
path = sys.argv[1]
src = open(path).read()

# 1. Dim light if requested
light_val = os.environ.get("PL_DIM_LIGHT", "0.9")
src = re.sub(r'(<intensity>)[\d\.]+(</intensity>)', f'\\g<1>{light_val}\\g<2>', src, count=1)

# 2. Overlays
overlays = []

# Always include real pad
overlays.append("""
    <!-- ==== Precision Landing project overlay ==== -->
    <include>
      <uri>model://landing_pad</uri>
      <pose>4 0 0.01 0 0 0</pose>
    </include>
""")

# Decoy
if os.environ.get("PL_DECOY") == "1":
    overlays.append("""
    <include>
      <uri>model://decoy_marker</uri>
      <pose>5 0 0.01 0 0 0</pose>
    </include>
""")

# Occluder
if os.environ.get("PL_OCCLUDER") == "1":
    overlays.append("""
    <include>
      <uri>model://occluder_box</uri>
      <pose>4 0 1.5 0 0 0</pose>
    </include>
""")

i = src.rfind("</world>")
assert i != -1, "closing </world> tag not found"
open(path, "w").write(src[:i] + "".join(overlays) + src[i:])
print(f"[generate] world updated: light={light_val}, decoy={'ON' if os.environ.get('PL_DECOY')=='1' else 'OFF'}, occluder={'ON' if os.environ.get('PL_OCCLUDER')=='1' else 'OFF'}")
PY
