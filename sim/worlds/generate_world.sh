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
light_val = os.environ.get("PL_DIM_LIGHT", "0.9")
src = re.sub(r'(<intensity>)[\d\.]+(</intensity>)', f'\\g<1>{light_val}\\g<2>', src, count=1)

def box(name, x, y, z, sx, sy, sz, rgb, yaw=0.0):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
      </link>
    </model>"""

def cyl(name, x, y, z, r, h, rgb):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        </collision>
      </link>
    </model>"""

overlays = ["""
    <include>
      <uri>model://landing_pad</uri>
      <pose>4 0 0.01 0 0 0</pose>
    </include>"""]

if os.environ.get("PL_DECOY") == "1":
    overlays.append("""
    <include>
      <uri>model://decoy_marker</uri>
      <pose>5 0 0.01 0 0 0</pose>
    </include>""")

if os.environ.get("PL_OCCLUDER") == "1":
    overlays.append("""
    <include>
      <uri>model://occluder_box</uri>
      <pose>4 0 1.5 0 0 0</pose>
    </include>""")

if os.environ.get("PL_OBSTACLES") == "1":
    W, WH, BL = "0.35 0.35 0.35", "0.9 0.9 0.9", "0.1 0.2 0.8"
    # bounding walls (the arena)
    overlays.append(box("wall_n", 4, 9, 0.75, 14, 0.3, 1.5, W))
    overlays.append(box("wall_s", 4, -4, 0.75, 14, 0.3, 1.5, W))
    overlays.append(box("wall_w", -2, 3, 0.75, 0.3, 14, 1.5, W))
    overlays.append(box("wall_e", 10, 3, 0.75, 0.3, 14, 1.5, W))
    # wall segments threading the circuit
    overlays.append(box("seg_a", 6, 6, 0.6, 3, 0.3, 1.2, W, 0.6))
    overlays.append(box("seg_b", 2, 3.2, 0.6, 2.5, 0.3, 1.2, W, -0.5))
    # white cubes
    for i, (x, y) in enumerate([(7, 4.5), (4, 6.5), (1, 6), (0.5, 0.5)]):
        overlays.append(box(f"cube_{i}", x, y, 0.4, 0.8, 0.8, 0.8, WH))
    # blue cylinders
    for i, (x, y) in enumerate([(6.5, 3), (5, 5.5), (1.5, 4.5), (2, 1)]):
        overlays.append(cyl(f"cyl_{i}", x, y, 0.5, 0.25, 1.0, BL))

i = src.rfind("</world>")
assert i != -1
open(path, "w").write(src[:i] + "\n".join(overlays) + src[i:])
print(f"[generate] light={light_val} decoy={os.environ.get('PL_DECOY','0')} "
      f"occluder={os.environ.get('PL_OCCLUDER','0')} obstacles={os.environ.get('PL_OBSTACLES','0')}")
PY
