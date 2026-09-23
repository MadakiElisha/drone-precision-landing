#!/bin/bash
#
# setup_symlinks.sh
# Publishes this project's simulation assets into the PX4 tree.
#
# Safety rules (multi-project coexistence):
#   - Never deletes anything except our own stale symlinks.
#   - Never overwrites a path this project does not own.
#   - Symlinks only created if destination absent or already ours.
#
# Note: airframes are COPIED, not symlinked. The PX4 ROMFS build does not
# follow symlinks, so linked airframes never reach the runtime rootfs.
# Airframe ID range 4100-4199 is reserved for this project (see README).
#
set -euo pipefail

source "$(dirname "$0")/config.sh"

safe_link() {
  local src="$1" dst="$2" cur
  if [ -L "$dst" ]; then
    cur="$(readlink -f "$dst")"
    if [ "$cur" = "$(readlink -f "$src")" ]; then
      echo "[setup] ok (already linked): $dst"
      return 0
    fi
    echo "[setup] SKIP (foreign symlink, left untouched): $dst -> $cur"
    return 0
  fi
  if [ -e "$dst" ]; then
    echo "[setup] SKIP (path exists, not ours, left untouched): $dst"
    return 0
  fi
  ln -s "$src" "$dst"
  echo "[setup] linked: $dst"
}

safe_copy_airframe() {
  local src="$1" dst="$2"
  if [ -L "$dst" ]; then
    case "$(readlink -f "$dst")" in
      *precision_landing*) rm -f "$dst"; echo "[setup] replaced our stale link: $dst";;
      *) echo "[setup] SKIP (foreign symlink, left untouched): $dst"; return 0;;
    esac
  fi
  cp "$src" "$dst"
  echo "[setup] copied airframe: $dst"
}

mkdir -p "${PX4_DIR}/Tools/simulation/gz/worlds"
mkdir -p "${PX4_DIR}/Tools/simulation/gz/models"

safe_link "${PROJECT_ROOT}/sim/worlds/precision_landing.sdf" \
          "${PX4_DIR}/Tools/simulation/gz/worlds/precision_landing.sdf"

safe_link "${PROJECT_ROOT}/sim/models/landing_pad" \
          "${PX4_DIR}/Tools/simulation/gz/models/landing_pad"

safe_link "${PROJECT_ROOT}/sim/models/x500_pl" \
          "${PX4_DIR}/Tools/simulation/gz/models/x500_pl"

safe_copy_airframe "${PROJECT_ROOT}/sim/airframes/4100_gz_x500_pl" \
          "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4100_gz_x500_pl"

echo "[setup] Done."
