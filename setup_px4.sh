#!/usr/bin/env bash
# Links GarudaNEX worlds into a PX4-Autopilot checkout so PX4 SITL can find them.
# Usage: bash setup_px4.sh [/path/to/PX4-Autopilot]
set -e
PX4=${1:-$HOME/PX4-Autopilot}
WS="$(cd "$(dirname "$0")" && pwd)"
W="$PX4/Tools/simulation/gz/worlds"
[ -d "$W" ] || { echo "PX4 worlds dir not found: $W"; exit 1; }
for f in "$WS"/src/garudanex_sim/worlds/*.sdf; do
  ln -sfv "$f" "$W/$(basename "$f")"
done
echo
echo "Linked worlds:"; ls -1 "$W" | grep garudanex
echo
echo "Airframe required: 4013_gz_x500_lidar_2d (ships with PX4 v1.17)"
echo "Model override:    export GZ_SIM_RESOURCE_PATH=$WS/src/garudanex_sim/models:\$GZ_SIM_RESOURCE_PATH"
