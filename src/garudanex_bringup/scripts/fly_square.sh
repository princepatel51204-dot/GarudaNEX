#!/usr/bin/env bash
# Fly a square circuit on /cmd_vel to build a map.
#

# --- PREFLIGHT: refuse to fly into a dead graph -----------------------------
_nav=$(timeout 5 ros2 topic echo /fmu/out/vehicle_status_v1 --once \
        --qos-reliability best_effort --qos-durability transient_local \
        --field nav_state 2>/dev/null | head -1)
_arm=$(timeout 5 ros2 topic echo /fmu/out/vehicle_status_v1 --once \
        --qos-reliability best_effort --qos-durability transient_local \
        --field arming_state 2>/dev/null | head -1)
if [ "${_arm}" != "2" ] || [ "${_nav}" != "14" ]; then
  echo "ABORT: arming_state='${_arm}' (need 2=ARMED), nav_state='${_nav}' (need 14=OFFBOARD)"
  echo "       run check.sh, then 'commander takeoff' + 'commander mode offboard' in the pxh shell"
  exit 1
fi
echo "preflight OK: ARMED + OFFBOARD"

#   ./fly_square.sh [side_seconds] [speed] [laps]
#
# PX4 must already be ARMED and in OFFBOARD mode:
#   pxh> commander takeoff        (wait ~8 s)
#   pxh> commander mode offboard
#
# A CLOSED circuit matters: loop closure can only fire when the drone
# re-observes a place it has already mapped. An open path just accumulates
# drift with nothing to correct it against.
set -euo pipefail

SIDE="${1:-6}"        # seconds of forward flight per side
SPEED="${2:-0.6}"     # m/s  - low speed keeps tilt small, which keeps the
                      #        2D scan near-planar (see MPC_TILTMAX_AIR)
LAPS="${3:-1}"
YAW_RATE=0.4          # rad/s
TURN=$(python3 -c "print(round(1.5708/${YAW_RATE}, 2))")   # 90 deg

pub() {  # pub <duration> <vx> <vy> <wz>
  timeout "$1" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $2, y: $3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $4}}" \
    >/dev/null 2>&1 || true
}

echo "square: ${SIDE}s sides at ${SPEED} m/s, ${LAPS} lap(s), turn ${TURN}s"
for lap in $(seq 1 "$LAPS"); do
  for side in 1 2 3 4; do
    echo "  lap ${lap} side ${side}: forward ${SIDE}s"
    pub "$SIDE" "$SPEED" 0.0 0.0
    echo "  lap ${lap} side ${side}: pause"
    pub 1.5 0.0 0.0 0.0          # settle so the scan is captured level
    echo "  lap ${lap} side ${side}: turn 90 deg"
    pub "$TURN" 0.0 0.0 "$YAW_RATE"
    pub 1.5 0.0 0.0 0.0
  done
done
echo "circuit complete - hovering"
