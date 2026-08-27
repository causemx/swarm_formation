#!/usr/bin/env bash
# Start N PX4 SIH SITL instances, each with its own home position.
#
#   ./run_sitl.sh [N]        (default 5)
#
# Build once first:
#   cd $PX4_DIR && make px4_sitl_sih sihsim_quadx
#
# Giving every vehicle a *different* home is deliberate: it means each EKF gets
# a different local origin, which is exactly the condition the swarm-frame
# transform in swarm_node.py exists to handle. Set them all equal and the demo
# still works, but it stops proving anything.

set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BUILD="$PX4_DIR/build/px4_sitl_sih"
N="${1:-5}"

# Must match ORIGIN_* in swarm_config.py
LAT0=47.397742
LON0=8.545594
ALT0=488.0
DLON=0.00011      # ~8 m of longitude at this latitude

if [ ! -x "$BUILD/bin/px4" ]; then
    echo "px4 binary not found at $BUILD/bin/px4"
    echo "build it with:  cd $PX4_DIR && make px4_sitl_sih sihsim_quadx"
    echo "(or set PX4_DIR=/path/to/PX4-Autopilot)"
    exit 1
fi

echo "stopping any previous instances"
pkill -f 'px4_sitl_sih/bin/px4' 2>/dev/null || true
sleep 1

for ((i = 0; i < N; i++)); do
    WD="$BUILD/instance_$i"
    mkdir -p "$WD"
    LON=$(python3 -c "print(f'{$LON0 + $i * $DLON:.7f}')")

    (
        cd "$WD"
        PX4_SIM_MODEL=sihsim_quadx \
        PX4_HOME_LAT=$LAT0 PX4_HOME_LON=$LON PX4_HOME_ALT=$ALT0 \
            "$BUILD/bin/px4" -i "$i" -d "$BUILD/etc" >"$WD/px4.log" 2>&1
    ) &

    printf 'instance %d  home %s,%s  mavsdk udp %d  log %s/px4.log\n' \
        "$i" "$LAT0" "$LON" "$((14540 + i))" "$WD"
    sleep 2
done

echo
echo "$N instances running. QGroundControl auto-connects on 14550."
echo "Wait for GPS lock (~15 s), then:  ./run_nodes.sh $N"
wait
