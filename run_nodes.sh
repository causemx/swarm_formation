#!/usr/bin/env bash
# Start one swarm node process per drone.
#
#   ./run_nodes.sh [N]       (default 5)
#
# Run this only after ./run_sitl.sh has come up and the vehicles have a GPS fix.

set -euo pipefail

N="${1:-5}"
LOGDIR="${LOGDIR:-/tmp/px4_swarm}"
mkdir -p "$LOGDIR"

PIDS=()
cleanup() {
    echo
    echo "stopping nodes"
    for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

for ((i = 0; i < N; i++)); do
    if ((i == 0)); then
        python3 swarm_node.py --id "$i" --command-interface >"$LOGDIR/node_$i.log" 2>&1 &
    else
        python3 swarm_node.py --id "$i" >"$LOGDIR/node_$i.log" 2>&1 &
    fi
    PIDS+=("$!")
    echo "node $i  ->  $LOGDIR/node_$i.log"
    sleep 0.5
done

echo
echo "follow the leader with:   tail -f $LOGDIR/node_0.log"
echo "see the whole swarm with: python3 swarm_monitor.py"
echo "ctrl-c to stop"
wait
