# Hierarchical leader–follower swarm — PX4 + MAVSDK-Python + UDP broadcast

Minimal formation-control demo on PX4 SITL with the SIH simulator. No fault
tolerance, no collision avoidance, no geofence — by design. Do not fly this on
real hardware without adding those.

```
        (0) leader          flies LEADER_PATH in the common swarm frame
        /        \
     (1)          (2)       level 1 — track the leader
      |            |
     (3)          (4)       level 2 — track a level-1 node
```

Each drone is one OS process talking to one PX4 SITL instance. A node broadcasts
its own state and reads its parent's out of the same broadcast cache — which is
why a follower can also be a parent. That is the entire hierarchy mechanism;
adding a tier is a line in `swarm_config.SWARM`.

## Files

| File | Role |
| --- | --- |
| `swarm_config.py` | topology, frame origin, gains, leader path — the only file you normally edit |
| `swarm_node.py` | per-drone process: connect, arm, climb, then lead or follow |
| `swarm_link.py` | UDP broadcast socket + peer state cache |
| `formation.py` | frame transform, offset rotation, vector clamp |
| `geo.py` | flat-earth geodetic ↔ NED |
| `swarm_monitor.py` | passive listener, prints live formation error |
| `leader_commands.py` | leader-only command channel: ARM/TAKEOFF/GOTO/HOLD/LAND, opt in with `--command-interface` |
| `swarm_cli.py` | operator CLI/REPL that sends commands to the leader over `leader_commands.py` |
| `test_offline.py` | checks the math and the socket without PX4 |
| `run_sitl.sh` / `run_nodes.sh` | launchers |

## Commanding the leader interactively

By default the leader still flies the fixed `LEADER_PATH` square. To drive it
from an operator console instead:

```bash
python3 swarm_node.py --id 0 --command-interface   # leader, waits for ARM
python3 swarm_node.py --id 1                        # followers, unchanged
python3 swarm_node.py --id 2
python3 swarm_node.py --id 3
python3 swarm_node.py --id 4

python3 swarm_cli.py arm
python3 swarm_cli.py takeoff --alt 5
python3 swarm_cli.py goto --ned 20 10 -15
python3 swarm_cli.py land
python3 swarm_cli.py                                # or just run the REPL
```

`run_nodes.sh` and `node_supervisor.py` don't pass `--command-interface`
through, since neither takes per-node flags -- start node 0 by hand as above
for command mode, or add the flag to either launcher yourself if you want it
wired in by default.

Followers are unaffected either way -- they always track whatever their
parent broadcasts, scripted or commanded. `GOTO`/`TAKEOFF` ack as soon as the
leader accepts the command, not on arrival; watch `swarm_monitor.py` for
progress.

## Prerequisites

```bash
pip install mavsdk                       # 2.x; for 1.x see the note below
pip install cython "git+https://github.com/sybrenstuvel/Python-RVO2"
                                          # ORCA collision avoidance (see avoidance.py) --
                                          # builds a native extension, so needs cmake + a
                                          # C++ compiler; not on PyPI under any name, hence
                                          # the git URL instead of a plain package name
cd ~/PX4-Autopilot
make px4_sitl_sih sihsim_quadx           # builds SIH without Gazebo deps
```

SIH runs the physics inside PX4 itself with no external simulator process, so
five instances are comfortable on a laptop.

## Run

```bash
python3 test_offline.py                  # sanity-check the math first
./run_sitl.sh 5                          # terminal 1 — five PX4 instances
./run_nodes.sh 5                         # terminal 2 — wait ~15 s for GPS lock
python3 swarm_monitor.py                 # terminal 3 — optional
```

`PX4_DIR=/path/to/PX4-Autopilot ./run_sitl.sh` if PX4 isn't in `~`.
QGroundControl auto-connects on 14550 and shows all five vehicles.

Expected sequence: every vehicle arms, climbs to 5 m over its own home, then the
tree comes up top-down; the leader walks an 80 m square while the followers close
into their slots. When the leader finishes it publishes `st: "land"`, its
children see that and land, and they in turn publish `land` for the level-2
nodes.

## How it fits together

**Ports.** PX4 SITL offsets its MAVLink ports by instance number, so instance
*i* is reachable on `udpin://0.0.0.0:{14540+i}`. Each Python process also spawns
its own `mavsdk_server`, so `System(port=50060+i)` keeps the gRPC ports apart.

**Frames.** Each vehicle's EKF origin sits wherever it booted, so raw local NED
is not comparable between vehicles. Every node computes a constant offset `T`
from one simultaneous (GPS fix, local NED) pair, publishes `local + T` in the
common swarm frame, and subtracts `T` again before sending a setpoint. This is
why `run_sitl.sh` deliberately gives each instance a different home — set them
all equal and the demo still works but stops proving anything.

**Formation.** The offset `(fwd, right, down)` is rotated by the parent's yaw, so
the formation turns with the parent rather than staying locked to compass north.
The follower sends a position setpoint plus the parent's velocity as
feed-forward; PX4's own position loop closes the error. Set
`SETPOINT_MODE = "vel"` to replace that with an explicit
`v = KP_POS·err + FF_GAIN·v_parent` outer loop, which is the knob to turn if you
want to study how disturbances amplify down the chain.

**Comms.** JSON over UDP broadcast at 20 Hz, fire and forget. Nodes bind the same
port with `SO_REUSEPORT`; broadcast datagrams reach every socket, whereas plain
unicast to that port would be load-balanced to just one — so the sender must
target `BCAST_ADDR`, not `127.0.0.1`. On a real network set `BCAST_ADDR` to the
subnet broadcast (e.g. `192.168.1.255`).

## Tuning

| Symptom | Try |
| --- | --- |
| Followers lag on turns | raise `FF_GAIN` toward 1.0, slow `LEADER_SPEED` |
| Level-2 oscillates, level-1 doesn't | error is amplifying down the chain — lower `KP_POS`, widen offsets, or flatten the tree |
| Followers never leave hover | check `swarm_monitor.py` sees the parent; if not, it's `BCAST_ADDR` or a firewall |
| Slow to reach waypoints | PX4's `MPC_XY_VEL_MAX` caps you regardless of `LEADER_SPEED` |

## Known limitations

Deliberate omissions, each of which is real work to add: no collision avoidance
between slots; a lost parent freezes its whole subtree in place forever; no
leader election, arming-state check, battery check, geofence or RTL; broadcast is
unauthenticated and unencrypted -- so is the leader command channel
(`leader_commands.py` / `swarm_cli.py`), so anyone who can reach `CMD_PORT` can
arm, fly, or land the leader; the flat-earth projection is only good for a few
hundred metres around the origin.

## MAVSDK version note

The URL scheme changed between major versions. `swarm_config.NodeCfg.mavsdk_url`
emits the 2.x form `udpin://0.0.0.0:14540`; on MAVSDK-Python 1.x change it to
`udp://:14540`. If `set_rate_attitude_euler` is missing on your version the node
logs a warning and carries on at PX4's default telemetry rate.
