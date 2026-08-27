"""
Swarm topology, common reference frame and tuning constants.

Everything that describes "what the swarm is" lives here; the node logic in
swarm_node.py is generic and reads only this file.
"""

from dataclasses import dataclass

# --------------------------------------------------------------- communication
UDP_PORT = 45455

# Leader command channel (leader_commands.py / swarm_cli.py). Separate from
# UDP_PORT: that link is broadcast telemetry keyed by node id, where the
# newest message for an id overwrites the last -- a command sent on it would
# either collide with the telemetry schema or get clobbered before the leader
# read it. This is a direct, unicast, operator<->leader channel instead.
CMD_PORT = UDP_PORT + 1

# Loopback broadcast: every SITL instance + node process runs on this machine.
# On a real network (or several machines) use the subnet broadcast address,
# e.g. "192.168.1.255".
BCAST_ADDR = "127.255.255.255"

PUBLISH_HZ = 20.0      # state broadcast + offboard setpoint rate (PX4 needs > 2 Hz)
PEER_TIMEOUT = 1.0     # s; parent state older than this counts as "no parent"

# ---------------------------------------------------------------- common frame
# Each vehicle has its own EKF origin (wherever it was powered up), so raw
# local-NED coordinates are NOT comparable between vehicles. Every node
# therefore converts to this single "swarm frame" before broadcasting, and
# converts back into its own local frame before sending a setpoint.
#
# Keep this equal to the PX4_HOME_* used for instance 0 in run_swarm.sh.
ORIGIN_LAT = 47.397742
ORIGIN_LON = 8.545594
ORIGIN_ALT = 488.0

# -------------------------------------------------------------------- control
# "pos_vel": position setpoint + parent velocity as feed-forward. PX4's own
#            position loop (MPC_XY_P / MPC_Z_P) closes the error. Robust.
# "vel"    : explicit outer loop  v_cmd = KP_POS * err + FF_GAIN * v_parent.
#            Gives you a knob to study string stability of the chain.
SETPOINT_MODE = "pos_vel"

KP_POS = 0.9        # 1/s, only used when SETPOINT_MODE == "vel"
V_MAX = 8.0         # m/s, clamp on the commanded velocity in "vel" mode
FF_GAIN = 1.0       # 0..1, how much of the parent velocity is fed forward

TAKEOFF_ALT = 5.0   # m AGL, initial climb over own home before joining formation
ARRIVAL_R = 2.0     # m, leader waypoint acceptance radius
LEADER_SPEED = 4.0  # m/s, feed-forward speed along the leader path
LEG_TIMEOUT = 120.0  # s, give up on a leader waypoint and move to the next one


@dataclass
class NodeCfg:
    node_id: int
    level: int                    # 0 = swarm leader
    parent: int | None         # node this one follows; None for the leader
    offset: tuple[float, float, float]   # (fwd, right, down) [m] in PARENT body frame
    px4_instance: int             # PX4 SITL  -i <n>

    @property
    def mavsdk_url(self) -> str:
        # PX4 SITL sends to 14540 + instance for offboard APIs.
        # MAVSDK-Python 1.x wants "udp://:14540" instead of "udpin://...".
        return f"udpin://0.0.0.0:{14540 + self.px4_instance}"

    @property
    def grpc_port(self) -> int:
        # Each process spawns its own mavsdk_server, so they need distinct ports.
        return 50060 + self.px4_instance


# --------------------------------------------------------------------- topology
#
#            (0) leader
#            /        \
#         (1)          (2)          level 1  -- follow the leader
#          |            |
#         (3)          (4)          level 2  -- follow a level-1 node
#
# Offsets are expressed in the parent's body frame, so the whole formation
# rotates with the parent's heading.
SWARM = {
    0: NodeCfg(0, 0, None, (0.0, 0.0, 0.0), 0),
    1: NodeCfg(1, 1, 0, (-8.0, -8.0, -2.0), 1),
    2: NodeCfg(2, 1, 0, (-8.0, +8.0, -2.0), 2),
    3: NodeCfg(3, 2, 1, (-8.0, -8.0, -2.0), 3),
    4: NodeCfg(4, 2, 2, (-8.0, +8.0, -2.0), 4),
}

# Leader path, in the common swarm frame: (north, east, down) [m].
LEADER_PATH = [
    (0.0, 0.0, -20.0),
    (80.0, 0.0, -20.0),
    (80.0, 80.0, -25.0),
    (0.0, 80.0, -25.0),
    (0.0, 0.0, -20.0),
]
