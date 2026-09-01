"""
Swarm topology, common reference frame and tuning constants.

Everything that describes "what the swarm is" lives here; the node logic in
swarm_node.py is generic and reads only this file.
"""

import math
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
class OrbitSpec:
    """
    A dynamic formation entry: the offset traces a circle over time instead of
    sitting still. fly_follower() re-evaluates this every publish tick using
    orbit_state(spec, t) from formation.py, then feeds it through
    formation_target() exactly like a static (fwd, right, down) tuple.
    """
    radius: float   # m, distance from the parent's current position
    omega: float    # rad/s, angular velocity; sign sets direction (+ = CCW)
    phase0: float   # rad, angle at t=0 -- t is seconds since the swarm
                    # switched INTO this formation (see fly_follower's
                    # orbit_t0), not wall-clock time, so re-entering an orbit
                    # formation always starts from this same layout angle
                    # instead of resuming a stale one.
    down: float     # m, fixed altitude offset from the parent (no vertical
                    # oscillation -- only fwd/right revolve)


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


def root_of(node_id):
    """Walk SWARM's follow-tree up to the ultimate leader (parent is None)."""
    n = SWARM[node_id]
    while n.parent is not None:
        n = SWARM[n.parent]
    return n.node_id


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

# ------------------------------------------------------------------- formations
# Each formation is a { node_id: offset } map, offset in the SAME (fwd, right,
# down) parent-body-frame convention as NodeCfg.offset above -- it is applied
# to the same follow tree (SWARM's level/parent columns don't change), so
# switching formation only ever changes where a node sits relative to its
# existing parent, never who its parent is.
#
# NodeCfg.offset holds "wedge" (the shape the swarm boots into); FORMATIONS
# lets a running swarm switch shape via swarm_cli.py's FORMATION command --
# see leader_commands.py and fly_follower() in swarm_node.py.
FORMATIONS = {
    "wedge": {nid: n.offset for nid, n in SWARM.items()},
    # single file behind the leader: 1,2 tuck directly in behind 0, and their
    # children (3, 4) tuck in behind them in turn -- the chain composes into
    # one straight line along the leader's forward axis.
    "line-vertical": {
        0: (0.0, 0.0, 0.0),
        1: (-8.0, 0.0, -2.0),
        2: (-16.0, 0.0, -2.0),
        3: (-16.0, 0.0, 0.0),
        4: (-16.0, 0.0, 0.0),
    },
    # abreast of the leader, spread along its right axis: 1,2 sit either side
    # of 0 and 3,4 extend the line further out past 1,2.
    "line-horizontal": {
        0: (0.0, 0.0, 0.0),
        1: (-8.0, -8.0, -2.0),
        2: (-8.0, +8.0, -2.0),
        3: (0.0, -8.0, 0.0),
        4: (0.0, +8.0, 0.0),
    },
    # sawtooth chain with the leader at the center low point: 1,2 sit ahead
    # and out to either side, and 3,4 continue the zigzag back down past
    # them -- so along the right axis the chain reads low(3) - high(1) -
    # low(0) - high(2) - low(4), same alternating shape as line-horizontal
    # but with height (forward offset) zigzagging instead of a flat line.
    "zigzag": {
        0: (0.0, 0.0, 0.0),
        1: (8.0, -8.0, -2.0),
        2: (8.0, +8.0, -2.0),
        3: (-8.0, -8.0, 0.0),
        4: (-8.0, +8.0, 0.0),
    },
    # Orbital revolution: the leader stays put at the center and 1,2 sweep
    # around it in a circle. 3,4 use OrbitSpec entries too, but their parent
    # is 1/2 (not 0) per SWARM's fixed follow-tree, so they orbit their own
    # parent instead of the leader directly -- moons revolving around a
    # planet that is itself revolving around the leader, rather than a flat
    # ring of 4. phase0 = 0/pi puts each pair on opposite sides of its parent
    # so they don't collide.
    #
    # ORBIT_OMEGA is picked so the fastest tangential speed (radius * omega)
    # stays well under V_MAX = 8 m/s -- at radius 8 m this is 8 * 0.15 = 1.2
    # m/s, a slow, visually clear revolution (~42 s per lap).
    "orbit": {
        0: (0.0, 0.0, 0.0),
        1: OrbitSpec(radius=8.0, omega=0.15, phase0=0.0, down=-2.0),
        2: OrbitSpec(radius=8.0, omega=0.15, phase0=math.pi, down=-2.0),
        3: OrbitSpec(radius=8.0, omega=0.15, phase0=0.0, down=0.0),
        4: OrbitSpec(radius=8.0, omega=0.15, phase0=math.pi, down=0.0),
    },
    # Single shared ring: unlike "orbit", every follower circles the SAME
    # center (the leader) at the SAME radius and angular velocity, evenly
    # spaced by phase -- a rigid rotation, so not only is each follower's
    # distance to the leader constant, every pair of followers' distance to
    # EACH OTHER is constant too (the whole ring turns as one piece). This
    # needs the leader as the orbit center regardless of tree depth, so
    # fly_follower() looks these entries' center up via FORMATION_CENTER
    # below instead of the node's immediate SWARM parent.
    #
    # radius 10 m (vs. 8 m for "orbit", just to look visually distinct);
    # omega unchanged at 0.15 rad/s -> 1.5 m/s tangential, still well under
    # V_MAX. phase0 = 2*pi*i/4 for the 4 followers spaces them 90 degrees
    # apart around the circle.
    "ring": {
        0: (0.0, 0.0, 0.0),
        1: OrbitSpec(radius=10.0, omega=0.15, phase0=0.0, down=-3.0),
        2: OrbitSpec(radius=10.0, omega=0.15, phase0=math.pi / 2, down=-3.0),
        3: OrbitSpec(radius=10.0, omega=0.15, phase0=math.pi, down=-3.0),
        4: OrbitSpec(radius=10.0, omega=0.15, phase0=3 * math.pi / 2, down=-3.0),
    },
}
DEFAULT_FORMATION = "wedge"

# Which peer's telemetry a formation's offsets are measured from. Every
# static formation and "orbit" is measured from the node's own SWARM parent
# (the default -- omitted here). "ring" is the one exception: it needs every
# follower on the SAME circle around the swarm leader regardless of tree
# depth, so its offsets are measured from the root instead of the immediate
# parent. See fly_follower()'s use of root_of() for how this is applied.
FORMATION_CENTER = {
    "ring": "root",
}

# Leader path, in the common swarm frame: (north, east, down) [m].
LEADER_PATH = [
    (0.0, 0.0, -20.0),
    (80.0, 0.0, -20.0),
    (80.0, 80.0, -25.0),
    (0.0, 80.0, -25.0),
    (0.0, 0.0, -20.0),
]
