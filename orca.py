"""
Decentralized ORCA collision avoidance for SETPOINT_MODE == "vel" followers.

Pure math, no MAVSDK -- same convention as formation.py, so it's importable
by test_offline.py without a running SITL instance. Each node runs its own
rvo2.PyRVOSimulator locally, every tick: there is no shared simulator state
across processes, only the neighbor telemetry already on the wire via
SwarmLink.all_peers(). The simulator is rebuilt fresh on every call rather
than persisted -- membership and every agent's position change every tick
anyway (peers go stale, formations change), so a persistent sim with stable
agent indices would need the same rebuild-on-change bookkeeping for no real
savings at swarm sizes this small.
"""

import rvo2

import swarm_config as cfg


def orca_velocity(self_pos, self_pref_vel, peers, self_id, *,
                   dt=1.0 / cfg.PUBLISH_HZ,
                   radius=cfg.ORCA_RADIUS,
                   neighbor_dist=cfg.ORCA_NEIGHBOR_DIST,
                   max_neighbors=cfg.ORCA_MAX_NEIGHBORS,
                   time_horizon=cfg.ORCA_TIME_HORIZON,
                   time_horizon_obst=cfg.ORCA_TIME_HORIZON_OBST,
                   max_speed=cfg.ORCA_MAX_SPEED):
    """
    self_pos, self_pref_vel: (n, e) in the swarm frame -- must be the same
        frame peers' n/e are broadcast in, not a node's own local NED.
    peers: a dict as returned by SwarmLink.all_peers(), {id: {n, e, vn, ve, ...}}.
        May or may not include an entry keyed self_id; either way it's skipped.
    self_id: this node's id, so its own broadcast (if present in peers) is
        excluded rather than double-counted as a neighbor of itself.

    Returns (vn, ve): the ORCA-adjusted velocity for this agent only. With no
    peers, this is exactly self_pref_vel unchanged.
    """
    sim = rvo2.PyRVOSimulator(dt, neighbor_dist, max_neighbors,
                               time_horizon, time_horizon_obst, radius, max_speed)
    a_self = sim.addAgent(tuple(self_pos))
    sim.setAgentPrefVelocity(a_self, tuple(self_pref_vel))

    for pid, p in peers.items():
        if pid == self_id:
            continue
        # A peer's true preferred velocity is unknown to us; RVO2 uses the
        # velocity passed to addAgent as that agent's prefVelocity unless
        # setAgentPrefVelocity is called on it, so this assumes each neighbor
        # intends to keep going the way it currently is.
        sim.addAgent((p["n"], p["e"]), neighbor_dist, max_neighbors,
                     time_horizon, time_horizon_obst, radius, max_speed,
                     (p.get("vn", 0.0), p.get("ve", 0.0)))

    sim.doStep()
    return sim.getAgentVelocity(a_self)
