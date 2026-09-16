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

import time

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
        An entry with an "rx_t" field (SwarmLink's local receive stamp) is
        dead-reckoned forward by its age using its own vn/ve; entries without
        one (e.g. hand-built peer dicts in tests) are treated as fresh.
    self_id: this node's id, so its own broadcast (if present in peers) is
        excluded rather than double-counted as a neighbor of itself.

    Returns (vn, ve): the ORCA-adjusted velocity for this agent only. With no
    peers, this is exactly self_pref_vel unchanged.
    """
    sim = rvo2.PyRVOSimulator(dt, neighbor_dist, max_neighbors,
                               time_horizon, time_horizon_obst, radius, max_speed)
    a_self = sim.addAgent(tuple(self_pos))
    sim.setAgentPrefVelocity(a_self, tuple(self_pref_vel))

    now = time.monotonic()
    for pid, p in peers.items():
        if pid == self_id:
            continue
        # A peer's true preferred velocity is unknown to us; RVO2 uses the
        # velocity passed to addAgent as that agent's prefVelocity unless
        # setAgentPrefVelocity is called on it, so this assumes each neighbor
        # intends to keep going the way it currently is.
        vn, ve = p.get("vn", 0.0), p.get("ve", 0.0)
        # peers is a cache (SwarmLink.all_peers()) and can be legitimately up
        # to PEER_TIMEOUT old -- at ORCA_MAX_SPEED that's tens of metres of
        # drift between the cached (n, e) and the peer's real position, which
        # is exactly the regime a "goto" leg puts neighbors in (near-zero
        # velocity at rest never accrues enough drift to matter). rx_t is
        # SwarmLink's local receive stamp, so dead-reckon the peer forward by
        # its actual age instead of trusting the stale snapshot outright.
        age = max(0.0, now - p["rx_t"]) if "rx_t" in p else 0.0
        pn, pe = p["n"] + vn * age, p["e"] + ve * age
        sim.addAgent((pn, pe), neighbor_dist, max_neighbors,
                     time_horizon, time_horizon_obst, radius, max_speed,
                     (vn, ve))

    sim.doStep()
    return sim.getAgentVelocity(a_self)
