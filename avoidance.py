"""
ORCA (Optimal Reciprocal Collision Avoidance) velocity correction.

fly_follower() engages this only for cfg.AVOID_WINDOW seconds after a
formation switch (see cfg.AVOID_WINDOW / fm_t0) -- the moment offsets jump to
a new shape and followers' straight-line paths to their new targets are most
likely to cross. Outside that window it isn't called at all.

Wraps Python-RVO2 (`import rvo2`, github.com/sybrenstuvel/Python-RVO2 -- see
README.md's Prerequisites; it's a native extension built from C++, not a
plain PyPI package). RVO2 is 2D: this only ever governs the horizontal (n, e)
velocity, never d (altitude) -- vertical separation stays whatever the
formation offset already gives it.
"""

import math

try:
    import rvo2
except ImportError:     # the native extension may not be built everywhere
    print("Python-RVO2 has not installed.")
    rvo2 = None          # this runs -- degrade to "no avoidance" rather than crash


def _clamp(n, e, max_speed):
    m = math.hypot(n, e)
    if m <= max_speed or m == 0.0:
        return n, e
    k = max_speed / m
    return n * k, e * k


def orca_velocity(self_n, self_e, self_vn, self_ve, neighbors,
                   pref_n, pref_e, radius, max_speed,
                   neighbor_dist, time_horizon):
    """
    Collision-free (n, e) velocity for one agent, given nearby swarm telemetry.

    neighbors: iterable of {"n", "e", "vn", "ve"} dicts, SWARM frame, as
    broadcast -- see swarm_link.SwarmLink.all_peers(). A fresh RVO2 simulator
    is built every call: at PUBLISH_HZ=20 and a handful of agents this is
    cheap, and it avoids the bookkeeping of keeping a persistent simulator's
    agent list in sync with who is currently in range.

    Falls back to the (speed-clamped) preferred velocity unchanged if rvo2
    isn't importable, there are no neighbors to avoid, or RVO2 itself errors
    -- this is a safety layer on top of normal formation tracking, not a
    replacement for it, so it must never be the reason a node's control loop
    dies.
    """
    pref_n, pref_e = _clamp(pref_n, pref_e, max_speed)
    neighbors = list(neighbors)
    if rvo2 is None or not neighbors:
        return pref_n, pref_e

    try:
        max_neighbors = len(neighbors) + 1
        dt = 1.0 / 20.0   # only paces RVO2's own ORCA math, unrelated to the
                           # caller's publish rate -- see cfg.PUBLISH_HZ
        sim = rvo2.PyRVOSimulator(dt, neighbor_dist, max_neighbors,
                                   time_horizon, time_horizon, radius, max_speed)
        self_agent = sim.addAgent((self_n, self_e), neighbor_dist, max_neighbors,
                                   time_horizon, time_horizon, radius, max_speed,
                                   (self_vn, self_ve))
        for nb in neighbors:
            sim.addAgent((nb["n"], nb["e"]), neighbor_dist, max_neighbors,
                          time_horizon, time_horizon, radius, max_speed,
                          (nb.get("vn", 0.0), nb.get("ve", 0.0)))
        sim.setAgentPrefVelocity(self_agent, (pref_n, pref_e))
        sim.doStep()
        return sim.getAgentVelocity(self_agent)
    except Exception:
        return pref_n, pref_e
