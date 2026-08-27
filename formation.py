"""
Formation geometry. Pure math, no MAVSDK -- so it can be imported by the
monitor, by unit tests, or by an offline analysis script.
"""

import math

import swarm_config as cfg
from geo import geodetic_to_ned


def local_to_swarm_offset(lat, lon, alt, pn, pe, pd):
    """
    Constant offset T such that  swarm_ned = local_ned + T.

    Computed once from a simultaneous (GPS fix, local NED) pair: the EKF origin
    does not move, so T is fixed. Recomputing it every cycle would just inject
    GPS noise straight into the formation.
    """
    cn, ce, cd = geodetic_to_ned(lat, lon, alt,
                                 cfg.ORIGIN_LAT, cfg.ORIGIN_LON, cfg.ORIGIN_ALT)
    return cn - pn, ce - pe, cd - pd


def formation_target(parent, offset):
    """
    Parent state + body-frame offset -> desired point in the swarm frame.

    The offset is rotated by the parent's yaw, so the formation turns with the
    parent instead of staying locked to compass north.
    """
    psi = math.radians(parent["yaw"])
    fwd, right, down = offset
    n = parent["n"] + fwd * math.cos(psi) - right * math.sin(psi)
    e = parent["e"] + fwd * math.sin(psi) + right * math.cos(psi)
    d = parent["d"] + down
    return n, e, d


def clamp_xyz(x, y, z, limit):
    """Scale a vector down to `limit` if it is longer, leave it alone otherwise."""
    m = math.sqrt(x * x + y * y + z * z)
    if m <= limit or m == 0.0:
        return x, y, z
    k = limit / m
    return x * k, y * k, z * k
