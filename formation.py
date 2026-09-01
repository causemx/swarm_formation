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


def _rotate(psi, fwd, right):
    """Body-frame (fwd, right) -> swarm-frame (n, e) at heading psi (rad)."""
    n = fwd * math.cos(psi) - right * math.sin(psi)
    e = fwd * math.sin(psi) + right * math.cos(psi)
    return n, e


def formation_target(parent, offset):
    """
    Parent state + body-frame offset -> desired point in the swarm frame.

    The offset is rotated by the parent's yaw, so the formation turns with the
    parent instead of staying locked to compass north.
    """
    fwd, right, down = offset
    dn, de = _rotate(math.radians(parent["yaw"]), fwd, right)
    n = parent["n"] + dn
    e = parent["e"] + de
    d = parent["d"] + down
    return n, e, d


def orbit_state(spec, t):
    """
    OrbitSpec + elapsed time -> (fwd, right, down, vfwd, vright).

    fwd/right/down feed into formation_target() exactly like a static offset;
    vfwd/vright are the tangential velocity of that circular motion in the
    SAME body frame, for the caller to rotate and add to the parent's
    velocity feed-forward (formation_target has no velocity output, so this
    is kept separate rather than overloading it).
    """
    theta = spec.phase0 + spec.omega * t
    fwd = spec.radius * math.cos(theta)
    right = spec.radius * math.sin(theta)
    vfwd = -spec.radius * spec.omega * math.sin(theta)
    vright = spec.radius * spec.omega * math.cos(theta)
    return fwd, right, spec.down, vfwd, vright


def orbit_velocity_ned(parent_yaw_deg, vfwd, vright):
    """Rotate an orbit's tangential body-frame velocity into the swarm frame."""
    return _rotate(math.radians(parent_yaw_deg), vfwd, vright)


def clamp_xyz(x, y, z, limit):
    """Scale a vector down to `limit` if it is longer, leave it alone otherwise."""
    m = math.sqrt(x * x + y * y + z * z)
    if m <= limit or m == 0.0:
        return x, y, z
    k = limit / m
    return x * k, y * k, z * k
