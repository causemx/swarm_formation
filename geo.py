"""
Flat-earth geodetic <-> NED conversion.

Equirectangular approximation. Error is well under a metre for the few-hundred
metre neighbourhoods a swarm demo operates in, which is all we need to reconcile
the different EKF origins of the vehicles.
"""

import math

R_EARTH = 6371000.0


def geodetic_to_ned(lat, lon, alt, lat0, lon0, alt0):
    """(lat, lon, alt_amsl) -> (north, east, down) relative to the origin."""
    n = math.radians(lat - lat0) * R_EARTH
    e = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    d = -(alt - alt0)
    return n, e, d


def ned_to_geodetic(n, e, d, lat0, lon0, alt0):
    """(north, east, down) -> (lat, lon, alt_amsl)."""
    lat = lat0 + math.degrees(n / R_EARTH)
    lon = lon0 + math.degrees(e / (R_EARTH * math.cos(math.radians(lat0))))
    alt = alt0 - d
    return lat, lon, alt
