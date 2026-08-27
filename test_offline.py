#!/usr/bin/env python3
"""
Offline checks that need neither PX4 nor MAVSDK.

    python3 test_offline.py

Covers the two things most likely to be silently wrong: the local<->swarm frame
transform between vehicles with different EKF origins, and the yaw rotation of
the formation offset.
"""

import asyncio
import math

import swarm_config as cfg
from formation import clamp_xyz, formation_target, local_to_swarm_offset
from geo import geodetic_to_ned, ned_to_geodetic
from swarm_link import open_link


def check(name, ok):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    assert ok, name


def test_geo_roundtrip():
    print("geodetic round-trip")
    n, e, d = 137.0, -84.0, -22.5
    lat, lon, alt = ned_to_geodetic(n, e, d, cfg.ORIGIN_LAT, cfg.ORIGIN_LON, cfg.ORIGIN_ALT)
    n2, e2, d2 = geodetic_to_ned(lat, lon, alt, cfg.ORIGIN_LAT, cfg.ORIGIN_LON, cfg.ORIGIN_ALT)
    check("ned -> geodetic -> ned within 1 mm",
          max(abs(n - n2), abs(e - e2), abs(d - d2)) < 1e-3)


def test_frame_transform():
    """
    Two vehicles, different EKF origins, physically at the same point.
    Their swarm-frame coordinates must agree even though their local NED does not.
    """
    print("swarm-frame transform across different EKF origins")

    # Vehicle A booted at the swarm origin, B booted 8 m east and 3 m higher.
    a_home = (cfg.ORIGIN_LAT, cfg.ORIGIN_LON, cfg.ORIGIN_ALT)
    b_home = ned_to_geodetic(0.0, 8.0, -3.0, *a_home)

    # Both hover at the same physical point: 50 m north, 20 m east, 30 m up.
    truth = (50.0, 20.0, -30.0)
    p_lat, p_lon, p_alt = ned_to_geodetic(*truth, *a_home)

    a_local = geodetic_to_ned(p_lat, p_lon, p_alt, *a_home)
    b_local = geodetic_to_ned(p_lat, p_lon, p_alt, *b_home)
    check("local frames genuinely differ", abs(a_local[1] - b_local[1]) > 7.0)

    tf_a = local_to_swarm_offset(p_lat, p_lon, p_alt, *a_local)
    tf_b = local_to_swarm_offset(p_lat, p_lon, p_alt, *b_local)

    a_swarm = tuple(a_local[i] + tf_a[i] for i in range(3))
    b_swarm = tuple(b_local[i] + tf_b[i] for i in range(3))

    check("A agrees with ground truth", math.dist(a_swarm, truth) < 0.01)
    check("B agrees with ground truth", math.dist(b_swarm, truth) < 0.01)
    check("A and B agree with each other", math.dist(a_swarm, b_swarm) < 0.01)


def test_formation_rotation():
    print("formation offset rotates with parent yaw")
    offset = (-8.0, -8.0, -2.0)   # 8 m behind, 8 m to the left, 2 m above

    parent = {"n": 0.0, "e": 0.0, "d": -20.0, "yaw": 0.0}
    n, e, d = formation_target(parent, offset)
    check("heading north -> target south-west",
          abs(n + 8.0) < 1e-6 and abs(e + 8.0) < 1e-6 and abs(d + 22.0) < 1e-6)

    parent["yaw"] = 90.0          # heading east
    n, e, d = formation_target(parent, offset)
    check("heading east -> target north-west",
          abs(n - 8.0) < 1e-6 and abs(e + 8.0) < 1e-6)

    parent["yaw"] = 180.0         # heading south
    n, e, d = formation_target(parent, offset)
    check("heading south -> target north-east",
          abs(n - 8.0) < 1e-6 and abs(e - 8.0) < 1e-6)

    # Distance to the parent must be invariant under rotation.
    dists = []
    for yaw in range(0, 360, 15):
        parent["yaw"] = float(yaw)
        n, e, d = formation_target(parent, offset)
        dists.append(math.hypot(n - parent["n"], e - parent["e"]))
    check("stand-off distance invariant", max(dists) - min(dists) < 1e-9)


def test_clamp():
    print("velocity clamp")
    x, y, z = clamp_xyz(30.0, 40.0, 0.0, 5.0)
    check("long vector scaled to the limit", abs(math.sqrt(x*x + y*y + z*z) - 5.0) < 1e-9)
    check("direction preserved", abs(x / y - 30.0 / 40.0) < 1e-9)
    x, y, z = clamp_xyz(1.0, 2.0, 2.0, 5.0)
    check("short vector untouched", (x, y, z) == (1.0, 2.0, 2.0))
    check("zero vector safe", clamp_xyz(0.0, 0.0, 0.0, 5.0) == (0.0, 0.0, 0.0))


def test_topology():
    print("topology sanity")
    roots = [n for n in cfg.SWARM.values() if n.parent is None]
    check("exactly one leader", len(roots) == 1)
    check("every parent exists",
          all(n.parent in cfg.SWARM for n in cfg.SWARM.values() if n.parent is not None))
    check("levels increase towards the leaves",
          all(cfg.SWARM[n.parent].level == n.level - 1
              for n in cfg.SWARM.values() if n.parent is not None))
    ports = [n.px4_instance for n in cfg.SWARM.values()]
    check("unique PX4 instances", len(set(ports)) == len(ports))
    # walk to the root from every node -- catches accidental cycles
    for n in cfg.SWARM.values():
        seen, cur = set(), n
        while cur.parent is not None:
            check("no cycle", cur.node_id not in seen)
            seen.add(cur.node_id)
            cur = cfg.SWARM[cur.parent]


async def test_link():
    print("UDP broadcast link")
    a = await open_link(101)
    b = await open_link(102)
    a.publish({"n": 1.0, "e": 2.0, "d": -3.0, "st": "form"})
    await asyncio.sleep(0.3)

    got = b.peer(101)
    if got is None:
        print("  SKIP  no loopback broadcast in this environment "
              f"(check BCAST_ADDR={cfg.BCAST_ADDR} and any firewall)")
        return
    check("payload delivered intact", got["n"] == 1.0 and got["st"] == "form")
    check("sender does not hear itself", a.peer(101) is None)
    check("stale peers are dropped", b.peer(101, max_age=0.0) is None)


if __name__ == "__main__":
    test_geo_roundtrip()
    test_frame_transform()
    test_formation_rotation()
    test_clamp()
    test_topology()
    asyncio.run(test_link())
    print("\nall checks passed")
