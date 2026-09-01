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
from formation import clamp_xyz, formation_target, local_to_swarm_offset, orbit_state
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


def test_orbit_formation():
    print("orbit formation geometry")
    spec = cfg.OrbitSpec(radius=8.0, omega=0.15, phase0=0.0, down=-2.0)
    parent = {"n": 0.0, "e": 0.0, "d": -20.0, "yaw": 0.0}

    radii, angles = [], []
    for i in range(200):
        t = i * 0.5
        fwd, right, down, vfwd, vright = orbit_state(spec, t)
        n, e, d = formation_target(parent, (fwd, right, down))
        radii.append(math.hypot(n - parent["n"], e - parent["e"]))
        angles.append(math.atan2(right, fwd))
        check("altitude offset stays fixed", abs(d - (parent["d"] + spec.down)) < 1e-9)
        # velocity must be tangential (perpendicular to the radius vector)
        # and consistent with the sign of omega, or the follower would be
        # asked to fly a spiral instead of a circle.
        check("velocity perpendicular to radius",
              abs(fwd * vfwd + right * vright) < 1e-9)
        check("velocity direction matches omega sign",
              (vfwd * -right + vright * fwd) * spec.omega >= -1e-9)

    check("radius constant over time", max(radii) - min(radii) < 1e-9)

    # phase0 offset between two nodes must be preserved as they both revolve.
    spec2 = cfg.OrbitSpec(radius=8.0, omega=0.15, phase0=math.pi, down=-2.0)
    for t in (0.0, 5.0, 37.0):
        f1, r1, _, _, _ = orbit_state(spec, t)
        f2, r2, _, _, _ = orbit_state(spec2, t)
        check("opposite-phase node stays diametrically opposite",
              abs((f1 + f2)) < 1e-9 and abs(r1 + r2) < 1e-9)


def test_ring_formation():
    print("ring formation geometry")
    leader = {"n": 0.0, "e": 0.0, "d": -20.0, "yaw": 0.0}
    followers = sorted(nid for nid, n in cfg.SWARM.items() if n.parent is not None)
    check("ring covers every follower", set(cfg.FORMATIONS["ring"]) - {0} == set(followers))

    def positions(t):
        pts = {}
        for nid in followers:
            spec = cfg.FORMATIONS["ring"][nid]
            fwd, right, down, _, _ = orbit_state(spec, t)
            pts[nid] = formation_target(leader, (fwd, right, down))
        return pts

    p0 = positions(0.0)
    to_leader = {nid: math.dist(p0[nid], (leader["n"], leader["e"], leader["d"]))
                 for nid in followers}
    check("every follower is the same distance from the leader",
          max(to_leader.values()) - min(to_leader.values()) < 1e-9)

    pair_dist_t0 = {(a, b): math.dist(p0[a], p0[b])
                    for i, a in enumerate(followers) for b in followers[i + 1:]}

    for t in (1.0, 13.0, 40.0, 97.0):
        pt = positions(t)
        for nid in followers:
            check(f"node {nid} distance to leader constant over time",
                  abs(math.dist(pt[nid], (leader["n"], leader["e"], leader["d"]))
                      - to_leader[nid]) < 1e-9)
        for pair, d0 in pair_dist_t0.items():
            a, b = pair
            check(f"pair {pair} distance to EACH OTHER constant over time (rigid ring)",
                  abs(math.dist(pt[a], pt[b]) - d0) < 1e-9)


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
    root = [n.node_id for n in cfg.SWARM.values() if n.parent is None][0]
    check("root_of() finds the same leader for every node",
          all(cfg.root_of(n.node_id) == root for n in cfg.SWARM.values()))
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
    test_orbit_formation()
    test_ring_formation()
    test_clamp()
    test_topology()
    asyncio.run(test_link())
    print("\nall checks passed")
