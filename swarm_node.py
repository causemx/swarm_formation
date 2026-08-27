#!/usr/bin/env python3
"""
One process per drone.

    python3 swarm_node.py --id 0     # leader
    python3 swarm_node.py --id 1     # follower of 0
    ...

Role is derived from swarm_config.SWARM: a node with parent=None flies the
LEADER_PATH, everything else tracks its parent's broadcast state. Because a
follower also broadcasts, it can itself be someone's parent -- that is the whole
of the "hierarchy".

Phases are propagated down the tree through the "st" field: when the leader
switches to "land", its children see it, land, and publish "land" in turn.
"""

import argparse
import asyncio
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

import swarm_config as cfg
from formation import clamp_xyz, formation_target, local_to_swarm_offset
from leader_commands import run_leader_commands
from swarm_link import open_link


# ============================================================== telemetry cache
class Vehicle:
    """Latest telemetry, kept fresh by background tasks."""

    def __init__(self):
        self.lat = None
        self.lon = None
        self.alt = None                       # AMSL
        self.pn = self.pe = self.pd = None    # own EKF-local NED
        self.vn = self.ve = self.vd = 0.0
        self.yaw = 0.0

    @property
    def ready(self):
        return self.lat is not None and self.pn is not None


async def _t_position(drone, v):
    async for p in drone.telemetry.position():
        v.lat, v.lon, v.alt = p.latitude_deg, p.longitude_deg, p.absolute_altitude_m


async def _t_pv_ned(drone, v):
    async for s in drone.telemetry.position_velocity_ned():
        v.pn, v.pe, v.pd = s.position.north_m, s.position.east_m, s.position.down_m
        v.vn, v.ve, v.vd = (s.velocity.north_m_s,
                            s.velocity.east_m_s,
                            s.velocity.down_m_s)


async def _t_attitude(drone, v):
    async for a in drone.telemetry.attitude_euler():
        v.yaw = a.yaw_deg


# =================================================================== broadcast
async def publisher(link, v, node, tf, phase):
    dt = 1.0 / cfg.PUBLISH_HZ
    while True:
        if v.ready:
            link.publish({
                "t": time.time(),
                "lvl": node.level,
                "st": phase[0],
                "n": v.pn + tf[0],
                "e": v.pe + tf[1],
                "d": v.pd + tf[2],
                "vn": v.vn, "ve": v.ve, "vd": v.vd,
                "yaw": v.yaw,
            })
        await asyncio.sleep(dt)


# ================================================================== behaviours
async def fly_leader(drone, v, tf, phase, log):
    dt = 1.0 / cfg.PUBLISH_HZ
    for i, (tn, te, td) in enumerate(cfg.LEADER_PATH):
        # heading along the leg
        dn, de = tn - (v.pn + tf[0]), te - (v.pe + tf[1])
        yaw = math.degrees(math.atan2(de, dn)) if math.hypot(dn, de) > 1.0 else v.yaw

        log(f"leg {i + 1}/{len(cfg.LEADER_PATH)} -> N{tn:.0f} E{te:.0f} D{td:.0f}")
        ln, le, ld = tn - tf[0], te - tf[1], td - tf[2]   # swarm -> own local

        deadline = time.monotonic() + cfg.LEG_TIMEOUT
        while True:
            err = math.sqrt((ln - v.pn) ** 2 + (le - v.pe) ** 2 + (ld - v.pd) ** 2)
            if err < cfg.ARRIVAL_R:
                break
            if time.monotonic() > deadline:
                log(f"leg timed out {err:.1f} m short, moving on")
                break
            # Feed forward cruise speed along the remaining vector. clamp_xyz
            # also gives a free taper: inside LEADER_SPEED metres of the
            # waypoint the command shrinks with the distance.
            fn, fe, fd = clamp_xyz(ln - v.pn, le - v.pe, ld - v.pd, cfg.LEADER_SPEED)
            await drone.offboard.set_position_velocity_ned(
                PositionNedYaw(ln, le, ld, yaw),
                VelocityNedYaw(fn, fe, fd, yaw))
            await asyncio.sleep(dt)

    log("path complete, commanding swarm to land")
    phase[0] = "land"
    # keep the last setpoint alive long enough for the children to hear "land"
    for _ in range(int(3.0 * cfg.PUBLISH_HZ)):
        await drone.offboard.set_position_ned(
            PositionNedYaw(v.pn, v.pe, v.pd, v.yaw))
        await asyncio.sleep(dt)


async def fly_follower(drone, v, link, node, tf, phase, log):
    dt = 1.0 / cfg.PUBLISH_HZ
    hold = None
    warned = False

    while True:
        parent = link.peer(node.parent)

        if parent is None:
            # No parent state: freeze in place. (Nothing more clever here --
            # this build has no fault tolerance by design.)
            if hold is None:
                hold = (v.pn, v.pe, v.pd, v.yaw)
                if not warned:
                    log(f"no state from node {node.parent}, holding")
                    warned = True
            await drone.offboard.set_position_ned(PositionNedYaw(*hold))
            await asyncio.sleep(dt)
            continue

        if hold is not None:
            log(f"node {node.parent} reacquired")
            hold, warned = None, False

        if parent.get("st") == "land":
            log("parent is landing, following")
            phase[0] = "land"
            return

        tn, te, td = formation_target(parent, node.offset)
        ln, le, ld = tn - tf[0], te - tf[1], td - tf[2]   # swarm -> own local
        yaw = parent["yaw"]

        if cfg.SETPOINT_MODE == "vel":
            vn = cfg.KP_POS * (ln - v.pn) + cfg.FF_GAIN * parent["vn"]
            ve = cfg.KP_POS * (le - v.pe) + cfg.FF_GAIN * parent["ve"]
            vd = cfg.KP_POS * (ld - v.pd) + cfg.FF_GAIN * parent["vd"]
            vn, ve, vd = clamp_xyz(vn, ve, vd, cfg.V_MAX)
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw))
        else:
            await drone.offboard.set_position_velocity_ned(
                PositionNedYaw(ln, le, ld, yaw),
                VelocityNedYaw(parent["vn"] * cfg.FF_GAIN,
                               parent["ve"] * cfg.FF_GAIN,
                               parent["vd"] * cfg.FF_GAIN,
                               yaw))

        await asyncio.sleep(dt)


# ======================================================================= main
async def run(node, command_interface=False):
    def log(msg):
        print(f"[node {node.node_id} lvl{node.level}] {msg}", flush=True)

    link = await open_link(node.node_id)
    log(f"broadcasting on {cfg.BCAST_ADDR}:{cfg.UDP_PORT}")

    drone = System(port=node.grpc_port)
    await drone.connect(system_address=node.mavsdk_url)
    log(f"waiting for PX4 on {node.mavsdk_url}")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    log("connected")

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    log("position estimate ok")

    for setter, rate in ((drone.telemetry.set_rate_position, 5.0),
                         (drone.telemetry.set_rate_position_velocity_ned, cfg.PUBLISH_HZ),
                         (drone.telemetry.set_rate_attitude_euler, cfg.PUBLISH_HZ)):
        try:
            await setter(rate)
        except Exception as exc:            # rate setting is best-effort
            log(f"rate setup skipped: {exc}")

    v = Vehicle()
    bg = [asyncio.create_task(f(drone, v))
          for f in (_t_position, _t_pv_ned, _t_attitude)]
    while not v.ready:
        await asyncio.sleep(0.1)

    tf = local_to_swarm_offset(v.lat, v.lon, v.alt, v.pn, v.pe, v.pd)
    log(f"local->swarm offset N{tf[0]:+.1f} E{tf[1]:+.1f} D{tf[2]:+.1f}")

    phase = ["climb"]
    bg.append(asyncio.create_task(publisher(link, v, node, tf, phase)))

    if node.parent is None and command_interface:
        # Operator drives arm/takeoff/goto/hold/land over leader_commands.py's
        # command link instead of the fixed climb-then-LEADER_PATH sequence
        # below. See run_leader_commands() for the state machine.
        phase[0] = "form"
        try:
            await run_leader_commands(drone, v, tf, phase, log)
        finally:
            phase[0] = "land"
            log("landing")
            try:
                await drone.offboard.stop()
            except OffboardError:
                pass
            try:
                await drone.action.land()
            except Exception as exc:      # never armed / already landed, etc.
                log(f"land skipped: {exc}")
            await asyncio.sleep(15.0)
            for t in bg:
                t.cancel()
        return

    # ------------------------------------------------------------ arm + climb
    ground_d = v.pd
    await drone.action.arm()
    await drone.offboard.set_position_ned(PositionNedYaw(v.pn, v.pe, v.pd, v.yaw))
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        log(f"offboard start failed: {exc._result.result}")
        await drone.action.disarm()
        return

    hover_d = ground_d - cfg.TAKEOFF_ALT
    log(f"climbing to {cfg.TAKEOFF_ALT:.0f} m AGL")
    n0, e0, yaw0 = v.pn, v.pe, v.yaw
    dt = 1.0 / cfg.PUBLISH_HZ
    deadline = time.monotonic() + 30.0
    while abs(v.pd - hover_d) > 0.6 and time.monotonic() < deadline:
        await drone.offboard.set_position_ned(PositionNedYaw(n0, e0, hover_d, yaw0))
        await asyncio.sleep(dt)

    # Let the tree come up top-down so a follower has parent state on its first
    # formation cycle.
    await asyncio.sleep(2.0 * node.level + 2.0)

    # -------------------------------------------------------------- behaviour
    phase[0] = "form"
    try:
        if node.parent is None:
            await fly_leader(drone, v, tf, phase, log)
        else:
            await fly_follower(drone, v, link, node, tf, phase, log)
    finally:
        phase[0] = "land"
        log("landing")
        try:
            await drone.offboard.stop()
        except OffboardError:
            pass
        await drone.action.land()
        await asyncio.sleep(15.0)
        for t in bg:
            t.cancel()


def main():
    ap = argparse.ArgumentParser(description="Hierarchical leader-follower swarm node")
    ap.add_argument("--id", type=int, required=True, help="node id from swarm_config.SWARM")
    ap.add_argument("--command-interface", action="store_true",
                     help="leader only: wait for ARM/TAKEOFF/GOTO/HOLD/LAND "
                          "from swarm_cli.py instead of flying the fixed "
                          "cfg.LEADER_PATH square. Ignored for followers, "
                          "who always track their parent regardless.")
    args = ap.parse_args()

    if args.id not in cfg.SWARM:
        raise SystemExit(f"unknown node id {args.id}; known: {sorted(cfg.SWARM)}")

    try:
        asyncio.run(run(cfg.SWARM[args.id], command_interface=args.command_interface))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
