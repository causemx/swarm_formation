"""
Command channel for the swarm leader: lets an external operator (swarm_cli.py)
issue ARM / TAKEOFF / GOTO / HOLD / LAND to node 0 while it runs, instead of
node 0 only ever flying the fixed cfg.LEADER_PATH square. Only used when
swarm_node.py is started with --command-interface; the scripted-demo path
(fly_leader() in swarm_node.py) is untouched and remains the default.

Wire protocol: JSON over UDP, one datagram per message, direct unicast --
NOT the broadcast telemetry link in swarm_link.py, which is keyed by node id
and would silently overwrite/collide with command messages. This is its own
socket on cfg.CMD_PORT.

    command: {"seq": int, "cmd": "ARM"|"TAKEOFF"|"GOTO"|"HOLD"|"LAND",
              "args": {...}}
    ack:     {"seq": int, "status": "accepted"|"rejected"|"error",
              "detail": str}

    TAKEOFF args (optional): {"alt": float}                  # metres AGL
    GOTO args (one of):      {"n": float, "e": float, "d": float}   # swarm frame
                              {"lat": float, "lon": float, "alt": float}
    FORMATION args:          {"name": str}              # key in cfg.FORMATIONS

Every ack is sent as soon as the command is validated and accepted or
rejected -- GOTO and TAKEOFF do not wait for arrival before acking. The
leader keeps broadcasting its real position on the normal telemetry link
throughout the move; watch that (or swarm_monitor.py) for progress, not the
ack. This channel is unauthenticated and unencrypted, same as the telemetry
broadcast -- see README.md Known Limitations.
"""

import asyncio
import json
import math
import time

from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

import swarm_config as cfg
from formation import clamp_xyz
from geo import geodetic_to_ned

VALID_CMDS = {"ARM", "TAKEOFF", "GOTO", "HOLD", "LAND", "FORMATION"}


# ============================================================== wire protocol
class CommandProtocol(asyncio.DatagramProtocol):
    def __init__(self, commander):
        self.commander = commander
        self._tr = None

    def connection_made(self, transport):
        self._tr = transport

    def datagram_received(self, data, addr):
        try:
            msg = json.loads(data.decode())
            seq, cmd, args = msg["seq"], msg["cmd"], msg.get("args", {})
        except (ValueError, KeyError, UnicodeDecodeError):
            return  # malformed / no seq to ack against -- drop silently
        asyncio.ensure_future(self._handle(seq, cmd, args, addr))

    def error_received(self, exc):
        pass

    async def _handle(self, seq, cmd, args, addr):
        status, detail = await self.commander.handle(cmd, args)
        self._reply(seq, status, detail, addr)

    def _reply(self, seq, status, detail, addr):
        if self._tr is None:
            return
        payload = json.dumps({"seq": seq, "status": status, "detail": detail}).encode()
        self._tr.sendto(payload, addr)


async def open_command_link(commander) -> CommandProtocol:
    loop = asyncio.get_running_loop()
    _, proto = await loop.create_datagram_endpoint(
        lambda: CommandProtocol(commander), local_addr=("0.0.0.0", cfg.CMD_PORT))
    return proto


# ================================================================== commander
class LeaderCommander:
    """
    Validates and executes ARM/TAKEOFF/GOTO/HOLD/LAND against the leader's
    live flight state (disarmed -> armed -> flying -> landing). One instance
    per leader process; run_leader_commands() below owns its lifecycle and
    drives the continuous setpoint stream offboard mode requires.
    """

    def __init__(self, drone, v, tf, formation, log):
        self.drone = drone
        self.v = v
        self.tf = tf          # (n, e, d) local -> swarm frame offset
        self.formation = formation   # ["name"], shared with publisher() -- see swarm_node.py
        self.log = log
        self.state = "disarmed"   # disarmed -> armed -> climbing -> flying -> landing
        self.target = None    # (n, e, d) in swarm frame; set once armed
        self.navigating = False
        self._leg_deadline = None
        self._climb_n = self._climb_e = self._climb_yaw = self._climb_hover_d = None
        self._climb_deadline = None

    # ---- called from CommandProtocol -----------------------------------
    async def handle(self, cmd, args):
        if cmd not in VALID_CMDS:
            return "rejected", f"unknown command {cmd!r}"
        try:
            return await getattr(self, f"_cmd_{cmd.lower()}")(args)
        except Exception as exc:
            self.log(f"command {cmd} raised: {exc}")
            return "error", str(exc)

    def _here(self):
        return (self.v.pn + self.tf[0], self.v.pe + self.tf[1], self.v.pd + self.tf[2])

    async def _cmd_arm(self, args):
        if self.state != "disarmed":
            return "rejected", f"already {self.state}"
        await self.drone.action.arm()
        # offboard requires one setpoint already published before start()
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(self.v.pn, self.v.pe, self.v.pd, self.v.yaw))
        try:
            await self.drone.offboard.start()
        except OffboardError as exc:
            await self.drone.action.disarm()
            return "error", f"offboard start failed: {exc._result.result}"
        self.state = "armed"
        self.target = self._here()  # hold here; offboard needs a steady stream
        self.navigating = False
        self.log("armed, holding on the ground")
        return "accepted", "armed"

    async def _cmd_takeoff(self, args):
        if self.state != "armed":
            return "rejected", f"must be armed, not {self.state}"
        alt = float(args.get("alt", cfg.TAKEOFF_ALT))
        self._climb_n, self._climb_e, self._climb_yaw = self.v.pn, self.v.pe, self.v.yaw
        self._climb_hover_d = self.v.pd - alt
        self._climb_deadline = time.monotonic() + 30.0
        self.state = "climbing"
        self.log(f"climbing to {alt:.0f} m AGL")
        # acks immediately, like GOTO -- step() drives the climb and flips
        # to "flying" once it arrives (or times out), same pattern as a leg
        return "accepted", f"climbing to {alt:.0f} m AGL"

    async def _cmd_goto(self, args):
        if self.state != "flying":
            return "rejected", f"must be flying, not {self.state}"
        if "lat" in args:
            n, e, d = geodetic_to_ned(args["lat"], args["lon"], args["alt"],
                                       cfg.ORIGIN_LAT, cfg.ORIGIN_LON, cfg.ORIGIN_ALT)
        elif "n" in args:
            n, e, d = args["n"], args["e"], args["d"]
        else:
            return "rejected", "goto needs n/e/d or lat/lon/alt"
        self.target = (n, e, d)
        self.navigating = True
        self._leg_deadline = time.monotonic() + cfg.LEG_TIMEOUT
        self.log(f"goto N{n:.1f} E{e:.1f} D{d:.1f}")
        return "accepted", f"navigating to N{n:.1f} E{e:.1f} D{d:.1f}"

    async def _cmd_hold(self, args):
        if self.state != "flying":
            return "rejected", f"must be flying, not {self.state}"
        self.target = self._here()
        self.navigating = False
        self.log("holding")
        return "accepted", "holding at current position"

    async def _cmd_formation(self, args):
        name = args.get("name")
        if name not in cfg.FORMATIONS:
            return "rejected", f"unknown formation {name!r}; known: {sorted(cfg.FORMATIONS)}"
        if name == self.formation[0]:
            return "accepted", f"already in {name}"
        self.formation[0] = name  # picked up by publisher() and cascaded to children's "fm"
        self.log(f"formation -> {name}")
        return "accepted", f"switching to {name}"

    async def _cmd_land(self, args):
        if self.state not in ("armed", "climbing", "flying"):
            return "rejected", f"nothing to land from state {self.state}"
        self.state = "landing"
        self.navigating = False
        self.log("land commanded")
        return "accepted", "landing"

    # ---- setpoint tick, called every cycle by run_leader_commands() ----
    async def step(self):
        """Publish one offboard setpoint if there's a target to hold, climb
        to, or fly to. Offboard mode needs a setpoint at > 2 Hz continuously
        from the moment it starts, so this runs while armed-on-the-ground
        and while climbing too, not only once "flying"."""
        if self.state == "climbing":
            reached = abs(self.v.pd - self._climb_hover_d) <= 0.6
            timed_out = time.monotonic() > self._climb_deadline
            if reached or timed_out:
                if timed_out and not reached:
                    self.log("climb timed out, holding at current altitude")
                self.state = "flying"
                self.target = self._here()
                self.navigating = False
                self.log("airborne, holding")
            else:
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(self._climb_n, self._climb_e,
                                   self._climb_hover_d, self._climb_yaw))
            return

        if self.state not in ("armed", "flying") or self.target is None:
            return

        n, e, d = self.target
        ln, le, ld = n - self.tf[0], e - self.tf[1], d - self.tf[2]  # swarm -> local
        yaw = self.v.yaw
        fn = fe = fd = 0.0

        if self.navigating:
            err = math.sqrt((ln - self.v.pn) ** 2 + (le - self.v.pe) ** 2 + (ld - self.v.pd) ** 2)
            timed_out = time.monotonic() > self._leg_deadline
            if err < cfg.ARRIVAL_R or timed_out:
                if timed_out:
                    self.log(f"goto timed out {err:.1f} m short, holding")
                self.navigating = False
            else:
                dn, de = ln - self.v.pn, le - self.v.pe
                if math.hypot(dn, de) > 1.0:
                    yaw = math.degrees(math.atan2(de, dn))
                fn, fe, fd = clamp_xyz(ln - self.v.pn, le - self.v.pe, ld - self.v.pd, cfg.LEADER_SPEED)

        await self.drone.offboard.set_position_velocity_ned(
            PositionNedYaw(ln, le, ld, yaw), VelocityNedYaw(fn, fe, fd, yaw))


# ============================================================ leader run loop
async def run_leader_commands(drone, v, tf, phase, formation, log):
    """
    Command-driven replacement for fly_leader(): waits for ARM/TAKEOFF over
    the command link, then holds/navigates per GOTO/HOLD/FORMATION until LAND
    is issued. Mirrors fly_leader()'s own tail (publish phase[0]="land" and
    hold position for 3s so children reliably see it in a broadcast packet)
    so landing cascades down the tree exactly the same way in both modes; the
    caller's run()/finally still does the actual PX4 offboard.stop()/land().
    """
    commander = LeaderCommander(drone, v, tf, formation, log)
    await open_command_link(commander)
    log(f"command interface listening on 0.0.0.0:{cfg.CMD_PORT} -- waiting for ARM")

    dt = 1.0 / cfg.PUBLISH_HZ
    while commander.state != "landing":
        await commander.step()
        await asyncio.sleep(dt)

    phase[0] = "land"
    log("land commanded, broadcasting for children")
    for _ in range(int(3.0 * cfg.PUBLISH_HZ)):
        await drone.offboard.set_position_ned(PositionNedYaw(v.pn, v.pe, v.pd, v.yaw))
        await asyncio.sleep(dt)
