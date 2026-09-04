#!/usr/bin/env python3
"""
Operator CLI for the swarm leader's command interface (leader_commands.py).
Requires the leader (node 0) to be running with --command-interface.

    swarm_cli.py arm
    swarm_cli.py takeoff [--alt 5]
    swarm_cli.py goto --ned N E D
    swarm_cli.py goto --latlon LAT LON ALT
    swarm_cli.py hold
    swarm_cli.py land
    swarm_cli.py formation {wedge,line-h,line-v}
    swarm_cli.py                      # REPL, one command per line, Ctrl-D to exit

Sends one JSON command datagram to the leader's cfg.CMD_PORT and waits up to
--timeout seconds for the matching-seq ack. GOTO and TAKEOFF ack as soon as
the leader ACCEPTS the command, not when it's actually reached -- watch
`tail -f` on the leader's log or swarm_monitor.py for real progress.
"""

import argparse
import itertools
import json
import socket

import swarm_config as cfg

_seq = itertools.count(1)


def send_command(host, port, timeout, cmd, args):
    seq = next(_seq)
    payload = json.dumps({"seq": seq, "cmd": cmd, "args": args}).encode()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(payload, (host, port))
        while True:
            data, _ = sock.recvfrom(4096)
            reply = json.loads(data.decode())
            if reply.get("seq") == seq:
                return reply
    except socket.timeout:
        return None
    finally:
        sock.close()


def report(reply) -> int:
    if reply is None:
        print("no response (timed out)")
        return 1
    status, detail = reply.get("status"), reply.get("detail", "")
    print(f"{status}: {detail}")
    return 0 if status == "accepted" else 1


def build_args(ns, cmd):
    if cmd == "TAKEOFF":
        return {"alt": ns.alt} if ns.alt is not None else {}
    if cmd == "GOTO":
        if ns.ned:
            n, e, d = ns.ned
            return {"n": n, "e": e, "d": d}
        lat, lon, alt = ns.latlon
        return {"lat": lat, "lon": lon, "alt": alt}
    if cmd == "FORMATION":
        return {"name": ns.name}
    return {}


def make_parser():
    ap = argparse.ArgumentParser(
        description="Send ARM/TAKEOFF/GOTO/HOLD/LAND/FORMATION commands to "
                     "the swarm leader's command interface (swarm_node.py "
                     "--command-interface). Run with no subcommand for an "
                     "interactive REPL.")
    ap.add_argument("--host", default="127.0.0.1", help="leader's command-link host")
    ap.add_argument("--port", type=int, default=cfg.CMD_PORT, help="leader's command port")
    ap.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for an ack")

    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("arm", help="arm the leader")

    tk = sub.add_parser("takeoff", help="climb and hold")
    tk.add_argument("--alt", type=float, default=None,
                     help=f"metres AGL (default {cfg.TAKEOFF_ALT})")

    go = sub.add_parser("goto", help="fly to a point, then hold")
    grp = go.add_mutually_exclusive_group(required=True)
    grp.add_argument("--ned", type=float, nargs=3, metavar=("N", "E", "D"),
                      help="swarm-frame north/east/down, metres")
    grp.add_argument("--latlon", type=float, nargs=3, metavar=("LAT", "LON", "ALT"),
                      help="geodetic lat/lon/alt-AMSL")

    sub.add_parser("hold", help="hover at the current position")
    sub.add_parser("land", help="land (cascades to followers)")

    fm = sub.add_parser("formation", help="switch the swarm's formation shape")
    fm.add_argument("name", choices=sorted(cfg.FORMATIONS),
                     help="formation to switch to")
    return ap


def run_repl(host, port, timeout):
    print("swarm command REPL -- Ctrl-D to exit")
    print("  arm")
    print("  takeoff [alt]")
    print("  goto N E D")
    print("  goto --latlon LAT LON ALT")
    print("  hold")
    print("  land")
    print(f"  formation {{{','.join(sorted(cfg.FORMATIONS))}}}")
    while True:
        try:
            line = input("swarm> ").strip()
        except EOFError:
            print()
            return

        if not line:
            continue
        parts = line.split()
        cmd = parts[0].upper()
        rest = parts[1:]

        if cmd not in ("ARM", "TAKEOFF", "GOTO", "HOLD", "LAND", "FORMATION"):
            print(f"unknown command {parts[0]!r}")
            continue

        try:
            if cmd == "TAKEOFF":
                args = {"alt": float(rest[0])} if rest else {}
            elif cmd == "GOTO":
                if rest and rest[0] == "--latlon":
                    lat, lon, alt = (float(x) for x in rest[1:])
                    args = {"lat": lat, "lon": lon, "alt": alt}
                else:
                    n, e, d = (float(x) for x in rest)
                    args = {"n": n, "e": e, "d": d}
            elif cmd == "FORMATION":
                if not rest:
                    print("usage: formation {" + ",".join(sorted(cfg.FORMATIONS)) + "}")
                    continue
                args = {"name": rest[0]}
            else:
                args = {}
        except ValueError:
            print("bad arguments")
            continue

        report(send_command(host, port, timeout, cmd, args))


def main(argv=None) -> int:
    ns = make_parser().parse_args(argv)
    if ns.cmd is None:
        run_repl(ns.host, ns.port, ns.timeout)
        return 0
    cmd = ns.cmd.upper()
    return report(send_command(ns.host, ns.port, ns.timeout, cmd, build_args(ns, cmd)))


if __name__ == "__main__":
    raise SystemExit(main())
