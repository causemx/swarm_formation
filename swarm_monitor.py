#!/usr/bin/env python3
"""
Passive listener. Joins the broadcast group as a node id that never transmits,
and prints what everyone is saying plus the live formation error.

    python3 swarm_monitor.py
"""

import asyncio
import math

import swarm_config as cfg
from swarm_link import open_link
from formation import formation_target


async def main():
    link = await open_link(node_id=-1)
    print(f"listening on {cfg.BCAST_ADDR}:{cfg.UDP_PORT}\n")

    while True:
        peers = link.all_peers()
        rows = []
        for nid in sorted(peers):
            p = peers[nid]
            node = cfg.SWARM.get(nid)
            err = "   -  "
            if node is not None and node.parent is not None:
                parent = peers.get(node.parent)
                if parent is not None:
                    tn, te, td = formation_target(parent, node.offset)
                    err = f"{math.dist((tn, te, td), (p['n'], p['e'], p['d'])):6.2f}"
            rows.append(
                f"  {nid}   {p['lvl']}   {p['st']:<6}"
                f" {p['n']:8.1f} {p['e']:8.1f} {p['d']:8.1f}"
                f" {p['yaw']:7.1f}"
                f" {math.hypot(p['vn'], p['ve']):6.2f}"
                f"  {err}")

        print("\033[2J\033[H", end="")
        print(" id lvl state       north     east     down     yaw   speed   f_err")
        print("\n".join(rows) if rows else "  (nothing on the wire)")
        await asyncio.sleep(0.5)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
