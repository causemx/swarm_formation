"""
UDP broadcast link.

Every node opens one socket bound to the same port, broadcasts its own state at
PUBLISH_HZ, and caches whatever it hears from the others. There is no
addressing, no handshake and no acknowledgement: a node simply picks its
parent's entry out of the cache. Adding a vehicle means starting a process.

Note on SO_REUSEPORT: several sockets on one host can bind the same UDP port,
and broadcast datagrams are delivered to all of them. Plain *unicast* to that
port would be load-balanced to a single socket, which is why the sender must
target the broadcast address and not 127.0.0.1.
"""

import asyncio
import json
import socket
import time

import swarm_config as cfg


class SwarmLink(asyncio.DatagramProtocol):
    def __init__(self, node_id: int):
        self.node_id = node_id
        self._peers = {}
        self._tr = None

    # ------------------------------------------------------------ asyncio hooks
    def connection_made(self, transport):
        self._tr = transport

    def datagram_received(self, data, addr):
        try:
            msg = json.loads(data.decode())
        except (ValueError, UnicodeDecodeError):
            return
        if msg.get("id") == self.node_id:
            return  # our own broadcast loops back to us
        msg["rx_t"] = time.monotonic()
        self._peers[msg["id"]] = msg

    def error_received(self, exc):
        pass

    # ------------------------------------------------------------------- public
    def publish(self, state: dict):
        if self._tr is None:
            return
        state["id"] = self.node_id
        self._tr.sendto(json.dumps(state).encode(), (cfg.BCAST_ADDR, cfg.UDP_PORT))

    def peer(self, peer_id, max_age=cfg.PEER_TIMEOUT):
        """Latest state of `peer_id`, or None if we never heard it / it went stale."""
        msg = self._peers.get(peer_id)
        if msg is None:
            return None
        if time.monotonic() - msg["rx_t"] > max_age:
            return None
        return msg

    def all_peers(self, max_age=cfg.PEER_TIMEOUT):
        now = time.monotonic()
        return {k: v for k, v in self._peers.items() if now - v["rx_t"] <= max_age}


async def open_link(node_id: int) -> SwarmLink:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", cfg.UDP_PORT))

    loop = asyncio.get_running_loop()
    _, proto = await loop.create_datagram_endpoint(lambda: SwarmLink(node_id), sock=sock)
    return proto
