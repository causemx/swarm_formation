#!/usr/bin/env python3
"""
Process supervisor for the swarm demo -- replaces run_nodes.sh.

    ./node_supervisor.py [-n N]        # default 5, same as run_nodes.sh 5

Starts one `swarm_node.py --id i` subprocess per drone (i in [0, N)), staggered
by --stagger seconds so the leader (id 0) is up before its children. Each
child's stdout/stderr goes to LOGDIR/node_<i>.log (LOGDIR env var or --logdir,
default /tmp/px4_swarm), truncated on each run.

Run this only after ./run_sitl.sh has come up and the vehicles have a GPS fix.

Shutdown: swarm_node.py only lands gracefully on SIGINT (its main() catches
KeyboardInterrupt; SIGTERM has no handler installed there and kills it
immediately, mid-flight). So on Ctrl-C / SIGTERM to the supervisor itself, all
children are sent SIGINT first and given --shutdown-timeout seconds to finish
their own landing + disarm sequence before SIGTERM, then SIGKILL, are used to
clean up anything still alive. Since the swarm link is best-effort UDP
broadcast with no addressing (see swarm_link.py) and every node's shutdown
path is independent of its peers, the order children are signalled in has no
safety effect -- they're all interrupted together.

While running, prints a live status table every --status-interval seconds:
node id, pid, uptime, alive/exit state, and the last line written to its log.

    tail -f LOGDIR/node_0.log     # follow the leader
    python3 swarm_monitor.py      # see the whole formation

Exit code is 0 after a clean interrupt-driven shutdown, 1 if any child
crashed (nonzero return code) before shutdown was requested.
"""

import argparse
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from subprocess import Popen


@dataclass
class Node:
    node_id: int
    log_path: Path
    proc: Popen
    start_t: float = field(default_factory=time.monotonic)
    restarts: int = 0
    reported_crash: bool = False


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Start and supervise one swarm_node.py process per drone.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("-n", "--nodes", type=int, default=5,
                     help="number of nodes to launch (ids 0..N-1)")
    ap.add_argument("--logdir", default=os.environ.get("LOGDIR", "/tmp/px4_swarm"),
                     help="directory for node_<i>.log files (env LOGDIR overrides default)")
    ap.add_argument("--stagger", type=float, default=0.5,
                     help="seconds to wait between launching each node")
    ap.add_argument("--python", default=sys.executable,
                     help="interpreter used to run swarm_node.py")
    ap.add_argument("--script", default="./swarm_node.py",
                     help="path to swarm_node.py")
    ap.add_argument("--shutdown-timeout", type=float, default=20.0,
                     help="seconds to wait after SIGINT before escalating to "
                          "SIGTERM/SIGKILL (swarm_node.py's own landing "
                          "sequence sleeps up to ~15s)")
    ap.add_argument("--status-interval", type=float, default=5.0,
                     help="seconds between live status table refreshes")
    ap.add_argument("--restart-on-crash", action="store_true",
                     help="relaunch a node if it exits with a nonzero code "
                          "while the swarm is still running. Off by default: "
                          "this is a flight demo, and silently respawning a "
                          "process mid-flight is dangerous, not helpful.")
    return ap.parse_args(argv)


def tail_last_line(path: Path, max_read: int = 4096) -> str:
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - max_read))
            chunk = f.read()
    except OSError:
        return ""
    lines = [ln for ln in chunk.decode(errors="replace").splitlines() if ln.strip()]
    return lines[-1] if lines else ""


class Supervisor:
    def __init__(self, args):
        self.args = args
        self.logdir = Path(args.logdir)
        self.logdir.mkdir(parents=True, exist_ok=True)
        self.script = Path(args.script)
        self.nodes: list[Node] = []
        self.shutting_down = False
        self.any_crash = False

    def launch(self, node_id: int) -> Node:
        log_path = self.logdir / f"node_{node_id}.log"
        with open(log_path, "w") as log_f:
            proc = Popen(
                [self.args.python, str(self.script), "--id", str(node_id)],
                stdout=log_f, stderr=log_f,
            )
        print(f"node {node_id}  ->  pid {proc.pid}  ->  {log_path}")
        return Node(node_id=node_id, log_path=log_path, proc=proc)

    def launch_all(self):
        for i in range(self.args.nodes):
            self.nodes.append(self.launch(i))
            if i < self.args.nodes - 1:
                time.sleep(self.args.stagger)

    def print_hints(self):
        leader_log = self.logdir / "node_0.log"
        print()
        print(f"follow the leader with:   tail -f {leader_log}")
        print("see the whole swarm with: python3 swarm_monitor.py")
        print("ctrl-c to stop")
        print()

    def print_status(self):
        now = time.monotonic()
        print(f"--- status {time.strftime('%H:%M:%S')} " + "-" * 40)
        for n in self.nodes:
            rc = n.proc.poll()
            uptime = now - n.start_t
            state = "alive" if rc is None else f"exited({rc})"
            last = tail_last_line(n.log_path)
            print(f"  node {n.node_id:>2}  pid {n.proc.pid:<7} "
                  f"up {uptime:7.1f}s  {state:<12}  {last}")
        print("-" * 62)

    def check_crashes(self):
        """Detect nodes that exited unexpectedly; restart if asked to."""
        for i, n in enumerate(self.nodes):
            rc = n.proc.poll()
            if rc is None or n.reported_crash or self.shutting_down:
                continue
            n.reported_crash = True
            if rc == 0:
                print(f"[supervisor] node {n.node_id} exited cleanly (rc=0), "
                      f"log: {n.log_path}")
                continue
            self.any_crash = True
            print(f"[supervisor] WARNING: node {n.node_id} crashed "
                  f"(rc={rc}), log: {n.log_path}", file=sys.stderr)
            if self.args.restart_on_crash:
                n.restarts += 1
                print(f"[supervisor] restarting node {n.node_id} "
                      f"(restart #{n.restarts})")
                self.nodes[i] = self.launch(n.node_id)

    def all_exited(self) -> bool:
        return all(n.proc.poll() is not None for n in self.nodes)

    def shutdown(self):
        self.shutting_down = True
        print("\nstopping nodes (SIGINT, so each node can land + disarm)")
        for n in self.nodes:
            if n.proc.poll() is None:
                try:
                    n.proc.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass

        deadline = time.monotonic() + self.args.shutdown_timeout
        while time.monotonic() < deadline:
            if all(n.proc.poll() is not None for n in self.nodes):
                break
            time.sleep(0.5)

        stragglers = [n for n in self.nodes if n.proc.poll() is None]
        if stragglers:
            ids = ", ".join(str(n.node_id) for n in stragglers)
            print(f"[supervisor] node(s) {ids} still alive after "
                  f"{self.args.shutdown_timeout:.0f}s, sending SIGTERM")
            for n in stragglers:
                try:
                    n.proc.terminate()
                except ProcessLookupError:
                    pass
            time.sleep(2.0)
            for n in stragglers:
                if n.proc.poll() is None:
                    print(f"[supervisor] node {n.node_id} still alive, SIGKILL")
                    try:
                        n.proc.kill()
                    except ProcessLookupError:
                        pass

        for n in self.nodes:
            n.proc.wait()
        print("all nodes stopped")

    def run(self):
        self.launch_all()
        self.print_hints()

        interrupted = False

        def on_signal(signum, frame):
            nonlocal interrupted
            interrupted = True

        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

        last_status = 0.0
        try:
            while not interrupted:
                self.check_crashes()
                if self.all_exited():
                    print("[supervisor] all nodes have exited on their own")
                    break
                now = time.monotonic()
                if now - last_status >= self.args.status_interval:
                    self.print_status()
                    last_status = now
                time.sleep(0.2)
        finally:
            self.shutdown()

        if self.any_crash and not interrupted:
            return 1
        return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    if not Path(args.script).exists():
        print(f"error: script not found: {args.script}", file=sys.stderr)
        return 2
    sup = Supervisor(args)
    return sup.run()


if __name__ == "__main__":
    raise SystemExit(main())
