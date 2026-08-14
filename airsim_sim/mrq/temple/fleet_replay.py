#!/usr/bin/env python3
"""Fleet 'mission control' replay publisher — archived mrq6 data -> one RViz
domain. No sim: reads meta_*.json (4.5Hz ground-truth poses) + coord_*.log
claim events, publishes MarkerArray /fleet_markers in the shared spawn-
corrected arena frame ('map'):

  * 4 drone spheres + name labels (per-drone colors, cinematic palette)
  * growing Path traces (LINE_STRIP live, dim history for pre-window flight)
  * claim discs: translucent r_claim=4.0m CYLINDER at each ACTIVE claim in the
    claiming drone's color + 'gain N' label; appears on 'claim set', vanishes
    on 'claim released' (claims are in each drone's own odom frame ->
    spawn-corrected; thunderstrike's late-run offset = its real VIO drift)
  * coverage cells accumulating (CUBE_LIST per drone; pre-window dim, live
    bright), scorecard-exact first-seen model (same as the baked texture)
  * clock 't+MM:SS' + 'explored: N cells' counter

Replays [t0-LEAD, t1+HOLD] at --speed (wall-clock pacing). Writes
fleet_replay_epochs.txt with the wall epoch of replay-start for video cutting.
"""
import argparse
import bisect
import glob
import json
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

RUN = "/home/lucas/hercules-sim/e1_frames/runs/mrq6"
SETTINGS = "/home/lucas/hercules-sim/settings-fleet-4drone.json"
KEYS = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple/fleet_keys_temple.json"
VEHS = ["ghost", "delta", "buckshee", "thunderstrike"]
COLORS = {"ghost": (0.20, 0.50, 1.00), "delta": (0.20, 1.00, 0.40),
          "buckshee": (0.10, 0.90, 1.00), "thunderstrike": (1.00, 0.45, 0.10)}
DISC = [(dx, dy) for dx in range(-4, 5) for dy in range(-4, 5)
        if dx * dx + dy * dy <= 16]
R_CLAIM = 4.0


def col(v, a=1.0, dim=1.0):
    r, g, b = COLORS[v]
    return ColorRGBA(r=r * dim, g=g * dim, b=b * dim, a=a)


class FleetReplay(Node):
    def __init__(self, speed, lead, hold, rate):
        super().__init__("fleet_replay")
        self.pub = self.create_publisher(MarkerArray, "/fleet_markers",
                                         QoSProfile(depth=5))
        self.speed, self.lead, self.hold, self.rate = speed, lead, hold, rate
        K = json.load(open(KEYS))
        self.t0, self.t1 = K["t0_epoch"], K["t1_epoch"]
        spawn = {v: (d["X"], d["Y"]) for v, d in
                 json.load(open(SETTINGS))["Vehicles"].items()}
        self.tracks = {}
        samples = []
        for v in VEHS:
            ox, oy = spawn[v]
            ts, xs, ys = [], [], []
            for m in sorted(glob.glob(f"{RUN}/mrq6_{v}/meta_*.json")):
                try:
                    j = json.load(open(m))
                except Exception:
                    continue
                ned = j.get("ned") or []
                if len(ned) < 2:
                    continue
                ts.append(float(j["t"]))
                xs.append(float(ned[0]) + ox)
                ys.append(float(ned[1]) + oy)
            self.tracks[v] = (ts, xs, ys)
            for t, x, y in zip(ts, xs, ys):
                samples.append((t, v, x, y))
        # coverage first-seen (scorecard-exact; cell from spawn-corrected int())
        samples.sort()
        self.first = {}
        for t, v, x, y in samples:
            cx, cy = int(x), int(y)
            for dx, dy in DISC:
                c = (cx + dx, cy + dy)
                if c not in self.first:
                    self.first[c] = (t, v)
        self.cov_sorted = sorted((t, v, c) for c, (t, v) in self.first.items())
        self.cov_times = [e[0] for e in self.cov_sorted]
        # claim events
        self.claims = {}
        for v in VEHS:
            ox, oy = spawn[v]
            ev = []
            for ln in open(f"{RUN}/coord_mrq6_{v}.log"):
                m = re.search(r"\[(\d+\.\d+)\].*?claim set: \(([-\d.]+), ([-\d.]+)\) gain (\d+)", ln)
                if m:
                    ev.append((float(m.group(1)), "set",
                               float(m.group(2)) + ox, float(m.group(3)) + oy,
                               int(m.group(4))))
                    continue
                m = re.search(r"\[(\d+\.\d+)\].*?claim released", ln)
                if m:
                    ev.append((float(m.group(1)), "rel", 0, 0, 0))
            self.claims[v] = sorted(ev)
        n_claims = sum(len(e) for e in self.claims.values())
        print(f"loaded: {len(samples)} poses, {len(self.first)} cells, "
              f"{n_claims} claim events, window [{self.t0:.2f},{self.t1:.2f}]", flush=True)

    def active_claim(self, v, T):
        cur = None
        for t, kind, x, y, g in self.claims[v]:
            if t > T:
                break
            cur = (x, y, g) if kind == "set" else None
        return cur

    def pose(self, v, T):
        ts, xs, ys = self.tracks[v]
        if T <= ts[0]:
            return xs[0], ys[0]
        if T >= ts[-1]:
            return xs[-1], ys[-1]
        i = bisect.bisect_right(ts, T) - 1
        f = (T - ts[i]) / (ts[i + 1] - ts[i])
        return xs[i] + f * (xs[i + 1] - xs[i]), ys[i] + f * (ys[i + 1] - ys[i])

    def frame(self, T):
        ma = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        ma.markers.append(wipe)
        mid = 0

        def mk(ns, mtype):
            nonlocal mid
            m = Marker()
            m.header.frame_id = "map"
            m.ns = ns
            m.id = mid
            mid += 1
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        # coverage cube lists: pre-window dim, in-window bright
        k = bisect.bisect_right(self.cov_times, T)
        pre = {v: [] for v in VEHS}
        live = {v: [] for v in VEHS}
        for t, v, c in self.cov_sorted[:k]:
            (pre if t < self.t0 else live)[v].append(c)
        for v in VEHS:
            for cells, alpha, dim, zed in ((pre[v], 0.28, 0.45, -0.08),
                                           (live[v], 0.62, 1.0, -0.04)):
                if not cells:
                    continue
                m = mk("cov_pre" if zed < -0.05 else "cov", Marker.CUBE_LIST)
                m.scale.x = m.scale.y = 0.94
                m.scale.z = 0.02
                m.color = col(v, alpha, dim)
                m.pose.position.z = zed
                m.points = [Point(x=c[0] + 0.5, y=c[1] + 0.5, z=0.0) for c in cells]
                ma.markers.append(m)
        # history + live paths
        for v in VEHS:
            ts, xs, ys = self.tracks[v]
            i0 = bisect.bisect_left(ts, self.t0)
            iT = bisect.bisect_right(ts, T)
            hist = mk("hist", Marker.LINE_STRIP)
            hist.scale.x = 0.07
            hist.color = col(v, 0.30, 0.7)
            hist.points = [Point(x=xs[i], y=ys[i], z=0.05) for i in range(0, min(i0, iT))]
            if len(hist.points) > 1:
                ma.markers.append(hist)
            live_p = mk("path", Marker.LINE_STRIP)
            live_p.scale.x = 0.16
            live_p.color = col(v, 0.95)
            live_p.points = [Point(x=xs[i], y=ys[i], z=0.10) for i in range(i0, iT)]
            px, py = self.pose(v, T)
            live_p.points.append(Point(x=px, y=py, z=0.10))
            if len(live_p.points) > 1:
                ma.markers.append(live_p)
            # drone
            d = mk("drone", Marker.SPHERE)
            d.scale.x = d.scale.y = d.scale.z = 0.9
            d.color = col(v, 1.0)
            d.pose.position.x, d.pose.position.y, d.pose.position.z = px, py, 0.4
            ma.markers.append(d)
            nm = mk("name", Marker.TEXT_VIEW_FACING)
            nm.text = v
            nm.scale.z = 1.1
            nm.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            nm.pose.position.x, nm.pose.position.y, nm.pose.position.z = px, py + 1.2, 0.6
            ma.markers.append(nm)
            # active claim
            c = self.active_claim(v, T)
            if c:
                cx, cy, g = c
                disc = mk("claim", Marker.CYLINDER)
                disc.scale.x = disc.scale.y = 2 * R_CLAIM
                disc.scale.z = 0.06
                disc.color = col(v, 0.30)
                disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = cx, cy, 0.15
                ma.markers.append(disc)
                ring = mk("claim_c", Marker.SPHERE)
                ring.scale.x = ring.scale.y = 0.5
                ring.scale.z = 0.1
                ring.color = col(v, 0.95)
                ring.pose.position.x, ring.pose.position.y, ring.pose.position.z = cx, cy, 0.2
                ma.markers.append(ring)
                txt = mk("gain", Marker.TEXT_VIEW_FACING)
                txt.text = f"gain {g}"
                txt.scale.z = 1.0
                txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.85)
                txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = cx, cy - 1.4, 0.3
                ma.markers.append(txt)
        # clock + counter (top-left of arena)
        clk = mk("clock", Marker.TEXT_VIEW_FACING)
        rel = max(0.0, T - self.t0)
        clk.text = f"t+{int(rel // 60):02d}:{int(rel % 60):02d}   explored: {k} cells"
        clk.scale.z = 1.6
        clk.color = ColorRGBA(r=0.9, g=0.93, b=1.0, a=0.95)
        clk.pose.position.x, clk.pose.position.y, clk.pose.position.z = 5.0, 16.5, 1.0
        ma.markers.append(clk)
        return ma

    def run(self):
        start_wall = time.time()
        with open("/home/lucas/UE5/hercules-sim-big/mrq_work/temple/fleet_replay_epochs.txt", "w") as f:
            f.write(f"start_wall {start_wall:.3f}\nspeed {self.speed}\nlead {self.lead}\n")
        total = (self.lead + (self.t1 - self.t0)) / self.speed + self.hold
        print(f"replay start (wall {start_wall:.3f}), {total:.0f}s total", flush=True)
        while rclpy.ok():
            el = time.time() - start_wall
            T = self.t0 - self.lead + el * self.speed
            if T > self.t1 + 0.01:
                T = self.t1
            self.pub.publish(self.frame(T))
            if el >= total:
                break
            time.sleep(1.0 / self.rate)
        print("replay done", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--lead", type=float, default=3.0)   # seconds of pre-roll state
    ap.add_argument("--hold", type=float, default=4.0)
    ap.add_argument("--rate", type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    n = FleetReplay(a.speed, a.lead, a.hold, a.rate)
    n.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
