#!/usr/bin/env python3
"""Cheap flight-plan preview from a run's meta_*.json — no UE, no bag.

Static PNG (paths + spawns + geofence + stats) and optional MP4 animation of
the paths growing at ANIM_SPEED x realtime.

usage: make_path_preview.py <label> [--runs-dir D] [--half-m H] [--anim]
"""
import argparse
import glob
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

COLORS = {"ghost": "#3f8cff", "delta": "#3fff72",
          "buckshee": "#ff4cf2", "thunderstrike": "#ff941e"}
DRONES = list(COLORS)

ap = argparse.ArgumentParser()
ap.add_argument("label")
ap.add_argument("--runs-dir", default="/home/lucas/hercules-sim/e1_frames/runs")
ap.add_argument("--half-m", type=float, default=float(os.environ.get("HERC_ARENA_HALF_M", "12")))
ap.add_argument("--anim", action="store_true")
ap.add_argument("--out-dir", default=None)
a = ap.parse_args()

run = os.path.join(a.runs_dir, a.label)
out_dir = a.out_dir or run
tracks = {}
for d in DRONES:
    rows = []
    for f in sorted(glob.glob(f"{run}/{a.label}_{d}/meta_*.json")):
        m = json.load(open(f))
        n = m.get("ned")
        if n and (abs(n[0]) + abs(n[1]) > 1e-6 or m.get("state") == "EXECUTE"):
            rows.append((m["t"], n[0], n[1], -n[2], m.get("state", "?")))
    tracks[d] = rows
    if not rows:
        raise SystemExit(f"no track for {d}")

# spawns from settings actually used (cross layout default)
S = json.load(open(os.environ.get(
    "HERC_SETTINGS", "/home/lucas/hercules-sim/settings-fleet-4drone-cross.json")))
spawns = {v.lower(): (c["X"], c["Y"]) for v, c in S["Vehicles"].items()}
cx = sum(x for x, _ in spawns.values()) / 4
cy = sum(y for _, y in spawns.values()) / 4
# TRUE fence = union box (team_box.py): [min spawn - half, max spawn + half]
H = a.half_m
fx0 = min(x for x, _ in spawns.values()) - H
fx1 = max(x for x, _ in spawns.values()) + H
fy0 = min(y for _, y in spawns.values()) - H
fy1 = max(y for _, y in spawns.values()) + H

t0 = min(r[0][0] for r in tracks.values())
t1 = max(r[-1][0] for r in tracks.values())

def draw(ax, tmax=None):
    ax.set_facecolor("#14161e")
    ax.add_patch(plt.Rectangle((fy0, fx0), fy1 - fy0, fx1 - fx0, fill=False,
                               ec="#8f98a8", ls="--", lw=1.2))
    stats = []
    for d in DRONES:
        rows = tracks[d]
        if tmax is not None:
            rows = [r for r in rows if r[0] <= tmax]
        if not rows:
            continue
        # world XY: spawn offset + NED (north=x, east=y); plot x=EAST, y=NORTH
        sx, sy = spawns[d]
        xs = [sy + r[2] for r in rows]
        ys = [sx + r[1] for r in rows]
        # dim history + bright recent (last 20 s)
        tcut = rows[-1][0] - 20.0
        hx = [x for x, r in zip(xs, rows) if r[0] <= tcut]
        hy = [y for y, r in zip(ys, rows) if r[0] <= tcut]
        rx = [x for x, r in zip(xs, rows) if r[0] > tcut]
        ry = [y for y, r in zip(ys, rows) if r[0] > tcut]
        ax.plot(hx, hy, color=COLORS[d], lw=1.1, alpha=0.35, solid_capstyle="round")
        ax.plot(rx, ry, color=COLORS[d], lw=2.2, alpha=0.95, solid_capstyle="round")
        ax.plot([sy], [sx], marker="s", ms=7, mfc="none", mec=COLORS[d], mew=1.5)
        if rx:
            ax.plot([rx[-1]], [ry[-1]], marker="o", ms=8, color=COLORS[d])
        path_m = sum(math.dist((xs[i], ys[i]), (xs[i - 1], ys[i - 1]))
                     for i in range(1, len(xs)))
        rmax = max(math.hypot(x - cy, y - cx) for x, y in zip(xs, ys))
        stats.append(f"{d.upper():13s} path {path_m:5.1f} m   max radius {rmax:4.1f} m")
    ax.set_aspect("equal")
    ax.set_xlim(fy0 - 2, fy1 + 2)
    ax.set_ylim(fx0 - 2, fx1 + 2)
    ax.set_xlabel("east (m)", color="#aab2c0")
    ax.set_ylabel("north (m)", color="#aab2c0")
    ax.tick_params(colors="#8f98a8", labelsize=8)
    for s in ax.spines.values():
        s.set_color("#3a3f4c")
    return stats

fig, ax = plt.subplots(figsize=(9.6, 9.6), dpi=100)
fig.patch.set_facecolor("#14161e")
stats = draw(ax)
el = t1 - t0
ax.set_title(f"{a.label} — flown paths, {el:.0f}s, fence ±{a.half_m:.0f}m (dashed)",
             color="white", fontsize=13, pad=12)
fig.text(0.02, 0.015, "\n".join(stats), color="#d8e0ec", fontsize=9,
         family="monospace", va="bottom")
png = f"{out_dir}/path_preview_{a.label}.png"
fig.savefig(png, bbox_inches="tight", facecolor=fig.get_facecolor())
print("wrote", png)

if a.anim:
    SPEED = float(os.environ.get("ANIM_SPEED", "15"))
    FPS = 30
    frames = int(el / SPEED * FPS)
    fig2, ax2 = plt.subplots(figsize=(9.6, 5.4), dpi=100)
    fig2.patch.set_facecolor("#14161e")
    mp4 = f"{out_dir}/path_preview_{a.label}.mp4"
    w = FFMpegWriter(fps=FPS, bitrate=1800)
    with w.saving(fig2, mp4, dpi=100):
        for k in range(frames + 1):
            ax2.clear()
            draw(ax2, tmax=t0 + k / FPS * SPEED)
            ax2.set_title(f"{a.label}  t+{k / FPS * SPEED:5.1f}s  ({SPEED:.0f}x)",
                          color="white", fontsize=12)
            w.grab_frame()
    print("wrote", mp4)
