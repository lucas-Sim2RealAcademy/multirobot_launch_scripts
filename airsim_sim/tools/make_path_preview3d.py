#!/usr/bin/env python3
"""3D flight-path preview: static isometric PNG + slow-orbit MP4.

Altitude is exaggerated by ZX (default 4x, labeled) so sub-meter altitude
planning reads clearly against a 36m arena.
usage: make_path_preview3d.py <label> [--zx 4] [--anim]
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
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3DCollection

COLORS = {"ghost": "#3f8cff", "delta": "#3fff72",
          "buckshee": "#ff4cf2", "thunderstrike": "#ff941e"}
DRONES = list(COLORS)

ap = argparse.ArgumentParser()
ap.add_argument("label")
ap.add_argument("--runs-dir", default="/home/lucas/hercules-sim/e1_frames/runs")
ap.add_argument("--half-m", type=float, default=float(os.environ.get("HERC_ARENA_HALF_M", "12")))
ap.add_argument("--zx", type=float, default=2.0)
ap.add_argument("--anim", action="store_true")
ap.add_argument("--mesh", default=None, help="mesh_cloud npz for obstacle context")
a = ap.parse_args()

run = os.path.join(a.runs_dir, a.label)
S = json.load(open(os.environ.get(
    "HERC_SETTINGS", "/home/lucas/hercules-sim/settings-fleet-4drone-cross.json")))
spawns = {v.lower(): (c["X"], c["Y"]) for v, c in S["Vehicles"].items()}
H = a.half_m
fx0 = min(x for x, _ in spawns.values()) - H
fx1 = max(x for x, _ in spawns.values()) + H
fy0 = min(y for _, y in spawns.values()) - H
fy1 = max(y for _, y in spawns.values()) + H

tracks = {}
for d in DRONES:
    rows = []
    for f in sorted(glob.glob(f"{run}/{a.label}_{d}/meta_*.json")):
        m = json.load(open(f))
        n = m.get("ned")
        if n and (abs(n[0]) + abs(n[1]) > 1e-6 or m.get("state") == "EXECUTE"):
            sx, sy = spawns[d]
            rows.append((m["t"], sy + n[1], sx + n[0], max(0.0, -n[2])))  # x=east y=north z=AGL
    tracks[d] = rows

zmax = max(r[3] for rows in tracks.values() for r in rows)

mesh_pts = None
if a.mesh:
    import numpy as np
    mesh_pts = np.load(a.mesh)["cloud"]
    # clip to fence + a small margin so the sky-high z scale is set by flight
    m = ((mesh_pts[:, 0] > fy0 - 1) & (mesh_pts[:, 0] < fy1 + 1) &
         (mesh_pts[:, 1] > fx0 - 1) & (mesh_pts[:, 1] < fx1 + 1) &
         (mesh_pts[:, 2] > 0.15) & (mesh_pts[:, 2] < 3.2))
    mesh_pts = mesh_pts[m]
    if len(mesh_pts) > 60000:
        mesh_pts = mesh_pts[np.random.default_rng(3).choice(len(mesh_pts), 60000, replace=False)]


def setup(ax):
    ax.set_facecolor("#14161e")
    zz = (max(zmax, 3.2 if mesh_pts is not None else zmax)) * a.zx
    for z in (0, zz):
        ax.plot([fy0, fy1, fy1, fy0, fy0], [fx0, fx0, fx1, fx1, fx0],
                [z] * 5, color="#4a5060", lw=0.8, ls="--")
    for cx, cy in ((fy0, fx0), (fy1, fx0), (fy1, fx1), (fy0, fx1)):
        ax.plot([cx, cx], [cy, cy], [0, zz], color="#4a5060", lw=0.8, ls="--")
    ax.set_xlim(fy0 - 1, fy1 + 1)
    ax.set_ylim(fx0 - 1, fx1 + 1)
    ax.set_zlim(0, zz * 1.05)
    ax.set_xlabel("east (m)", color="#aab2c0", labelpad=8)
    ax.set_ylabel("north (m)", color="#aab2c0", labelpad=8)
    ax.set_zlabel("altitude (m)", color="#aab2c0", labelpad=6)
    ax.tick_params(colors="#8f98a8", labelsize=7)
    ax.zaxis.set_major_formatter(lambda v, _: f"{v / a.zx:.1f}")
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_facecolor("#1a1d28")
        pane.set_edgecolor("#2a2e3c")
    ax.grid(False)


def draw_mesh(ax):
    if mesh_pts is None:
        return
    z = mesh_pts[:, 2]
    zn = (z - z.min()) / max(0.4, (z.max() - z.min()))
    cols = plt.cm.bone(0.25 + 0.5 * zn)
    cols[:, 3] = 0.32
    ax.scatter(mesh_pts[:, 0], mesh_pts[:, 1], z * a.zx,
               c=cols, s=1.6, linewidths=0, depthshade=False)


def draw_paths(ax, tmax=None):
    for d in DRONES:
        rows = tracks[d]
        if tmax is not None:
            rows = [r for r in rows if r[0] <= tmax]
        if len(rows) < 2:
            continue
        xs = [r[1] for r in rows]
        ys = [r[2] for r in rows]
        zs = [r[3] * a.zx for r in rows]
        ax.plot(xs, ys, [0] * len(xs), color=COLORS[d], lw=0.8, alpha=0.18)
        segs = [[(xs[i], ys[i], zs[i]), (xs[i + 1], ys[i + 1], zs[i + 1])]
                for i in range(len(xs) - 1)]
        lc = Line3DCollection(segs, colors=[COLORS[d]] * len(segs),
                              linewidths=1.8, alpha=0.9)
        ax.add_collection3d(lc)
        ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], color=COLORS[d], s=28,
                   depthshade=False)
        sx, sy = spawns[d]
        ax.scatter([sy], [sx], [0], color=COLORS[d], s=30, marker="s",
                   facecolors="none", depthshade=False)


t0 = min(r[0][0] for r in tracks.values() if r)
t1 = max(r[-1][0] for r in tracks.values() if r)

fig = plt.figure(figsize=(10.8, 8.6), dpi=100)
fig.patch.set_facecolor("#14161e")
ax = fig.add_subplot(111, projection="3d")
setup(ax)
draw_mesh(ax)
draw_paths(ax)
ax.view_init(elev=28, azim=-60)
ax.set_title(f"{a.label} — 3D flown paths, {t1-t0:.0f}s  (altitude exaggerated x{a.zx:.0f})",
             color="white", fontsize=13)
png = f"{run}/path3d_{a.label}.png"
fig.savefig(png, bbox_inches="tight", facecolor=fig.get_facecolor())
print("wrote", png)

if a.anim:
    FPS = 30
    DUR = 24.0
    frames = int(DUR * FPS)
    fig2 = plt.figure(figsize=(9.6, 5.4), dpi=100)
    fig2.patch.set_facecolor("#14161e")
    ax2 = fig2.add_subplot(111, projection="3d")
    mp4 = f"{run}/path3d_{a.label}.mp4"
    w = FFMpegWriter(fps=FPS, bitrate=2200)
    with w.saving(fig2, mp4, dpi=100):
        for k in range(frames):
            ax2.clear()
            setup(ax2)
            draw_mesh(ax2)
            frac = k / (frames - 1)
            draw_paths(ax2, tmax=t0 + frac * (t1 - t0))
            ax2.view_init(elev=26 + 8 * math.sin(2 * math.pi * frac),
                          azim=-60 + 360 * frac)
            ax2.set_title(f"{a.label}  t+{frac*(t1-t0):5.1f}s  (alt exaggerated x{a.zx:.0f})",
                          color="white", fontsize=11)
            w.grab_frame()
    print("wrote", mp4)
