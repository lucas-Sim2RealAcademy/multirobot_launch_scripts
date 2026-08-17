#!/usr/bin/env python3
"""Per-drone map panels: what each drone individually mapped + explored,
then the merged result.  Answers "show a segment of each drone".

usage: make_perdrone_preview.py <label> [--half-m 12]
"""
import argparse
import glob
import json
import os
import sqlite3

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

COLORS = {"ghost": "#3f8cff", "delta": "#3fff72",
          "buckshee": "#ff4cf2", "thunderstrike": "#ff941e"}
DRONES = list(COLORS)

ap = argparse.ArgumentParser()
ap.add_argument("label")
ap.add_argument("--half-m", type=float,
                default=float(os.environ.get("HERC_ARENA_HALF_M", "12")))
ap.add_argument("--runs-dir", default="/home/lucas/hercules-sim/e1_frames/runs")
ap.add_argument("--bag-dir",
                default="/home/lucas/UE5/hercules-sim-big/mrq_work/mapview")
a = ap.parse_args()

S = json.load(open(os.environ.get(
    "HERC_SETTINGS", "/home/lucas/hercules-sim/settings-bamboo-4drone.json")))
spawns = {}
for i, n in enumerate(DRONES):
    key = n.capitalize() if n.capitalize() in S["Vehicles"] else n
    v = S["Vehicles"][key]
    spawns[i + 1] = (float(v["X"]), float(v["Y"]))


def u32(mv, off):
    return int.from_bytes(mv[off:off + 4], "little")


def parse(data):
    b = memoryview(data)
    off = 12
    slen = u32(b, off)
    off += 4 + slen
    off = 4 + (((off - 4) + 3) & ~3)
    off += 4
    nbi = u32(b, off)
    off += 4
    idx = np.frombuffer(b, "<i4", nbi * 3, off).reshape(-1, 3)
    off += nbi * 12
    nblk = u32(b, off)
    off += 4
    out = []
    for _ in range(nblk):
        nv = u32(b, off); voff = off + 4; off = voff + nv * 12
        nn = u32(b, off); off += 4 + nn * 12
        nc = u32(b, off); off += 4 + nc * 16
        nt = u32(b, off); off += 4 + nt * 4
        out.append(np.frombuffer(b, "<f4", nv * 3, voff).reshape(-1, 3).copy()
                   if nv else None)
    return idx, out


# ---- per-drone mesh clouds (world frame) ----
db = glob.glob(f"{a.bag_dir}/bag_{a.label}/*.db3")[0]
con = sqlite3.connect(db)
tids = {t: int(n[2]) for t, n in con.execute("SELECT id,name FROM topics")
        if n.endswith("/mesh")}
clouds = {}
for tid, dom in sorted(tids.items()):
    blocks = {}
    for (data,) in con.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        try:
            idx, blks = parse(data)
        except Exception:
            continue
        for j, bl in enumerate(blks):
            k = tuple(idx[j]) if j < len(idx) else ("x", j)
            if bl is not None and len(bl):
                blocks[k] = bl
    if not blocks:
        continue
    v = np.concatenate(list(blocks.values()))
    sx, sy = spawns[dom]
    clouds[DRONES[dom - 1]] = np.stack([sy - v[:, 1], sx + v[:, 0], v[:, 2]], axis=1)

# ---- per-drone tracks ----
tracks = {}
for i, d in enumerate(DRONES):
    rows = []
    for f in sorted(glob.glob(f"{a.runs_dir}/{a.label}/{a.label}_{d}/meta_*.json")):
        m = json.load(open(f))
        n = m.get("ned")
        if n and (abs(n[0]) + abs(n[1]) > 1e-6):
            sx, sy = spawns[i + 1]
            rows.append((sy + n[1], sx + n[0]))
    tracks[d] = rows

H = a.half_m
xs = [s[1] for s in spawns.values()]
ys = [s[0] for s in spawns.values()]
fx0, fx1 = min(xs) - H, max(xs) + H
fy0, fy1 = min(ys) - H, max(ys) + H

fig, axes = plt.subplots(1, 5, figsize=(22, 5.0), dpi=100)
fig.patch.set_facecolor("#14161e")


def panel(ax, title, drones, color_mesh=True):
    ax.set_facecolor("#14161e")
    for d in drones:
        c = clouds.get(d)
        if c is None or not len(c):
            continue
        m = c[(c[:, 2] > 0.2) & (c[:, 2] < 9)]
        if len(m) > 40000:
            m = m[np.random.default_rng(3).choice(len(m), 40000, replace=False)]
        ax.scatter(m[:, 0], m[:, 1], s=0.6, linewidths=0,
                   c=(COLORS[d] if color_mesh else "#8f98a8"), alpha=0.35)
    for d in drones:
        t = tracks.get(d) or []
        if len(t) > 1:
            ax.plot([p[0] for p in t], [p[1] for p in t],
                    color=COLORS[d], lw=1.6, alpha=0.95)
        sx, sy = spawns[DRONES.index(d) + 1]
        ax.plot([sy], [sx], marker="s", ms=6, mfc="none", mec=COLORS[d], mew=1.4)
    ax.set_xlim(fy0 - 1, fy1 + 1)
    ax.set_ylim(fx0 - 1, fx1 + 1)
    ax.set_aspect("equal")
    ax.set_title(title, color="white", fontsize=11)
    ax.tick_params(colors="#8f98a8", labelsize=7)
    for s in ax.spines.values():
        s.set_color("#3a3f4c")


for i, d in enumerate(DRONES):
    n_pts = len(clouds.get(d, []))
    panel(axes[i], f"{d.upper()}\n{n_pts:,} map points", [d])
panel(axes[4], "MERGED\nall four maps", DRONES, color_mesh=True)
fig.suptitle(f"{a.label} — what each drone mapped, and the shared result",
             color="white", fontsize=13)
out = f"{a.runs_dir}/{a.label}/perdrone_{a.label}.png"
fig.savefig(out, bbox_inches="tight", facecolor=fig.get_facecolor())
print("wrote", out)
