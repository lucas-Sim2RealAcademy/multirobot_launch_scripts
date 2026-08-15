#!/usr/bin/env python3
"""Verify the fleet's nvblox reconstruction against UE ground truth.

Overlays GT obstacle wireframes (Blocks map, PlayerStart-referenced) on the
mesh cloud and reports accuracy (point-to-box distance) + completeness
(GT obstacles inside the fence that got mapped).
usage: verify_map_gt.py <label>
"""
import json
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

label = sys.argv[1]
GT = json.load(open("/home/lucas/hercules-sim/tools/gt/blocks_gt.json"))
ps = GT["player_start"]
cloud = np.load(f"/home/lucas/hercules-sim/e1_frames/runs/{label}/mesh_cloud_{label}.npz")["cloud"]
S = json.load(open("/home/lucas/hercules-sim/settings-fleet-4drone-cross.json"))
spawns = {v: (c["X"], c["Y"]) for v, c in S["Vehicles"].items()}
H = 12.0
xs = [x for x, _ in spawns.values()]
ys = [y for _, y in spawns.values()]
fn0, fn1 = min(xs) - H, max(xs) + H      # north bounds
fe0, fe1 = min(ys) - H, max(ys) + H      # east bounds
GROUND_UE_Z = 100.0

# GT boxes -> arena frame (north, east, alt-AGL meters)
gt = []
for b in GT["boxes"]:
    if b.get("big"):
        continue
    n0 = (b["min"][0] - ps[0]) / 100.0
    n1 = (b["max"][0] - ps[0]) / 100.0
    e0 = (b["min"][1] - ps[1]) / 100.0
    e1 = (b["max"][1] - ps[1]) / 100.0
    a0 = (b["min"][2] - GROUND_UE_Z) / 100.0
    a1 = (b["max"][2] - GROUND_UE_Z) / 100.0
    # keep obstacles overlapping the fence and rising above 0.15m AGL
    if n1 < fn0 - 1 or n0 > fn1 + 1 or e1 < fe0 - 1 or e0 > fe1 + 1:
        continue
    if a1 < 0.15:
        continue
    gt.append({"label": b["label"], "n": (n0, n1), "e": (e0, e1), "a": (max(0.0, a0), a1)})
print(f"GT obstacles in fence: {len(gt)}")

# mesh points in obstacle band
mp = cloud[(cloud[:, 2] > 0.15) & (cloud[:, 2] < 3.2)]
mp = mp[(mp[:, 0] > fe0) & (mp[:, 0] < fe1) & (mp[:, 1] > fn0) & (mp[:, 1] < fn1)]
print(f"mesh points in band+fence: {len(mp)}")

def dist_to_box(p, g):
    de = max(g["e"][0] - p[0], 0, p[0] - g["e"][1])
    dn = max(g["n"][0] - p[1], 0, p[1] - g["n"][1])
    da = max(g["a"][0] - p[2], 0, p[2] - g["a"][1])
    return (de * de + dn * dn + da * da) ** 0.5

# accuracy: distance from each mesh point to the nearest GT box surface
sub = mp[np.random.default_rng(1).choice(len(mp), min(6000, len(mp)), replace=False)]
dists = []
for p in sub:
    dists.append(min(dist_to_box(p, g) for g in gt))
dists = np.array(dists)
print(f"ACCURACY  point->nearest-GT-box: median {np.median(dists):.2f}m  "
      f"p90 {np.percentile(dists, 90):.2f}m  frac<0.3m {np.mean(dists < 0.3):.2f}")

# completeness: GT obstacles with >=30 mesh points within 0.4m
mapped = 0
det = []
for g in gt:
    c = 0
    for p in sub:
        if dist_to_box(p, g) < 0.4:
            c += 1
    det.append((g["label"], c))
    if c >= 8:
        mapped += 1
print(f"COMPLETENESS  {mapped}/{len(gt)} fence obstacles mapped (>=8 nearby pts in 6k sample)")
for lbl, c in sorted(det, key=lambda t: -t[1]):
    print(f"   {lbl:24s} nearby_pts={c}")

# overlay figure
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa
fig = plt.figure(figsize=(10.8, 8.6), dpi=100)
fig.patch.set_facecolor("#14161e")
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("#14161e")
ZX = 2.0
z = mp[:, 2]
zn = (z - z.min()) / max(0.4, z.max() - z.min())
cols = plt.cm.bone(0.25 + 0.5 * zn)
cols[:, 3] = 0.3
step = max(1, len(mp) // 50000)
ax.scatter(mp[::step, 0], mp[::step, 1], mp[::step, 2] * ZX, c=cols[::step], s=1.6,
           linewidths=0, depthshade=False)
for g in gt:
    e0, e1 = g["e"]; n0, n1 = g["n"]; a0, a1 = g["a"][0] * ZX, min(g["a"][1], 3.2) * ZX
    for aa in (a0, a1):
        ax.plot([e0, e1, e1, e0, e0], [n0, n0, n1, n1, n0], [aa] * 5,
                color="#ff5555", lw=1.2, alpha=0.9)
    for ce, cn in ((e0, n0), (e1, n0), (e1, n1), (e0, n1)):
        ax.plot([ce, ce], [cn, cn], [a0, a1], color="#ff5555", lw=1.2, alpha=0.9)
ax.set_xlim(fe0 - 1, fe1 + 1)
ax.set_ylim(fn0 - 1, fn1 + 1)
ax.set_zlim(0, 3.2 * ZX)
ax.set_xlabel("east (m)", color="#aab2c0")
ax.set_ylabel("north (m)", color="#aab2c0")
ax.set_zlabel("alt (m)", color="#aab2c0")
ax.zaxis.set_major_formatter(lambda v, _: f"{v / ZX:.1f}")
ax.tick_params(colors="#8f98a8", labelsize=7)
for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
    pane.set_facecolor("#1a1d28"); pane.set_edgecolor("#2a2e3c")
ax.grid(False)
ax.view_init(elev=32, azim=-55)
ax.set_title(f"{label}: fleet map (grey) vs ground truth (red wireframes)",
             color="white", fontsize=12)
out = f"/home/lucas/hercules-sim/e1_frames/runs/{label}/map_vs_gt_{label}.png"
fig.savefig(out, bbox_inches="tight", facecolor=fig.get_facecolor())
print("wrote", out)
