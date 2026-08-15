#!/usr/bin/env python3
"""Extract the final merged nvblox mesh from a run's bag into a world-frame
point cloud npz (for the 3D path preview). Latest-per-block accumulation,
per-drone spawn transform (odom FLU -> world east/north)."""
import glob
import json
import os
import sqlite3
import sys

import numpy as np

label = sys.argv[1]
bag = sys.argv[2] if len(sys.argv) > 2 else f"/home/lucas/UE5/hercules-sim-big/mrq_work/mapview/bag_{label}"
out = sys.argv[3] if len(sys.argv) > 3 else f"/home/lucas/hercules-sim/e1_frames/runs/{label}/mesh_cloud_{label}.npz"
S = json.load(open(os.environ.get(
    "HERC_SETTINGS", "/home/lucas/hercules-sim/settings-fleet-4drone-cross.json")))
NAMES = ["ghost", "delta", "buckshee", "thunderstrike"]
spawns = {i + 1: (S["Vehicles"][n.capitalize()] if n.capitalize() in S["Vehicles"]
                  else S["Vehicles"][n]) for i, n in enumerate(NAMES)}


def u32(mv, off):
    return int.from_bytes(mv[off:off + 4], "little")


def parse(data):
    b = memoryview(data)
    off = 4 + 8
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
    blocks = []
    for _ in range(nblk):
        nv = u32(b, off); voff = off + 4; off = voff + nv * 12
        nn = u32(b, off); off = off + 4 + nn * 12
        nc = u32(b, off); off = off + 4 + nc * 16
        nt = u32(b, off); off = off + 4 + nt * 4
        blocks.append(np.frombuffer(b, "<f4", nv * 3, voff).reshape(-1, 3).copy()
                      if nv else None)
    return idx, blocks


db = glob.glob(bag + "/*.db3")[0]
con = sqlite3.connect(db)
tids = {}
for tid, name in con.execute("SELECT id,name FROM topics"):
    for i in range(1, 5):
        if name == f"/d{i}/mesh":
            tids[tid] = i
clouds = []
for tid, dom in tids.items():
    blocks = {}
    n = 0
    for (data,) in con.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        try:
            idx, blks = parse(data)
        except Exception:
            continue
        n += 1
        for j in range(len(blks)):
            key = tuple(idx[j]) if j < len(idx) else ("x", j)
            if blks[j] is not None and len(blks[j]):
                blocks[key] = blks[j]
            else:
                blocks.pop(key, None)
    if not blocks:
        continue
    v = np.concatenate(list(blocks.values()))
    sp = spawns[dom]
    sx, sy = float(sp["X"]), float(sp["Y"])
    east = sy - v[:, 1]
    north = sx + v[:, 0]
    alt = v[:, 2]
    clouds.append(np.stack([east, north, alt], axis=1))
    print(f"d{dom}: {n} msgs, {len(blocks)} blocks, {len(v)} verts")
allv = np.concatenate(clouds)
# de-noise: keep sane band, downsample
allv = allv[(allv[:, 2] > -0.5) & (allv[:, 2] < 4.0)]
if len(allv) > 120000:
    allv = allv[np.random.default_rng(7).choice(len(allv), 120000, replace=False)]
np.savez_compressed(out, cloud=allv.astype(np.float32))
print("wrote", out, len(allv), "pts")
