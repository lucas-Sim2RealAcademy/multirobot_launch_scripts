#!/usr/bin/env python3
"""Bake the mrq6 fleet coverage into a 64x64 RGBA16 first-seen-time texture.

Cell model = fidelity_scorecard.sh A6 block EXACTLY: 1m cells via int()
truncation of spawn-corrected NED (n->cx, e->cy), 4m sensing disc as the
49-offset mask dx*dx+dy*dy<=16. UNCLIPPED union (spec: 1242 cells expected).

Texture (64x64 RGBA16, row 0 = top = originY):
  R = first_t normalized over the SHARED DOMAIN [t0-PRE, t1]
  G = first_toucher id * 16384 (ids: ghost 0, delta 1, buckshee 2, thunder 3)
  B = 0
  A = 65535 if first_t <= t1 else 0   (post-window cells never render)

coverage_meta.json: origin, PRE, counts, transform constants, clock keys.
"""
import glob
import json
import math
import struct
import zlib

RUN = "/home/lucas/hercules-sim/e1_frames/runs/mrq6"
SETTINGS = "/home/lucas/hercules-sim/settings-fleet-4drone.json"
KEYS = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple/fleet_keys_temple.json"
OUT_PNG = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple/coverage_mrq6.png"
OUT_META = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple/coverage_meta.json"

VEHS = ["ghost", "delta", "buckshee", "thunderstrike"]      # id order 0..3
DISC = [(dx, dy) for dx in range(-4, 5) for dy in range(-4, 5)
        if dx * dx + dy * dy <= 16]

K = json.load(open(KEYS))
T0 = K["t0_epoch"]
T1 = K["t1_epoch"]
assert abs((T1 - T0) - 80.0) < 1e-6

spawn = {v: (float(d.get("X", 0)), float(d.get("Y", 0)))
         for v, d in json.load(open(SETTINGS))["Vehicles"].items()}

# ---- all samples of all drones, time-sorted -> first (t, drone) per cell ----
samples = []
for vid, v in enumerate(VEHS):
    for m in sorted(glob.glob(f"{RUN}/mrq6_{v}/meta_*.json")):
        try:
            j = json.load(open(m))
        except Exception:
            continue
        ned = j.get("ned") or []
        if len(ned) < 2:
            continue
        samples.append((float(j["t"]), vid, float(ned[0]), float(ned[1])))
samples.sort()
print(f"samples: {len(samples)}  t[{samples[0][0]:.2f},{samples[-1][0]:.2f}]")

first = {}
for t, vid, n, e in samples:
    ox, oy = spawn[VEHS[vid]]
    cx = int(n + ox)                     # scorecard: int() truncation
    cy = int(e + oy)
    for dx, dy in DISC:
        c = (cx + dx, cy + dy)
        if c not in first:
            first[c] = (t, vid)

xs = [c[0] for c in first]
ys = [c[1] for c in first]
n_pre = sum(1 for t, _ in first.values() if t < T0)
n_win = sum(1 for t, _ in first.values() if T0 <= t <= T1)
n_post = sum(1 for t, _ in first.values() if t > T1)
tmin = min(t for t, _ in first.values())
PRE = T0 - tmin
DUR = PRE + 80.0
print(f"cells: {len(first)}  bbox X[{min(xs)},{max(xs)}] Y[{min(ys)},{max(ys)}]")
print(f"pre-window {n_pre}  in-window {n_win}  post {n_post}  PRE={PRE:.2f}s")

OX, OY = min(xs), min(ys)
assert max(xs) - OX < 64 and max(ys) - OY < 64, "bbox exceeds 64x64"

# ---- write RGBA16 PNG (hand-rolled; row j = cell originY+j, v axis = cy) ----
rows = []
for j in range(64):
    row = bytearray()
    for i in range(64):
        c = (OX + i, OY + j)
        if c in first:
            t, vid = first[c]
            r = max(0, min(65535, int(round((t - tmin) / DUR * 65535))))
            g = vid * 16384
            a = 65535 if t <= T1 else 0
        else:
            r = g = a = 0
        row += struct.pack(">HHHH", r, g, 0, a)
    rows.append(bytes(row))


def chunk(tag, data):
    return (struct.pack(">I", len(data)) + tag + data +
            struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


png = (b"\x89PNG\r\n\x1a\n" +
       chunk(b"IHDR", struct.pack(">IIBBBBB", 64, 64, 16, 6, 0, 0, 0)) +
       chunk(b"IDAT", zlib.compress(b"".join(b"\x00" + r for r in rows), 9)) +
       chunk(b"IEND", b""))
open(OUT_PNG, "wb").write(png)
print(f"wrote {OUT_PNG} ({len(png)} bytes)")

# ---- transform constants (must match transform_keys_temple.py exactly) ----
tt = K["info"]["temple_transform"]
ax, ay, agz = tt["anchor"]
yaw = math.radians(tt["yaw_deg"])
pvx, pvy = tt["pivot_blocks"]
zlift = tt["zlift"]

# temple bbox of the painted cells (for decal placement): corners of cell bbox
BAX, BAY = 1370.0, 1340.0                # blocks anchor (export_keys.py)


def cell_to_temple(cx, cy):
    bx = BAX + cx * 100.0
    by = BAY + cy * 100.0
    dx, dy = bx - pvx, by - pvy
    return (ax + math.cos(yaw) * dx - math.sin(yaw) * dy,
            ay + math.sin(yaw) * dx + math.cos(yaw) * dy)


corners = [cell_to_temple(x, y) for x in (OX, max(xs) + 1) for y in (OY, max(ys) + 1)]
ctx = (min(c[0] for c in corners) + max(c[0] for c in corners)) / 2
cty = (min(c[1] for c in corners) + max(c[1] for c in corners)) / 2
half = max(max(c[0] for c in corners) - ctx, max(c[1] for c in corners) - cty)

meta = {
    "origin_cell": [OX, OY],
    "pre_s": PRE, "dur_s": DUR, "t0": T0, "t1": T1, "tmin": tmin,
    "counts": {"total": len(first), "pre": n_pre, "in": n_win, "post": n_post},
    "blocks_anchor": [BAX, BAY],
    "pivot": [pvx, pvy],
    "translate": [ax, ay],
    "cos_yaw": math.cos(yaw), "sin_yaw": math.sin(yaw),
    "ground_z": agz, "zlift": zlift,
    "decal_center": [round(ctx, 1), round(cty, 1), agz],
    "decal_half_xy": round(half + 100, 0),
    "clock_keys": {"-3": 0.0, "0": 0.0,
                   "60": round((PRE + 2.0) / DUR, 6), "2400": 1.0},
}
json.dump(meta, open(OUT_META, "w"), indent=1)
print(json.dumps(meta["clock_keys"]))
print(f"decal center ({ctx:.0f},{cty:.0f}) half {half + 100:.0f}")
print("BAKE_OK")
