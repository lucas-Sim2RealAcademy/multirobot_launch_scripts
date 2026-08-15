#!/usr/bin/env python3
"""v3 camera: fix the 'too static' read (user feedback pass 2).

Keeps shot 1 (establishing 0-449) and the hero-track from frame 1290 onward
EXACTLY as rendered (rows copied verbatim from the v3 keys), and replaces ONLY
frames [450,1290) with a NEW SHOT 2b.

Design history: a scene-blind az/dist grid first picked a south-west rig at
(3572,-3784,1736) -- probe frame_0900 showed it INSIDE a foliage curtain, and
no side vantage can hold the 2300 cm-dispersed fleet in frame anyway.  Final
design is a DEPTH-STACK from over the pond (west), on the proven-clean
establishing-camera side: all four drones sit within ~4 deg of one view axis
(thunderstrike near/large 127-224 px, buckshee mid ~70 px, delta + parked
ghost far ~50 px), so the compose window [900,1290) shows the run's only
3-simultaneous-mover peak (~f1055-1100, >40 cm/s) with ALL FOUR drones in
frame.  Gentle dolly P0->P1 (smoothstep over the whole segment, ~220 cm of
travel inside the compose window) + aim tracking the smoothed 3-mover
centroid keeps the shot alive without breaking the framing.

usage: design_camera_v3.py [src=fleet_keys_temple_v3.json] [dst=..._v4.json]
env: P0="2780,-2130,1210" P1="3020,-2470,1360" S2A=900 S2B=1290
"""
import json
import math
import os
import sys

SRC = sys.argv[1] if len(sys.argv) > 1 else "fleet_keys_temple_v3.json"
DST = sys.argv[2] if len(sys.argv) > 2 else "fleet_keys_temple_v4.json"
S2A = int(os.environ.get("S2A", "900"))
S2B = int(os.environ.get("S2B", "1290"))
P0 = [float(v) for v in os.environ.get("P0", "2780,-2130,1210").split(",")]
P1 = [float(v) for v in os.environ.get("P1", "3020,-2470,1360").split(",")]
SEG_LO, SEG_HI = 450, 1290

K = json.load(open(SRC))
N, FPS = K["nframes"], K["fps"]
D = K["drones"]
MOVERS = ["delta", "buckshee", "thunderstrike"]
ALL = ["ghost", "delta", "buckshee", "thunderstrike"]
HFOV2 = math.radians(33.4)
VFOV2 = math.atan(math.tan(HFOV2) * 1080 / 1920)


def moving_avg(vals, half):
    n = len(vals)
    return [sum(vals[max(0, i - half):min(n, i + half + 1)]) /
            (min(n, i + half + 1) - max(0, i - half)) for i in range(n)]


cx = [sum(D[m][k][0] for m in MOVERS) / 3 for k in range(N)]
cy = [sum(D[m][k][1] for m in MOVERS) / 3 for k in range(N)]
cz = [sum(D[m][k][2] for m in MOVERS) / 3 for k in range(N)]
acx, acy, acz = (moving_avg(v, int(1.0 * FPS / 2)) for v in (cx, cy, cz))


def look(cam, tgt):
    dx, dy, dz = tgt[0] - cam[0], tgt[1] - cam[1], tgt[2] - cam[2]
    return (math.degrees(math.atan2(dz, math.hypot(dx, dy))),
            math.degrees(math.atan2(dy, dx)))


def project(cam, pitch, yaw, p):
    dx, dy, dz = p[0] - cam[0], p[1] - cam[1], p[2] - cam[2]
    yr, pr = math.radians(yaw), math.radians(pitch)
    x1 = math.cos(yr) * dx + math.sin(yr) * dy
    y1 = -math.sin(yr) * dx + math.cos(yr) * dy
    x2 = math.cos(pr) * x1 + math.sin(pr) * dz
    z2 = -math.sin(pr) * x1 + math.cos(pr) * dz
    if x2 <= 50:
        return None
    return math.atan2(y1, x2), math.atan2(z2, x2), math.sqrt(dx * dx + dy * dy + dz * dz)


cam = [list(r) for r in K["camera"]]
prev_yaw = None
rows = []
for k in range(SEG_LO, SEG_HI):
    u = (k - SEG_LO) / (SEG_HI - 1 - SEG_LO)
    s = u * u * (3 - 2 * u)                       # smoothstep dolly
    c = [P0[i] + (P1[i] - P0[i]) * s for i in range(3)]
    pitch, yaw = look(c, (acx[k], acy[k], acz[k]))
    if prev_yaw is not None:
        while yaw - prev_yaw > 180:
            yaw -= 360
        while yaw - prev_yaw < -180:
            yaw += 360
    prev_yaw = yaw
    rows.append([c[0], c[1], c[2], pitch, yaw])

# light smoothing INSIDE the segment only (hard cuts at both ends preserved)
for idx, rnd in ((0, 2), (1, 2), (2, 2), (3, 3), (4, 3)):
    seg = moving_avg([r[idx] for r in rows], FPS // 4)
    for i, r in enumerate(rows):
        r[idx] = round(seg[i], rnd)
for k in range(SEG_LO, SEG_HI):
    cam[k] = rows[k - SEG_LO]

K["camera"] = cam
K["info"]["camera_v3"] = {"segment": [SEG_LO, SEG_HI], "score_window": [S2A, S2B],
                          "p0": P0, "p1": P1,
                          "design": "depth-stack dolly over the pond; 3 movers "
                                    "+ parked ghost all in frame; shot1 + "
                                    "hero-track rows untouched"}
json.dump(K, open(DST, "w"))

# ---- verification over the compose window ----
worst = {n: 0.0 for n in ALL}
pxmin = {n: 1e9 for n in ALL}
pxmax = {n: 0.0 for n in ALL}
sp = 0.0
prev = {}
for k in range(S2A, S2B):
    c = cam[k]
    for n in ALL:
        pr = project(c[:3], c[3], c[4], D[n][k])
        assert pr is not None, (n, k)
        worst[n] = max(worst[n], max(abs(pr[0]) / HFOV2, abs(pr[1]) / VFOV2))
        px = 1920 * 100 / (2 * pr[2] * math.tan(HFOV2))
        pxmin[n] = min(pxmin[n], px)
        pxmax[n] = max(pxmax[n], px)
        if n in MOVERS:
            if n in prev:
                sp += math.hypot(pr[0] - prev[n][0], pr[1] - prev[n][1])
            prev[n] = pr
for n in ALL:
    print(f"  {n:14s} worst-frame-frac {worst[n]:.2f}  {pxmin[n]:4.0f}..{pxmax[n]:4.0f} px")
print(f"mover screen-speed sum {sp:.3f} rad over [{S2A},{S2B})")
print(f"cam[{S2A}] = {[round(v,1) for v in cam[S2A]]}")
print(f"cam[{S2B-1}] = {[round(v,1) for v in cam[S2B-1]]}")
print(f"wrote {DST}")
