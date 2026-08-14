#!/usr/bin/env python3
"""v2 camera for the temple fleet cinematic (user feedback: v1 low chase read
awkward; drones tiny/occluded).

Two-shot design in ONE camera track (hard jump at the cut frame):
  SHOT 1 [0, CUT): establishing wide from over the pond, gentle push-in,
      framing the east-shore hero complex (stone head + facade) with ghost,
      delta and thunderstrike in frame.
  SHOT 2 [CUT, N): elevated 3/4 TRACKING shot. The 4-drone centroid is nearly
      static (ghost+delta barely move), so the camera hero-tracks THUNDERSTRIKE
      (longest path, ends at the east shore): camera WEST of the hero
      (azimuth AZ0->AZ1 drift), D lateral, H above the ground plane, looking at
      the smoothed hero -> pitch ~ -26 deg, hero ~120 px, silhouetted against
      the shore ruins across the water.

Reads fleet_keys_temple.json, writes fleet_keys_temple_v2.json (drones
unchanged, camera replaced). Prints design stats for verification.
Env knobs: CUT=450 AZ0=200 AZ1=175 D0=1100 D1=950 H=780
           E_CAM="3600,-2600,2202" E_TGT="5400,-1600,950" E_PUSH=0.12
"""
import json
import math
import os
import sys

SRC = sys.argv[1] if len(sys.argv) > 1 else "fleet_keys_temple.json"
DST = sys.argv[2] if len(sys.argv) > 2 else "fleet_keys_temple_v2.json"
K = json.load(open(SRC))
N = K["nframes"]
FPS = K["fps"]
GZ = K["info"]["road_z"]                      # 802 = temple ground plane

CUT = int(os.environ.get("CUT", "450"))
AZ0 = float(os.environ.get("AZ0", "200"))
AZ1 = float(os.environ.get("AZ1", "175"))
D0 = float(os.environ.get("D0", "1100"))
D1 = float(os.environ.get("D1", "950"))
H = float(os.environ.get("H", "780"))
E_CAM = [float(v) for v in os.environ.get("E_CAM", "3600,-2600,2202").split(",")]
E_TGT = [float(v) for v in os.environ.get("E_TGT", "5400,-1600,950").split(",")]
E_PUSH = float(os.environ.get("E_PUSH", "0.12"))

hero = K["drones"]["thunderstrike"]


def moving_avg(vals, half):
    n = len(vals)
    return [sum(vals[max(0, i - half):min(n, i + half + 1)]) /
            (min(n, i + half + 1) - max(0, i - half)) for i in range(n)]


# heavy-smoothed hero for the camera POSITION (2.5 s), light for the AIM (0.8 s)
hx = [r[0] for r in hero]; hy = [r[1] for r in hero]; hz = [r[2] for r in hero]
phx, phy, phz = (moving_avg(v, int(2.5 * FPS / 2)) for v in (hx, hy, hz))
ahx, ahy, ahz = (moving_avg(v, int(0.8 * FPS / 2)) for v in (hx, hy, hz))


def look(cam, tgt, prev_yaw):
    dx, dy, dz = tgt[0] - cam[0], tgt[1] - cam[1], tgt[2] - cam[2]
    yaw = math.degrees(math.atan2(dy, dx))
    if prev_yaw is not None:
        while yaw - prev_yaw > 180:
            yaw -= 360
        while yaw - prev_yaw < -180:
            yaw += 360
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    return pitch, yaw


cam = []
prev_yaw = None
for k in range(N):
    if k < CUT:                                # ---- establishing push-in ----
        u = k / max(1, CUT - 1)
        s = u * u * (3 - 2 * u) * E_PUSH       # smoothstep 0..E_PUSH
        c = [E_CAM[i] + (E_TGT[i] - E_CAM[i]) * s for i in range(3)]
        pitch, yaw = look(c, E_TGT, prev_yaw)
    else:                                      # ---- elevated 3/4 tracking ----
        u = (k - CUT) / max(1, N - 1 - CUT)
        az = math.radians(AZ0 + (AZ1 - AZ0) * u)
        d = D0 + (D1 - D0) * u
        c = [phx[k] + d * math.cos(az), phy[k] + d * math.sin(az), phz[k] - 1035 + GZ + H]
        tgt = [ahx[k], ahy[k], ahz[k]]
        pitch, yaw = look(c, tgt, prev_yaw)
    prev_yaw = yaw
    cam.append([c[0], c[1], c[2], pitch, yaw])

# light smoothing INSIDE each shot only (never across the cut -> keep it hard)
for lo, hi in ((0, CUT), (CUT, N)):
    for idx, r in ((0, 2), (1, 2), (2, 2), (3, 3), (4, 3)):
        seg = moving_avg([cam[k][idx] for k in range(lo, hi)], FPS // 4)
        for k in range(lo, hi):
            cam[k][idx] = round(seg[k - lo], r)

K["camera"] = cam
K["info"]["camera_v2"] = {"cut": CUT, "az": [AZ0, AZ1], "d": [D0, D1], "h": H,
                          "e_cam": E_CAM, "e_tgt": E_TGT, "e_push": E_PUSH,
                          "design": "15s establishing wide + elevated 3/4 hero-track"}
json.dump(K, open(DST, "w"))

# ---- stats ----
sl = [math.dist(cam[k][:3], (ahx[k], ahy[k], ahz[k])) for k in range(CUT, N)]
pi = [cam[k][3] for k in range(CUT, N)]
px_ = [1920 * 100 / (2 * s * math.tan(math.radians(33.4))) for s in sl]
print(f"shot1: cam {[round(v) for v in cam[0][:3]]} -> {[round(v) for v in cam[CUT-1][:3]]} "
      f"pitch {cam[0][3]:.1f}..{cam[CUT-1][3]:.1f}")
print(f"shot2: slant {min(sl):.0f}..{max(sl):.0f} cm  pitch {min(pi):.1f}..{max(pi):.1f} deg  "
      f"hero px {min(px_):.0f}..{max(px_):.0f}")
print(f"shot2 cam X[{min(c[0] for c in cam[CUT:]):.0f},{max(c[0] for c in cam[CUT:]):.0f}] "
      f"Y[{min(c[1] for c in cam[CUT:]):.0f},{max(c[1] for c in cam[CUT:]):.0f}] "
      f"Z[{min(c[2] for c in cam[CUT:]):.0f},{max(c[2] for c in cam[CUT:]):.0f}]")
print(f"wrote {DST}")
