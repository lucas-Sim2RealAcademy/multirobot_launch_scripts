#!/usr/bin/env python3
"""SHOT 6 <-> RViz map continuity check (EXECUTION-PLAN-V6 §2.7 / §5.1).

Finds the four per-drone colored markers in (a) the UE SHOT-6 first frame and
(b) an RViz reshot screenshot, and verifies the 4/4 endpoint QUADRANT match
(sign of screen offset from the 4-endpoint centroid) plus the footprint
screen-extent ratio <= 20%.

Colors: UE drones are grey quads, so for the UE side we use the RIBBON tip
colors near the drones... in practice the UE side quadrants were computed
analytically at design time (camera_v6_arc.s6.quadrants in the keys json);
here we only extract the RViz side and compare against that record.

usage: check_s6_continuity.py <keys_final.json> <rviz_shot.png>
"""
import json
import sys

import numpy as np
from PIL import Image

KEYS, SHOT = sys.argv[1], sys.argv[2]
K = json.load(open(KEYS))
quads_ue = K["info"]["camera_v6_arc"]["s6"]["quadrants"]

# augment_replay marker colors (mapview/augment_replay.py DRONES table)
COLORS = {"ghost": (64, 140, 255), "delta": (64, 255, 115),
          "buckshee": (255, 76, 242), "thunderstrike": (255, 148, 30)}

img = np.asarray(Image.open(SHOT).convert("RGB"), dtype=np.int16)
H, W, _ = img.shape
pos = {}
for v, (r, g, b) in COLORS.items():
    d = (np.abs(img[:, :, 0] - r) + np.abs(img[:, :, 1] - g) +
         np.abs(img[:, :, 2] - b))
    mask = d < 120
    if mask.sum() < 8:
        print(f"  {v}: NOT FOUND on rviz shot ({int(mask.sum())} px)")
        pos[v] = None
        continue
    ys, xs = np.nonzero(mask)
    pos[v] = (float(xs.mean()), float(ys.mean()))
    print(f"  {v}: rviz marker at ({pos[v][0]:.0f},{pos[v][1]:.0f}) "
          f"{int(mask.sum())}px")

found = {v: p for v, p in pos.items() if p is not None}
if len(found) < 4:
    print("CONTINUITY: INCONCLUSIVE (markers missing)")
    sys.exit(2)
cx = sum(p[0] for p in found.values()) / 4
cy = sum(p[1] for p in found.values()) / 4
match = 0
for v, p in found.items():
    q_rv = [1 if p[0] > cx else -1, 1 if (cy - p[1]) > 0 else -1]  # +y up
    q_ue = quads_ue[v]
    ok = q_rv == q_ue
    match += ok
    print(f"  {v}: rviz_quad={q_rv} ue_quad={q_ue} {'OK' if ok else 'MISMATCH'}")

# footprint extent ratio: rviz pixel bbox diag vs UE-projected diag is checked
# as a spread ratio between the two marker sets (normalized by frame width)
xs = [p[0] for p in found.values()]
ys = [p[1] for p in found.values()]
rv_diag = ((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2) ** 0.5 / W

# UE-side endpoint spread from the keys (s6 first frame, analytic projection)
sys.path.insert(0, "/home/lucas/UE5/hercules-sim-big/mrq_work/temple")
from shot_qc import project, screen_px, DRONES
s6 = [s for s in K["info"]["shots"] if s["name"] == "s6_mapmatch"][0]
A6, B6 = s6["start"], s6["end"]
c0 = K["camera"][A6]
ue_pts = []
for v in DRONES:
    pr = project(c0[:3], c0[3], c0[4], K["drones"][v][B6 - 1])
    if pr:
        ue_pts.append(screen_px(pr))
uxs = [p[0] for p in ue_pts]
uys = [p[1] for p in ue_pts]
ue_diag = ((max(uxs) - min(uxs)) ** 2 + (max(uys) - min(uys)) ** 2) ** 0.5 / 1920.0
ratio = abs(rv_diag - ue_diag) / max(rv_diag, ue_diag)
print(f"  endpoint spread: rviz={rv_diag:.3f} ue={ue_diag:.3f} frame-widths "
      f"(ratio delta {100*ratio:.0f}%)")
extent_ok = ratio <= 0.20
print(f"CONTINUITY: {match}/4 quadrant match, extent "
      f"{'OK' if extent_ok else 'OVER'} "
      f"-> {'PASS' if match == 4 and extent_ok else 'FAIL'}")
sys.exit(0 if (match == 4 and extent_ok) else 3)
