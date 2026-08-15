#!/usr/bin/env python3
"""PHASE-2 (a) COST, load-normalised.

A sibling agent is running its own UE on this host, so load1 swung 7 -> 34 across the
sweep and raw milliseconds are not comparable between maps measured minutes apart.
Two defences:

  1. REFERENCE CALL.  Every burst also times simGetVehiclePose, an RPC that crosses the
     same socket and the same game thread but issues NO draw calls.  Host contention
     inflates both; scene draw-call cost inflates only the stereo call.  The reported
     cost is the RATIO stereo_ms / pose_ms, which divides the contention out.
  2. MINIMUM STATISTIC.  p10 and min of each burst, not the mean: the fastest samples are
     the ones that happened to land in a quiet slice.

Anchor: the same script on Blocks gives the incumbent's ratio, and every candidate is
reported as a multiple of it.  JapanFest is included as the known-rejected calibration
point (it cost 1.39x Blocks in the original isolated probe).

usage: timeonly.py <label> <outdir> [--port 41471] [--burst 80] [--reps 3]
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, '/home/lucas/hercules-sim/HERCULES/PythonClient')
import hercules_cosysairsim as airsim   # noqa: E402

LABEL, OUT = sys.argv[1], sys.argv[2]
PORT = int(sys.argv[sys.argv.index('--port') + 1]) if '--port' in sys.argv else 41471
BURST = int(sys.argv[sys.argv.index('--burst') + 1]) if '--burst' in sys.argv else 80
REPS = int(sys.argv[sys.argv.index('--reps') + 1]) if '--reps' in sys.argv else 3
os.makedirs(OUT, exist_ok=True)
VEH = 'ghost'

c = airsim.MultirotorClient(port=PORT)
c.confirmConnection()
REQ = [airsim.ImageRequest('front_left', airsim.ImageType.Scene, False, False),
       airsim.ImageRequest('front_right', airsim.ImageType.Scene, False, False)]


def load1():
    return float(open('/proc/loadavg').read().split()[0])


sp = c.simGetVehiclePose(VEH)
base = np.array([sp.position.x_val, sp.position.y_val, sp.position.z_val])

# hover the fleet centroid at 1.5 m so the frustum sees representative scene, not the
# inside of whatever the drone is resting on
p = airsim.Pose(airsim.Vector3r(float(base[0]), float(base[1]) + 4.5, float(base[2]) - 1.5),
                airsim.euler_to_quaternion(0.0, 0.0, 0.0))
c.enableApiControl(True, VEH)
c.simSetVehiclePose(p, True, VEH)
time.sleep(0.5)

# warm-up: shader/texture residency, otherwise the first bursts score the warm-up
for _ in range(200):
    c.simGetImages(REQ, vehicle_name=VEH)

res = {"label": LABEL, "port": PORT, "burst": BURST, "reps": REPS,
       "spawn_pose_ned": [round(float(x), 2) for x in base], "reps_detail": []}
S, P = [], []
for r in range(REPS):
    l0 = load1()
    ts = []
    for _ in range(BURST):
        t0 = time.perf_counter()
        c.simGetImages(REQ, vehicle_name=VEH)
        ts.append((time.perf_counter() - t0) * 1000.0)
    tp = []
    for _ in range(BURST):
        t0 = time.perf_counter()
        c.simGetVehiclePose(VEH)
        tp.append((time.perf_counter() - t0) * 1000.0)
    ts, tp = np.array(ts), np.array(tp)
    S.append(ts)
    P.append(tp)
    d = {"rep": r, "load1": l0,
         "stereo_p10": round(float(np.percentile(ts, 10)), 3),
         "stereo_p50": round(float(np.percentile(ts, 50)), 3),
         "stereo_min": round(float(ts.min()), 3),
         "pose_p10": round(float(np.percentile(tp, 10)), 3),
         "pose_p50": round(float(np.percentile(tp, 50)), 3),
         "pose_min": round(float(tp.min()), 3)}
    d["ratio_p10"] = round(d["stereo_p10"] / d["pose_p10"], 2) if d["pose_p10"] else None
    d["ratio_min"] = round(d["stereo_min"] / d["pose_min"], 2) if d["pose_min"] else None
    res["reps_detail"].append(d)
    print("rep%d load=%5.2f stereo p10=%7.2f min=%7.2f | pose p10=%6.3f min=%6.3f | "
          "ratio p10=%6.2f" % (r, l0, d["stereo_p10"], d["stereo_min"], d["pose_p10"],
                               d["pose_min"], d["ratio_p10"]), flush=True)

A, B = np.concatenate(S), np.concatenate(P)
res.update(stereo_p10=round(float(np.percentile(A, 10)), 3),
           stereo_p50=round(float(np.percentile(A, 50)), 3),
           stereo_min=round(float(A.min()), 3),
           pose_p10=round(float(np.percentile(B, 10)), 3),
           pose_p50=round(float(np.percentile(B, 50)), 3),
           pose_min=round(float(B.min()), 3),
           load1_end=load1())
res["ratio_p10"] = round(res["stereo_p10"] / res["pose_p10"], 2)
res["ratio_min"] = round(res["stereo_min"] / res["pose_min"], 2)
res["best_rep_stereo_p10"] = round(min(d["stereo_p10"] for d in res["reps_detail"]), 3)
json.dump(res, open('%s/time_%s.json' % (OUT, LABEL), 'w'), indent=1)
print("TIMESUMMARY " + json.dumps({k: v for k, v in res.items() if k != "reps_detail"}))
