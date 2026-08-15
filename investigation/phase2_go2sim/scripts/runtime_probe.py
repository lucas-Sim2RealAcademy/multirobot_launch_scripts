#!/usr/bin/env python3
"""PHASE-2 combined probe, run against a live -game AirSim process.  All three criteria.

Editor-commandlet collision queries proved unreliable here (three passes disagreed, and a
trace that "hit" the Blocks ground in one pass missed the same column in isolation), so
OPEN is measured in the real game world via simCreateVoxelGrid -- AirSim's own occupancy
rasteriser, the same collision the drones will actually fly against.

(a) COST     stereo round-trip (front_left+front_right Scene, ONE simGetImages call) at
             every probe pose, single RPC connection, 640x480.
(b) OPEN     64x64x16 m voxel grid at 1 m res centred on the fleet spawn ->
             flyable = free at spawn+1/2/3 m AND some occupied voxel below within 8 m;
             largest clear box / largest square / 630-cell arena count / 4-drone line.
(c) TEXTURE  L/R/depth .npy at each pose for measure_features.py.

usage: runtime_probe.py <label> <outdir> [--port 41471] [--burst 40]
"""
import json
import os
import struct
import sys
import time

import numpy as np

sys.path.insert(0, '/home/lucas/hercules-sim/HERCULES/PythonClient')
import hercules_cosysairsim as airsim   # noqa: E402

LABEL, OUT = sys.argv[1], sys.argv[2]
PORT = int(sys.argv[sys.argv.index('--port') + 1]) if '--port' in sys.argv else 41471
BURST = int(sys.argv[sys.argv.index('--burst') + 1]) if '--burst' in sys.argv else 40
os.makedirs(OUT, exist_ok=True)
VEH = 'ghost'

GRID_M = 64          # voxel grid side, metres (covers the +-30 m survey and the arena)
GRID_Z = 16
RES = 1.0
HEIGHTS = [1.0, 2.0, 3.0]
FLOOR_LOOK_M = 8


def load1():
    return float(open('/proc/loadavg').read().split()[0])


def read_binvox(path):
    """Minimal binvox reader (RLE, values 0/1).  Returns (data[x][y][z], dims, tr, sc)."""
    with open(path, 'rb') as f:
        line = f.readline().strip()
        if b'#binvox' not in line:
            raise IOError('not binvox: %r' % line)
        dims = trans = None
        scale = 1.0
        while True:
            line = f.readline().strip()
            if line.startswith(b'dim'):
                dims = [int(v) for v in line.split()[1:]]
            elif line.startswith(b'translate'):
                trans = [float(v) for v in line.split()[1:]]
            elif line.startswith(b'scale'):
                scale = float(line.split()[1])
            elif line.startswith(b'data'):
                break
        raw = f.read()
    vals = []
    i = 0
    while i < len(raw) - 1:
        v, cnt = raw[i], raw[i + 1]
        vals.append(np.full(cnt, v, np.uint8))
        i += 2
    d = np.concatenate(vals) if vals else np.zeros(0, np.uint8)
    n = dims[0] * dims[1] * dims[2]
    if d.size < n:
        d = np.concatenate([d, np.zeros(n - d.size, np.uint8)])
    # binvox index order is x-major, then z, then y  (wxyz: for x, for z, for y)
    return d[:n].reshape(dims[0], dims[2], dims[1]).transpose(0, 2, 1), dims, trans, scale


def largest_box(mask):
    nx, ny = mask.shape
    best, area_best = (0, 0, 0, 0), 0
    heights = np.zeros(ny, int)
    for i in range(nx):
        heights = np.where(mask[i], heights + 1, 0)
        stack = []
        for j in range(ny + 1):
            h = heights[j] if j < ny else 0
            start = j
            while stack and stack[-1][1] > h:
                sj, sh = stack.pop()
                if sh * (j - sj) > area_best:
                    area_best = sh * (j - sj)
                    best = (int(sh), int(j - sj), int(i - sh + 1), int(sj))
                start = sj
            stack.append((start, h))
    return best, area_best


def largest_square(mask):
    """Classic DP maximal all-True square."""
    nx, ny = mask.shape
    dp = np.zeros((nx, ny), int)
    best = 0
    for i in range(nx):
        for j in range(ny):
            if mask[i, j]:
                dp[i, j] = 1 if (i == 0 or j == 0) else 1 + min(dp[i - 1, j], dp[i, j - 1],
                                                                dp[i - 1, j - 1])
                best = max(best, dp[i, j])
    return int(best)


c = airsim.MultirotorClient(port=PORT)
c.confirmConnection()
res = {"label": LABEL, "port": PORT, "burst_n": BURST, "load1_start": load1()}

sp = c.simGetVehiclePose(VEH)
base = np.array([sp.position.x_val, sp.position.y_val, sp.position.z_val])
res["spawn_pose_ned"] = [round(float(x), 2) for x in base]

# ================= (b) OPEN =================
# Voxel grid is centred on the FLEET centroid (spawns are X=-7, Y=-7..+2), not on ghost,
# so the 630-cell arena sits inside it.
centre = airsim.Vector3r(float(base[0]) + 0.0, float(base[1]) + 4.5, float(base[2]))
bvx = os.path.abspath('%s/%s.binvox' % (OUT, LABEL))
t0 = time.time()
try:
    okv = c.simCreateVoxelGrid(centre, GRID_M, GRID_M, GRID_Z, RES, bvx)
except Exception as e:
    okv = False
    res["voxel_error"] = str(e)
res["voxel_ok"] = bool(okv)
res["voxel_secs"] = round(time.time() - t0, 1)
res["voxel_centre_ned"] = [round(centre.x_val, 2), round(centre.y_val, 2),
                           round(centre.z_val, 2)]

# Analysis of the grid is offline in voxan.py: the ground slab is several voxels thick
# and the AirSim origin sits above it, so ground must be found per column, not from a
# fixed spawn_z + 1 m rule (that rule reported zero free space on Blocks).
if okv and os.path.exists(bvx):
    res["voxel_bytes"] = os.path.getsize(bvx)

# ================= poses for (a)+(c) =================
# Anchor on the flyable box centre when the spawn line itself is unusable, so cost and
# texture are measured over real arena rather than over a void.
cxx, cyy = float(base[0]), float(base[1]) + 4.5   # fleet centroid
res["probe_anchor_ned"] = [round(cxx, 1), round(cyy, 1)]

POSES = [("q0_anchor", (cxx, cyy, base[2] - 1.5), 0),
         ("q1_e", (cxx + 4, cyy, base[2] - 1.5), 0),
         ("q2_n", (cxx, cyy + 4, base[2] - 1.5), 90),
         ("q3_w", (cxx - 4, cyy, base[2] - 1.5), 180),
         ("q4_s", (cxx, cyy - 4, base[2] - 1.5), 270),
         ("q5_hi", (cxx + 2, cyy + 2, base[2] - 3.0), 45),
         ("q6_lo", (cxx - 2, cyy - 2, base[2] - 0.8), 225),
         ("q7_spawn", (float(base[0]), float(base[1]), base[2] - 1.5), 0)]

REQ_STEREO = [airsim.ImageRequest('front_left', airsim.ImageType.Scene, False, False),
              airsim.ImageRequest('front_right', airsim.ImageType.Scene, False, False)]
REQ_TRIPLE = REQ_STEREO + [airsim.ImageRequest('front_center',
                                               airsim.ImageType.DepthPlanar, True, False)]

c.enableApiControl(True, VEH)
# WARM-UP: Blocks' first four poses read 20.9/33.6/24.8/32.9 ms and then settled at
# 13.2-13.5 ms for the rest -- shader and texture residency, not scene cost.  Timing any
# map before that settles would score its warm-up, not its draw-call load.
for _ in range(200):
    c.simGetImages(REQ_STEREO, vehicle_name=VEH)
res["warmup_calls"] = 200
per, all_st = [], []
for name, xyz, yawdeg in POSES:
    p = airsim.Pose(airsim.Vector3r(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                    airsim.euler_to_quaternion(0.0, 0.0, np.deg2rad(float(yawdeg))))
    c.simSetVehiclePose(p, True, VEH)
    time.sleep(0.4)
    l0 = load1()
    for _ in range(10):
        c.simGetImages(REQ_STEREO, vehicle_name=VEH)
    t = []
    for _ in range(BURST):
        t1 = time.perf_counter()
        c.simGetImages(REQ_STEREO, vehicle_name=VEH)
        t.append((time.perf_counter() - t1) * 1000.0)
    st = np.array(t)
    all_st.append(st)
    c.simSetVehiclePose(p, True, VEH)
    time.sleep(0.2)
    r = c.simGetImages(REQ_TRIPLE, vehicle_name=VEH)
    ok = len(r) >= 3 and r[0].height > 0 and r[2].height > 0
    if ok:
        for nm, rr in zip(('L', 'R'), r[:2]):
            im = np.frombuffer(rr.image_data_uint8, np.uint8).reshape(rr.height, rr.width, 3)
            g = ((im[:, :, 0].astype(np.uint32) * 4899 + im[:, :, 1].astype(np.uint32) * 9617
                  + im[:, :, 2].astype(np.uint32) * 1868) >> 14).astype(np.uint8)
            np.save('%s/%s_%s.npy' % (OUT, name, nm), g)
        d = np.array(r[2].image_data_float, np.float32).reshape(r[2].height, r[2].width)
        np.save('%s/%s_depth.npy' % (OUT, name), d)
    e = {"pose": name, "ned": [round(float(v), 1) for v in xyz], "yaw": yawdeg,
         "captured": bool(ok), "load1": l0,
         "stereo_ms_p50": round(float(np.percentile(st, 50)), 2),
         "stereo_ms_p10": round(float(np.percentile(st, 10)), 2),
         "stereo_ms_p90": round(float(np.percentile(st, 90)), 2)}
    if ok:
        e["depth_mid_m"] = round(float(d[240, 320]), 2)
        e["depth_bot_m"] = round(float(d[460, 320]), 2)
    per.append(e)
    print("%-10s stereo_p50=%6.2f load1=%4.2f cap=%s" % (name, e["stereo_ms_p50"], l0, ok),
          flush=True)

res["per_pose"] = per
if all_st:
    a = np.concatenate(all_st)
    p50s = [float(np.percentile(s, 50)) for s in all_st]
    res["stereo_ms_all_p50"] = round(float(np.percentile(a, 50)), 2)
    res["stereo_ms_all_p90"] = round(float(np.percentile(a, 90)), 2)
    res["stereo_ms_pose_median"] = round(float(np.median(p50s)), 2)
    res["stereo_ms_pose_min"] = round(float(np.min(p50s)), 2)
    res["stereo_ms_pose_max"] = round(float(np.max(p50s)), 2)
res["load1_end"] = load1()
json.dump(res, open('%s/probe_%s.json' % (OUT, LABEL), 'w'), indent=1)
print("SUMMARY " + json.dumps({k: v for k, v in res.items() if k != "per_pose"}))
