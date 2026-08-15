#!/usr/bin/env python3
"""PHASE-2 (b) OPEN.  Offline analyser for the AirSim simCreateVoxelGrid binvox dumps.

Runs offline so the metric can be iterated without relaunching UE.

BINVOX LAYOUT (decoded from the Blocks control, which is known-good geometry):
  header 'dim 64 16 64' for a request of x=64 y=64 z=16 -> dims are (x, VERTICAL, y)
  data index = x*(dims1*dims2) + k*dims2 + y, with k the vertical index INCREASING UPWARD.
  Blocks per-layer occupancy was  k0-5: 0 | k6-8: 4096 (the floor slab, solid) |
  k9-15: 459-506 (the blocks standing on it) -- empty below the floor mesh, sparse above.

GROUND IS FOUND PER COLUMN, not from an absolute altitude: the floor slab is 3 voxels
thick in Blocks and the AirSim NED origin sits ~1.9 m above the UE floor, so any fixed
"spawn_z + 1 m" rule lands inside the slab and reports zero free space (it did).
  ground_k = highest occupied layer that has >= NEED free layers directly above it,
             restricted to |ground_k - spawn_k| <= NEAR so rooftops do not count as ground
  flyable  = such a layer exists  (i.e. real ground with >= 3 m of clear air over it)
"""
import glob
import json
import os
import sys

import numpy as np

NEED = 3      # metres of clear air required above the ground (the 1-3 m cruise band)
NEAR = 4      # ground must be within this many metres of the spawn plane


def read_binvox(path):
    with open(path, 'rb') as f:
        assert b'#binvox' in f.readline()
        dims = trans = None
        scale = 1.0
        while True:
            l = f.readline().strip()
            if l.startswith(b'dim'):
                dims = [int(v) for v in l.split()[1:]]
            elif l.startswith(b'translate'):
                trans = [float(v) for v in l.split()[1:]]
            elif l.startswith(b'scale'):
                scale = float(l.split()[1])
            elif l.startswith(b'data'):
                break
        raw = f.read()
    vals, i = [], 0
    while i < len(raw) - 1:
        vals.append(np.full(raw[i + 1], raw[i], np.uint8))
        i += 2
    d = np.concatenate(vals) if vals else np.zeros(0, np.uint8)
    n = int(np.prod(dims))
    if d.size < n:
        d = np.concatenate([d, np.zeros(n - d.size, np.uint8)])
    # (x, k, y) -> (x, y, k)
    return d[:n].reshape(dims[0], dims[1], dims[2]).transpose(0, 2, 1), dims, trans, scale


def largest_box(mask):
    nx, ny = mask.shape
    best, area = (0, 0, 0, 0), 0
    h = np.zeros(ny, int)
    for i in range(nx):
        h = np.where(mask[i], h + 1, 0)
        st = []
        for j in range(ny + 1):
            cur = h[j] if j < ny else 0
            start = j
            while st and st[-1][1] > cur:
                sj, sh = st.pop()
                if sh * (j - sj) > area:
                    area = sh * (j - sj)
                    best = (int(sh), int(j - sj), int(i - sh + 1), int(sj))
                start = sj
            st.append((start, cur))
    return best, area


def largest_square(mask):
    nx, ny = mask.shape
    dp = np.zeros((nx, ny), int)
    b = 0
    for i in range(nx):
        for j in range(ny):
            if mask[i, j]:
                dp[i, j] = 1 if (i == 0 or j == 0) else 1 + min(dp[i - 1, j], dp[i, j - 1],
                                                                dp[i - 1, j - 1])
                b = max(b, dp[i, j])
    return int(b)


def analyse(bvx, meta):
    occ, dims, tr, sc = read_binvox(bvx)
    nx, ny, nz = occ.shape
    centre = meta["voxel_centre_ned"]
    base = meta["spawn_pose_ned"]
    GX = GY = 64.0
    GZ = 16.0
    sx, sy, sz = GX / nx, GY / ny, GZ / nz
    x0, y0 = centre[0] - GX / 2.0, centre[1] - GY / 2.0
    # vertical: k increases upward; layer k centre altitude (m, +up) relative to the
    # AirSim origin = -centre_z_ned - GZ/2 + (k+0.5)*sz
    def alt_of_k(k):
        return -centre[2] - GZ / 2.0 + (k + 0.5) * sz
    spawn_alt = -base[2]
    k_spawn = int(round((spawn_alt + centre[2] + GZ / 2.0) / sz - 0.5))

    out = {"voxel_dims": dims, "nx_ny_nz": [nx, ny, nz],
           "occupied": int(occ.sum()), "occupied_pct": round(100.0 * occ.sum() / occ.size, 1),
           "layer_occupancy": occ.sum(axis=(0, 1)).tolist(),
           "k_spawn": k_spawn, "spawn_alt_m": round(spawn_alt, 2),
           "layer_alt_m": [round(alt_of_k(k), 2) for k in range(nz)]}

    fly = np.zeros((nx, ny), bool)
    clearance = np.zeros((nx, ny), int)
    gk = np.full((nx, ny), -1, int)
    for i in range(nx):
        for j in range(ny):
            col = occ[i, j]
            for k in range(min(nz - 1, k_spawn + NEAR), max(-1, k_spawn - NEAR - 1), -1):
                if col[k]:
                    free = 0
                    for m in range(k + 1, nz):
                        if col[m]:
                            break
                        free += 1
                    if free >= NEED:
                        fly[i, j] = True
                        clearance[i, j] = free
                        gk[i, j] = k
                    break
    out["cells_flyable"] = int(fly.sum())
    out["cells_grid_total"] = int(nx * ny)
    cl = clearance[fly]
    if cl.size:
        out["clearance_m_p5_p50_p95"] = [int(np.percentile(cl, 5)), int(np.percentile(cl, 50)),
                                         int(np.percentile(cl, 95))]

    (bx, by, i0, j0), area = largest_box(fly)
    out["largest_flyable_box_m"] = [round(bx * sx, 1), round(by * sy, 1)]
    out["largest_flyable_box_area_m2"] = round(area * sx * sy, 1)
    out["largest_flyable_box_min_side_m"] = round(min(bx * sx, by * sy), 1)
    out["largest_flyable_box_origin_ned_m"] = [round(x0 + i0 * sx, 1), round(y0 + j0 * sy, 1)]
    sq = largest_square(fly)
    out["largest_flyable_square_m"] = round(sq * sx, 1)
    out["fits_25x25"] = (sq * sx) >= 25.0

    def ij(xn, yn):
        return int((xn - x0) / sx), int((yn - y0) / sy)

    at = ao = 0
    for cx in range(-17, 3):
        for cy in range(-17, 12):
            i, j = ij(cx + 0.5, cy + 0.5)
            if 0 <= i < nx and 0 <= j < ny:
                at += 1
                ao += int(fly[i, j])
    out["arena_cells_tested"] = at
    out["arena_cells_flyable"] = ao
    out["arena_flyable_pct"] = round(100.0 * ao / at, 1) if at else 0.0

    line = []
    for yv in (-7, -4, -1, 2):
        i, j = ij(-6.5, yv + 0.5)
        line.append(bool(fly[i, j]) if (0 <= i < nx and 0 <= j < ny) else False)
    out["spawn_line_flyable"] = line
    out["spawn_line_fits_4x3m"] = all(line)
    np.save(bvx.replace('.binvox', '_flymask.npy'), fly)
    return out


rows = []
for d in sorted(glob.glob(sys.argv[1] if len(sys.argv) > 1
                          else '/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/probe_*')):
    if not os.path.isdir(d):
        continue
    lab = os.path.basename(d).replace('probe_', '')
    bvx = '%s/%s.binvox' % (d, lab)
    mj = '%s/probe_%s.json' % (d, lab)
    if not (os.path.exists(bvx) and os.path.exists(mj)):
        continue
    meta = json.load(open(mj))
    try:
        r = analyse(bvx, meta)
    except Exception as e:
        r = {"error": repr(e)}
    r["label"] = lab
    rows.append(r)
    print("%-16s fly=%4d/%4d  box=%sx%s (%s m2, min side %s)  sq=%s  arena=%d/%d (%.0f%%)  line=%s" %
          (lab, r.get("cells_flyable", -1), r.get("cells_grid_total", -1),
           r.get("largest_flyable_box_m", [0, 0])[0], r.get("largest_flyable_box_m", [0, 0])[1],
           r.get("largest_flyable_box_area_m2"), r.get("largest_flyable_box_min_side_m"),
           r.get("largest_flyable_square_m"), r.get("arena_cells_flyable", -1),
           r.get("arena_cells_tested", -1), r.get("arena_flyable_pct", 0),
           r.get("spawn_line_fits_4x3m")))
    print("    layers:", r.get("layer_occupancy"))
json.dump(rows, open('/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/openness.json', 'w'), indent=1)
