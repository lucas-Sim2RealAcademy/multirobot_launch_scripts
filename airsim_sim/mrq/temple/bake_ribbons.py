#!/usr/bin/env python3
"""Bake per-drone reveal-ribbon OBJs, world-space (temple cm), time as
TEXCOORD0.u over the SHARED DOMAIN [t0-PRE, t1] (u = (t - tmin)/DUR).

In-window geometry comes from the same fleet_keys_temple.json rows that drive
the drone transform tracks (cannot desync); pre-window extension from the meta
NEDs through the same blocks->temple closed form, smoothed 0.9s like
export_keys.py. Strip = X-profile (two perpendicular planes) so it never
vanishes edge-on; points decimated to every 2nd frame (15/s).
"""
import glob
import json
import math

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple"
RUN = "/home/lucas/hercules-sim/e1_frames/runs/mrq6"
SETTINGS = "/home/lucas/hercules-sim/settings-fleet-4drone.json"

K = json.load(open(f"{W}/fleet_keys_temple.json"))
M = json.load(open(f"{W}/coverage_meta.json"))
T0, TMIN, DUR = M["t0"], M["tmin"], M["dur_s"]
tt = K["info"]["temple_transform"]
ax, ay = tt["anchor"][0], tt["anchor"][1]
agz, zlift = tt["anchor"][2], tt["zlift"]
cy_, sy_ = M["cos_yaw"], M["sin_yaw"]
pvx, pvy = M["pivot"]
BAX, BAY, BAZ = 1370.0, 1340.0, 190.0
FPS = 30
WIDTH = 16.0            # cm ribbon half... full width 16 -> half 8
ZOFF = -30.0

spawn = {v: (float(d.get("X", 0)), float(d.get("Y", 0)), float(d.get("Z", 0)))
         for v, d in json.load(open(SETTINGS))["Vehicles"].items()}


def blocks_to_temple(bx, by, bz):
    dx, dy = bx - pvx, by - pvy
    return (ax + cy_ * dx - sy_ * dy, ay + sy_ * dx + cy_ * dy,
            agz + (bz - 100.0) + zlift)


def moving_avg(vals, half):
    n = len(vals)
    return [sum(vals[max(0, i - half):min(n, i + half + 1)]) /
            (min(n, i + half + 1) - max(0, i - half)) for i in range(n)]


def interp(ts, vs, t):
    if t <= ts[0]:
        return vs[0]
    if t >= ts[-1]:
        return vs[-1]
    import bisect
    i = bisect.bisect_right(ts, t) - 1
    f = (t - ts[i]) / (ts[i + 1] - ts[i])
    return vs[i] + f * (vs[i + 1] - vs[i])


for name, rows in K["drones"].items():
    # ---- pre-window: meta ned -> temple, smoothed, resampled 15/s ----
    t_, x_, y_, z_ = [], [], [], []
    for m in sorted(glob.glob(f"{RUN}/mrq6_{name}/meta_*.json")):
        try:
            j = json.load(open(m))
        except Exception:
            continue
        ned = j.get("ned") or []
        if len(ned) < 3:
            continue
        t_.append(float(j["t"]))
        x_.append(float(ned[0]))
        y_.append(float(ned[1]))
        z_.append(float(ned[2]))
    rate = 1.0 / ((t_[-1] - t_[0]) / (len(t_) - 1))
    half = max(1, int(round(0.9 * rate / 2)))
    x_, y_, z_ = moving_avg(x_, half), moving_avg(y_, half), moving_avg(z_, half)
    sx, sy, sz = spawn[name]
    pts = []                                     # (u, X, Y, Z) temple cm
    tp = TMIN
    while tp < T0:
        n = interp(t_, x_, tp)
        e = interp(t_, y_, tp)
        d = interp(t_, z_, tp)
        bx = BAX + (sx + n) * 100.0
        by = BAY + (sy + e) * 100.0
        bz = BAZ - (sz + d) * 100.0
        X, Y, Z = blocks_to_temple(bx, by, bz)
        pts.append(((tp - TMIN) / DUR, X, Y, Z + ZOFF))
        tp += 2.0 / FPS
    # ---- in-window: straight from the baked drone keys, every 2nd frame ----
    pre_n = len(pts)
    for f in range(0, len(rows), 2):
        r = rows[f]
        pts.append(((T0 - TMIN + f / FPS) / DUR, r[0], r[1], r[2] + ZOFF))

    # ---- X-profile strip OBJ ----
    v_lines, vt_lines, f_lines = [], [], []

    def emit_strip(offsets):
        base = len(v_lines)
        for i, (u, X, Y, Z) in enumerate(pts):
            j0, j1 = max(0, i - 1), min(len(pts) - 1, i + 1)
            tx = pts[j1][1] - pts[j0][1]
            tyy = pts[j1][2] - pts[j0][2]
            o = offsets(tx, tyy)
            # NOTE: OBJ Y is NEGATED -- UE Interchange mirrors Y on import
            # (smoke test: v(0,200,0) imported at y=-200). Two-sided material
            # makes the flipped winding irrelevant.
            v_lines.append(f"v {X + o[0]:.1f} {-(Y + o[1]):.1f} {Z + o[2]:.1f}")
            v_lines.append(f"v {X - o[0]:.1f} {-(Y - o[1]):.1f} {Z - o[2]:.1f}")
            vt_lines.append(f"vt {u:.6f} 0.0")
            vt_lines.append(f"vt {u:.6f} 1.0")
            if i:
                a, b = base + 2 * i - 1, base + 2 * i
                c, d = base + 2 * i + 1, base + 2 * i + 2
                f_lines.append(f"f {a}/{a} {b}/{b} {d}/{d} {c}/{c}")

    def horiz(tx, tyy):                          # perpendicular in XY
        L = math.hypot(tx, tyy) or 1.0
        return (-tyy / L * WIDTH / 2, tx / L * WIDTH / 2, 0.0)

    emit_strip(horiz)
    emit_strip(lambda tx, tyy: (0.0, 0.0, WIDTH / 2))   # vertical plane

    with open(f"{W}/ribbon_{name}.obj", "w") as fo:
        fo.write("o ribbon_%s\n" % name)
        fo.write("\n".join(v_lines) + "\n")
        fo.write("\n".join(vt_lines) + "\n")
        fo.write("\n".join(f_lines) + "\n")
    us = [p[0] for p in pts]
    print(f"{name}: {len(pts)} pts ({pre_n} pre + {len(pts)-pre_n} win), "
          f"u[{min(us):.4f},{max(us):.4f}], verts {len(v_lines)}")
print("RIBBONS_OK")
