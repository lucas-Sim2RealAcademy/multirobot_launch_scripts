#!/usr/bin/env python3
"""Export HERCULES fleet ground-truth trajectories (meta_*.json) to UE keyframes.

Reads a run archive (e1_frames/runs/<label>/<label>_<veh>/meta_*.json), converts
each drone's per-vehicle NED pose to UE world space in JapanFest_Street:

    UE_cm = ANCHOR + (spawn_offset_m + ned_m) * (100, 100, -100)

where spawn offsets come from the AirSim settings JSON (per drone) and ANCHOR is
the map's PlayerStart (AirSim world origin) at (-1006, -119, 2348) cm; the road
plane sits at Z=2250 cm.

Smooths (moving average), picks the most active window, resamples to 30 fps, and
designs a lateral-dolly CineCamera path that aims at the smoothed fleet centroid.

Output JSON: {fps, nframes, t0_epoch, t1_epoch, drones: {name: [[x,y,z,yaw_deg]..]},
              camera: [[x,y,z,pitch_deg,yaw_deg]..], info: {...}}

Usage: export_keys.py <run_dir> <settings.json> <out.json> [window_seconds]
"""
import json
import math
import sys
import glob

ANCHOR = (-1006.0, -119.0, 2348.0)   # PlayerStart, cm (verified in-editor by build_fleet_seq.py)
ROAD_Z = 2250.0                      # cm
FPS = 30
VEHS = ["ghost", "delta", "buckshee", "thunderstrike"]
CAM_SIDE_Y = -1050.0                 # cm: south of the spawn line (Y=-419), inside the flown corridor
CAM_Z = ROAD_Z + 430.0               # 4.3 m above the road
CAM_MARGIN_X = 350.0                 # cm beyond centroid range at both ends
SMOOTH_S = 0.9                       # position smoothing window (s) on the 4.5 Hz samples
CENTROID_SMOOTH_S = 3.0              # heavy smoothing for the camera aim


def moving_avg(vals, half):
    n = len(vals)
    out = []
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        out.append(sum(vals[lo:hi]) / (hi - lo))
    return out


def load_track(run_dir, label, veh):
    files = sorted(glob.glob(f"{run_dir}/{label}_{veh}/meta_*.json"))
    t, x, y, z, yaw = [], [], [], [], []
    for f in files:
        m = json.load(open(f))
        n = m["ned"]
        t.append(m["t"]); x.append(n[0]); y.append(n[1]); z.append(n[2]); yaw.append(n[3])
    # unwrap yaw
    for i in range(1, len(yaw)):
        while yaw[i] - yaw[i - 1] > math.pi:
            yaw[i] -= 2 * math.pi
        while yaw[i] - yaw[i - 1] < -math.pi:
            yaw[i] += 2 * math.pi
    return t, x, y, z, yaw


def interp(ts, vs, t):
    """linear interp, clamped."""
    if t <= ts[0]:
        return vs[0]
    if t >= ts[-1]:
        return vs[-1]
    import bisect
    i = bisect.bisect_right(ts, t) - 1
    f = (t - ts[i]) / (ts[i + 1] - ts[i])
    return vs[i] + f * (vs[i + 1] - vs[i])


def main():
    run_dir, settings_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    window_s = float(sys.argv[4]) if len(sys.argv) > 4 else 80.0
    label = run_dir.rstrip("/").split("/")[-1]

    settings = json.load(open(settings_path))
    spawns = {v: settings["Vehicles"][v] for v in VEHS}

    tracks = {}
    for v in VEHS:
        t, x, y, z, yaw = load_track(run_dir, label, v)
        rate = 1.0 / ((t[-1] - t[0]) / (len(t) - 1))
        half = max(1, int(round(SMOOTH_S * rate / 2)))
        x, y, z = moving_avg(x, half), moving_avg(y, half), moving_avg(z, half)
        yaw = moving_avg(yaw, half)
        sp = spawns[v]
        wx = [ANCHOR[0] + (sp["X"] + xi) * 100.0 for xi in x]
        wy = [ANCHOR[1] + (sp["Y"] + yi) * 100.0 for yi in y]
        wz = [ANCHOR[2] - (sp.get("Z", 0) + zi) * 100.0 for zi in z]
        wyaw = [math.degrees(yi) for yi in yaw]
        tracks[v] = (t, wx, wy, wz, wyaw)

    t0_all = max(tracks[v][0][0] for v in VEHS)
    t1_all = min(tracks[v][0][-1] for v in VEHS)
    dur = t1_all - t0_all
    window_s = min(window_s, math.floor(dur))

    # --- pick the most active window: total fleet speed integrated per 1 s bucket ---
    nb = int(dur)
    act = [0.0] * nb
    for v in VEHS:
        t, wx, wy, wz, _ = tracks[v]
        for i in range(1, len(t)):
            b = int(t[i] - t0_all)
            if 0 <= b < nb:
                act[b] += math.dist((wx[i], wy[i], wz[i]), (wx[i - 1], wy[i - 1], wz[i - 1]))
    best_s, best_v = 0, -1.0
    w = int(window_s)
    for s in range(0, nb - w + 1):
        vsum = sum(act[s:s + w])
        if vsum > best_v:
            best_v, best_s = vsum, s
    t0 = t0_all + best_s
    t1 = t0 + window_s
    nframes = int(window_s * FPS)

    # --- resample all drones to 30 fps ---
    drones = {}
    for v in VEHS:
        t, wx, wy, wz, wyaw = tracks[v]
        rows = []
        for k in range(nframes):
            tk = t0 + k / FPS
            rows.append([round(interp(t, wx, tk), 2), round(interp(t, wy, tk), 2),
                         round(interp(t, wz, tk), 2), round(interp(t, wyaw, tk), 3)])
        drones[v] = rows

    # --- camera: FOLLOW mode. The street is covered/overhung in places, so a
    # hand-placed dolly can end up inside geometry (verified by diag renders).
    # The drones' flown corridor is guaranteed free space, so the camera rides
    # the heavily smoothed, time-delayed path of the farthest-travelling drone
    # (slightly above it) and aims at the smoothed fleet centroid. ---
    cx = [sum(drones[v][k][0] for v in VEHS) / 4 for k in range(nframes)]
    cy = [sum(drones[v][k][1] for v in VEHS) / 4 for k in range(nframes)]
    cz = [sum(drones[v][k][2] for v in VEHS) / 4 for k in range(nframes)]
    half = int(CENTROID_SMOOTH_S * FPS / 2)
    cx, cy, cz = moving_avg(cx, half), moving_avg(cy, half), moving_avg(cz, half)

    def path_len(v):
        t, wx, wy, wz, _ = tracks[v]
        return sum(math.dist((wx[i], wy[i]), (wx[i - 1], wy[i - 1])) for i in range(1, len(t)))
    carrier = max(VEHS, key=path_len)
    ct, cwx, cwy, cwz, _ = tracks[carrier]
    # heavy smoothing of the carrier track (5 s at sample rate)
    crate = 1.0 / ((ct[-1] - ct[0]) / (len(ct) - 1))
    chalf = max(1, int(round(5.0 * crate / 2)))
    cwx, cwy, cwz = moving_avg(cwx, chalf), moving_avg(cwy, chalf), moving_avg(cwz, chalf)

    CAM_DELAY_S = 3.3
    CAM_Z_LIFT = 50.0
    cam = []
    prev_yaw = None
    for k in range(nframes):
        tk = t0 + k / FPS - CAM_DELAY_S
        px = interp(ct, cwx, tk)
        py = interp(ct, cwy, tk)
        pz = interp(ct, cwz, tk) + CAM_Z_LIFT
        dx, dy, dz = cx[k] - px, cy[k] - py, cz[k] - pz
        yaw = math.degrees(math.atan2(dy, dx))
        if prev_yaw is not None:
            while yaw - prev_yaw > 180:
                yaw -= 360
            while yaw - prev_yaw < -180:
                yaw += 360
        prev_yaw = yaw
        pitch = max(-30.0, min(10.0, math.degrees(math.atan2(dz, math.hypot(dx, dy)))))
        cam.append([round(px, 2), round(py, 2), round(pz, 2), round(pitch, 3), round(yaw, 3)])
    # light smoothing of the final camera curve to kill any interp kinks
    for idx in range(3):
        vals = moving_avg([c[idx] for c in cam], FPS // 2)
        for k in range(nframes):
            cam[k][idx] = round(vals[k], 2)
    for idx in (3, 4):
        vals = moving_avg([c[idx] for c in cam], FPS // 2)
        for k in range(nframes):
            cam[k][idx] = round(vals[k], 3)

    # --- sanity ---
    print(f"run={label} usable [{t0_all:.2f},{t1_all:.2f}] dur={dur:.1f}s")
    print(f"window [{t0:.2f},{t1:.2f}] = {window_s:.0f}s, offset +{best_s}s, activity={best_v:.0f}cm")
    for v in VEHS:
        r = drones[v]
        zs = [p[2] for p in r]
        xs = [p[0] for p in r]
        ys = [p[1] for p in r]
        print(f"  {v}: X[{min(xs):.0f},{max(xs):.0f}] Y[{min(ys):.0f},{max(ys):.0f}] "
              f"Z[{min(zs):.0f},{max(zs):.0f}] (road {ROAD_Z:.0f}; AGL {min(zs)-ROAD_Z:.0f}..{max(zs)-ROAD_Z:.0f} cm)")
    pxs = [c[0] for c in cam]; pys = [c[1] for c in cam]; pzs = [c[2] for c in cam]
    print(f"  cam(follow {carrier}, delay {CAM_DELAY_S}s): X[{min(pxs):.0f},{max(pxs):.0f}] "
          f"Y[{min(pys):.0f},{max(pys):.0f}] Z[{min(pzs):.0f},{max(pzs):.0f}]")

    out = {"fps": FPS, "nframes": nframes, "t0_epoch": t0, "t1_epoch": t1,
           "drones": drones, "camera": cam,
           "info": {"run": label, "anchor": ANCHOR, "road_z": ROAD_Z,
                    "window_offset_s": best_s, "window_s": window_s}}
    json.dump(out, open(out_path, "w"))
    print(f"wrote {out_path} nframes={nframes}")


if __name__ == "__main__":
    main()
