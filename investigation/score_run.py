#!/usr/bin/env python3
"""Selection score for fleet cinematic runs (EXECUTION-PLAN-V6 §1.5-1.6).

SCORE = U + 1.5*P_min + 120*F_min - 0.8*T_stall
  U       fleet unique cells (GATED variant if present) from the scorecard
  P_min   min per-drone XY path length (m) from consecutive meta ned deltas
  F_min   min per-drone moving fraction (|v| > 0.3 m/s between metas)
  T_stall max per-drone longest CONSECUTIVE same-goal streak (s) from planner logs

Gates (reject outright):
  G1  run archived non-INVALID + PLAN OK in all 4 planner logs
  G2  no fleet_drones_excluded row in the scorecard (odom divergence <= 2x fence)
  G3  bag has >= 300 /dN/mesh msgs per drone (bag metadata.yaml)
  G4  stereo >= 16 Hz all drones ("--- sensors" lines in run_<label>.out)

Camera-worthiness floors for a winner: P_min >= 30, F_min >= 0.15, T_stall <= 90.

--timeline writes stall_timeline_<label>.json (per-drone start/end of every
same-goal streak > 20s, absolute + flight-relative times) into the run dir.

usage: score_run.py <label> [--runs-dir D] [--bag B] [--timeline] [--no-gates]
"""
import argparse
import glob
import json
import math
import os
import re
import sys

DRONES = ["ghost", "delta", "buckshee", "thunderstrike"]
RUNS_DEFAULT = "/home/lucas/hercules-sim/e1_frames/runs"
BAG_BASE = "/home/lucas/UE5/hercules-sim-big/mrq_work/mapview"
SIM_BASE = "/home/lucas/hercules-sim"
V_MOVING = 0.3          # m/s
STREAK_MIN_TIMELINE = 20.0

GOAL_RE = re.compile(r"goal=\(([^)]*)\)")
TS_RE = re.compile(r"\[(\d+\.\d+)\]")
SENSOR_RE = re.compile(r"--- sensors (\w+):.*?stereo ([\d.]+) Hz")


def load_tracks(rundir, label):
    tracks = {}
    for v in DRONES:
        pts = []
        for m in sorted(glob.glob(os.path.join(rundir, f"{label}_{v}", "meta_*.json"))):
            try:
                j = json.load(open(m))
            except Exception:
                continue
            ned = j.get("ned") or []
            if len(ned) < 2:
                continue
            pts.append((float(j.get("t", 0.0)), float(ned[0]), float(ned[1])))
        pts.sort()
        tracks[v] = pts
    return tracks


def path_and_moving(pts):
    """XY path length (m) and moving fraction (|v|>0.3 between metas)."""
    if len(pts) < 2:
        return 0.0, 0.0
    path = 0.0
    moving = 0
    segs = 0
    for (t0, x0, y0), (t1, x1, y1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        dt = t1 - t0
        path += d
        if dt <= 0:
            continue
        segs += 1
        if d / dt > V_MOVING:
            moving += 1
    return path, (moving / segs if segs else 0.0)


def goal_streaks(logpath):
    """List of (goal_str, t_start, t_end) for consecutive same-goal runs."""
    streaks = []
    cur = None
    t0 = t1 = None
    try:
        f = open(logpath, errors="replace")
    except OSError:
        return streaks
    for line in f:
        g = GOAL_RE.search(line)
        if not g:
            continue
        ts = TS_RE.search(line)
        if not ts:
            continue
        t = float(ts.group(1))
        goal = g.group(1)
        if goal == cur:
            t1 = t
        else:
            if cur is not None:
                streaks.append((cur, t0, t1))
            cur, t0, t1 = goal, t, t
    if cur is not None:
        streaks.append((cur, t0, t1))
    return streaks


def parse_scorecard(path):
    """Return (U, excluded_list, divergence_rows). GATED unique preferred."""
    u_plain = u_gated = None
    excluded = []
    div = {}
    if not os.path.exists(path):
        return None, ["<no scorecard>"], div
    for line in open(path, errors="replace"):
        f = line.split()
        if not f:
            continue
        if f[0] == "fleet_unique_m2" and len(f) >= 3:
            u_plain = float(f[2])
        elif f[0] == "fleet_unique_m2_gated" and len(f) >= 3:
            u_gated = float(f[2])
        elif f[0] == "fleet_drones_excluded" and len(f) >= 3:
            excluded = f[2].split(",")
        elif f[0].startswith("odom_divergence_m[") and len(f) >= 5:
            v = f[0][len("odom_divergence_m["):-1]
            div[v] = (f[3], line.strip().split()[-1])
    return (u_gated if u_gated is not None else u_plain), excluded, div


def parse_stereo(run_out):
    rates = {}
    if not os.path.exists(run_out):
        return rates
    for line in open(run_out, errors="replace"):
        m = SENSOR_RE.search(line)
        if m:
            rates[m.group(1)] = float(m.group(2))   # last line per drone wins
    return rates


def bag_mesh_counts(bagdir):
    meta = os.path.join(bagdir, "metadata.yaml")
    counts = {}
    if not os.path.exists(meta):
        return None
    name = None
    for line in open(meta):
        s = line.strip()
        if s.startswith("name: /"):
            name = s.split("name: ")[1]
        elif s.startswith("message_count:") and name and re.fullmatch(r"/d\d/mesh", name):
            counts[name] = int(s.split(":")[1])
            name = None
    return counts


def spawn_offsets(scorecard_path):
    """Spawn X,Y per drone from the settings file named in the scorecard."""
    fname = None
    if os.path.exists(scorecard_path):
        for line in open(scorecard_path, errors="replace"):
            if line.startswith("fleet_spawn_offsets"):
                fname = line.split()[3]      # cols: metric, "settings", "json", file
                break
    if not fname:
        return None
    p = fname if os.path.isabs(fname) else os.path.join(SIM_BASE, fname)
    if not os.path.exists(p):
        return None
    try:
        s = json.load(open(p))
        return {v: (float(c.get("X", 0)), float(c.get("Y", 0)))
                for v, c in s.get("Vehicles", {}).items()}
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label")
    ap.add_argument("--runs-dir", default=RUNS_DEFAULT)
    ap.add_argument("--bag", default=None)
    ap.add_argument("--timeline", action="store_true")
    ap.add_argument("--no-gates", action="store_true",
                    help="score only (for calibration on old runs without bags)")
    args = ap.parse_args()

    label = args.label
    rundir = os.path.join(args.runs_dir, label)
    if not os.path.isdir(rundir):
        alt = rundir + "_INVALID"
        if os.path.isdir(alt):
            print(f"G1 FAIL: run archived as {label}_INVALID — rejected")
            sys.exit(2)
        print(f"no run dir: {rundir}")
        sys.exit(2)

    # ---- components -------------------------------------------------------
    scorecard = os.path.join(rundir, f"scorecard_{label}.txt")
    U, excluded, div = parse_scorecard(scorecard)
    if U is None:
        print("FATAL: no fleet_unique_m2 in scorecard")
        sys.exit(2)

    tracks = load_tracks(rundir, label)
    per = {}
    for v in DRONES:
        p, f = path_and_moving(tracks.get(v, []))
        per[v] = {"path_m": p, "moving_frac": f, "metas": len(tracks.get(v, []))}

    t_stall = {}
    worst_streak = {}
    all_streaks = {}
    for v in DRONES:
        st = goal_streaks(os.path.join(rundir, f"planner_{label}_{v}.log"))
        durs = [(t1 - t0, g, t0, t1) for g, t0, t1 in st]
        all_streaks[v] = st
        if durs:
            d, g, t0, t1 = max(durs)
            t_stall[v] = d
            worst_streak[v] = (g, t0, t1)
        else:
            t_stall[v] = 0.0

    P_min = min(per[v]["path_m"] for v in DRONES)
    F_min = min(per[v]["moving_frac"] for v in DRONES)
    T_stall = max(t_stall.values())
    SCORE = U + 1.5 * P_min + 120.0 * F_min - 0.8 * T_stall

    # flight t0 = earliest meta timestamp (fan-out beat reference)
    t0_fleet = min((tr[0][0] for tr in tracks.values() if tr), default=None)

    # ---- gates ------------------------------------------------------------
    gates = {}
    plan_ok = {}
    for v in DRONES:
        lp = os.path.join(rundir, f"planner_{label}_{v}.log")
        n = 0
        if os.path.exists(lp):
            n = sum(1 for line in open(lp, errors="replace") if "PLAN OK" in line)
        plan_ok[v] = n
    gates["G1"] = (not label.endswith("_INVALID")) and all(plan_ok[v] >= 1 for v in DRONES)
    gates["G2"] = not excluded
    bagdir = args.bag or os.path.join(BAG_BASE, f"bag_{label}")
    mesh = bag_mesh_counts(bagdir)
    if mesh is None:
        gates["G3"] = None if args.no_gates else False
    else:
        gates["G3"] = len(mesh) == 4 and all(c >= 300 for c in mesh.values())
    stereo = parse_stereo(os.path.join(rundir, f"run_{label}.out"))
    gates["G4"] = (len(stereo) == 4 and all(r >= 16.0 for r in stereo.values())) \
        if stereo else (None if args.no_gates else False)

    floors = {"P_min>=30": P_min >= 30.0, "F_min>=0.15": F_min >= 0.15,
              "T_stall<=90": T_stall <= 90.0}

    # ---- arena bbox (world frame = spawn + ned) ---------------------------
    sp = spawn_offsets(scorecard)
    bbox = None
    if sp:
        xs, ys = [], []
        for v in DRONES:
            ox, oy = sp.get(v, (0.0, 0.0))
            for _, x, y in tracks.get(v, []):
                xs.append(ox + x)
                ys.append(oy + y)
        if xs:
            bbox = (min(xs), max(xs), min(ys), max(ys))

    # ---- outwardness (rank runs by outward coverage; report-only, SCORE
    # untouched so historical comparisons stay valid) -----------------------
    outward = None
    if sp:
        cx = sum(sp.get(v, (0.0, 0.0))[0] for v in DRONES) / len(DRONES)
        cy = sum(sp.get(v, (0.0, 0.0))[1] for v in DRONES) / len(DRONES)
        ocells = set()
        for v in DRONES:
            ox, oy = sp.get(v, (0.0, 0.0))
            for _, x, y in tracks.get(v, []):
                # int() truncation = scorecard cell convention, 1 m cells, no
                # sensor-disc dilation: measures where drones WENT.
                ocells.add((int(ox + x), int(oy + y)))
        if ocells:
            rs = sorted(math.hypot(i + 0.5 - cx, j + 0.5 - cy)
                        for i, j in ocells)
            out_p90 = rs[min(len(rs) - 1, int(0.9 * (len(rs) - 1)))]
            out_frac8 = sum(1 for r in rs if r > 8.0) / len(rs)
            outward = (out_p90, out_frac8, len(rs))

    # ---- report -----------------------------------------------------------
    print(f"=== score_run {label} ===")
    print(f"U={U:.0f}  P_min={P_min:.1f}m  F_min={F_min:.3f}  T_stall={T_stall:.1f}s")
    print(f"SCORE = {SCORE:.1f}")
    for v in DRONES:
        w = worst_streak.get(v)
        rel = f" (t+{w[1]-t0_fleet:.0f}s..t+{w[2]-t0_fleet:.0f}s goal=({w[0]}))" \
            if w and t0_fleet else ""
        print(f"  {v:14s} path={per[v]['path_m']:6.1f}m moving={per[v]['moving_frac']:.3f} "
              f"metas={per[v]['metas']:4d} stall={t_stall[v]:6.1f}s{rel} "
              f"stereo={stereo.get(v,'-')} plan_ok={plan_ok[v]}")
    if excluded:
        print(f"  EXCLUDED drones (odom divergence): {','.join(excluded)}")
    for g in ("G1", "G2", "G3", "G4"):
        s = gates[g]
        print(f"  {g}: {'PASS' if s else ('n/a' if s is None else 'FAIL')}")
    for k, v in floors.items():
        print(f"  floor {k}: {'PASS' if v else 'FAIL'}")
    if mesh:
        print("  bag mesh msgs: " + " ".join(f"{k}={v}" for k, v in sorted(mesh.items())))
    if bbox:
        x0, x1, y0, y1 = bbox
        print(f"  arena bbox world XY: x[{x0:.1f},{x1:.1f}] y[{y0:.1f},{y1:.1f}] "
              f"extent {x1-x0:.1f}m x {y1-y0:.1f}m "
              f"({(x1-x0)*100:.0f}cm x {(y1-y0)*100:.0f}cm)")
    if outward:
        print(f"  outwardness: p90_radius={outward[0]:.1f}m "
              f"frac_beyond_8m={outward[1]:.2f} visited_cells={outward[2]}")
    verdict_gates = all(gates[g] for g in gates if gates[g] is not None)
    verdict = verdict_gates and all(floors.values())
    print(f"VERDICT: score={SCORE:.1f} gates={'PASS' if verdict_gates else 'FAIL'} "
          f"floors={'PASS' if all(floors.values()) else 'FAIL'} "
          f"-> {'CANDIDATE' if verdict else 'REJECT/RESERVE'}")

    # ---- timeline ---------------------------------------------------------
    if args.timeline:
        tl = {"label": label, "t0_fleet": t0_fleet, "drones": {}}
        for v in DRONES:
            ent = []
            for g, s0, s1 in all_streaks[v]:
                d = s1 - s0
                if d > STREAK_MIN_TIMELINE:
                    ent.append({
                        "goal": g, "t_start": s0, "t_end": s1, "dur_s": round(d, 1),
                        "rel_start_s": round(s0 - t0_fleet, 1) if t0_fleet else None,
                        "rel_end_s": round(s1 - t0_fleet, 1) if t0_fleet else None,
                    })
            tl["drones"][v] = ent
        out = os.path.join(rundir, f"stall_timeline_{label}.json")
        json.dump(tl, open(out, "w"), indent=1)
        n = sum(len(e) for e in tl["drones"].values())
        print(f"timeline: {n} streaks>20s -> {out}")

    # machine-readable one-liner for the attempt table
    print("CSV," + ",".join(str(x) for x in [
        label, f"{SCORE:.1f}", f"{U:.0f}", f"{P_min:.1f}", f"{F_min:.3f}",
        f"{T_stall:.1f}",
        int(bool(gates['G1'])), int(bool(gates['G2'])),
        int(bool(gates['G3'])) if gates['G3'] is not None else "-",
        int(bool(gates['G4'])) if gates['G4'] is not None else "-",
        int(floors['P_min>=30']), int(floors['F_min>=0.15']),
        int(floors['T_stall<=90'])]))


if __name__ == "__main__":
    main()
