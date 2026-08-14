#!/usr/bin/env python3
"""VIO stabilization metrics — consolidated replacement for the ephemeral
align.py / big.py / lead.py / div.py / div2.py / an.py / jump.py / stats.py
referenced in investigation/VIO-STABILIZATION-PLAN.md section 3.

Usage:  vio_metrics.py <label> [logdir]
        vio_metrics.py radio4
        vio_metrics.py radio4_R1 /home/lucas/hercules-sim/e1_frames

Ground truth  : e1_frames/<label>_<veh>/meta_*.json  ->  {"t": epoch, "ned":[x,y,z,yaw]}
Belief        : e1_frames/planner_<label>_<veh>.log  ->  "[INFO] [<epoch>] ... pos=(x,y)"
Frame deltas  : e1_frames/cuvslam_<label>_<veh>.log  ->  "Delta between ... [<ms> ms]"

Belief is the planner's FLU frame (x fwd/N, y left).  AirSim NED is (x N, y E).
The y-sign convention is *fitted* on the first 15 paired samples rather than
assumed (closes GAP-CLOSURE-PLAN.md row 6).
"""
import glob
import json
import math
import os
import re
import sys

import numpy as np

BASE = '/home/lucas/hercules-sim/e1_frames'
VEHS = ('ghost', 'delta', 'buckshee', 'thunderstrike')

POS_RE = re.compile(r'\[INFO\] \[(\d+\.\d+)\].*pos=\(([-\d.]+),([-\d.]+)\)')
DELTA_RE = re.compile(
    r'\[WARN\] \[(\d+\.\d+)\] \[visual_slam\]: '
    r'Delta between current and previous frame \[([\d.]+) ms\]')


def q(a, p):
    if len(a) == 0:
        return float('nan')
    s = sorted(a)
    return s[min(int(p * (len(s) - 1)), len(s) - 1)]


def load_belief(logdir, label, veh):
    p = os.path.join(logdir, f'planner_{label}_{veh}.log')
    out = []
    if not os.path.exists(p):
        return np.zeros((0, 3))
    for line in open(p, errors='ignore'):
        m = POS_RE.search(line)
        if m:
            out.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    return np.array(out, float).reshape(-1, 3)


def load_gt(logdir, label, veh):
    out = []
    for f in sorted(glob.glob(os.path.join(logdir, f'{label}_{veh}', 'meta_*.json'))):
        try:
            d = json.load(open(f))
            n = d['ned']
            out.append((d['t'], n[0], n[1], n[3]))
        except Exception:
            pass
    return np.array(out, float).reshape(-1, 4)


def pair(B, G):
    """Return (t_rel, err, ysign) for every belief sample, GT nearest-neighbour."""
    if len(B) < 2 or len(G) < 2:
        return None
    gt_t = G[:, 0]
    # fit the y-sign convention on the first 15 paired samples
    best, ysign = None, 1.0
    for s in (1.0, -1.0):
        e = []
        for i in range(min(15, len(B))):
            j = int(np.argmin(np.abs(gt_t - B[i, 0])))
            e.append(math.hypot(B[i, 1] - G[j, 1], B[i, 2] - s * G[j, 2]))
        m = float(np.mean(e))
        if best is None or m < best:
            best, ysign = m, s
    t0 = min(B[0, 0], G[0, 0])
    rows = []
    for i in range(len(B)):
        j = int(np.argmin(np.abs(gt_t - B[i, 0])))
        if abs(gt_t[j] - B[i, 0]) > 2.0:
            continue
        err = math.hypot(B[i, 1] - G[j, 1], B[i, 2] - ysign * G[j, 2])
        rows.append((B[i, 0] - t0, err, j))
    return np.array(rows, float), ysign, t0


def yaw_rates(G):
    if len(G) < 3:
        return []
    r = []
    for i in range(1, len(G)):
        dt = G[i, 0] - G[i - 1, 0]
        if dt <= 1e-3:
            continue
        dy = G[i, 3] - G[i - 1, 3]
        dy = (dy + math.pi) % (2 * math.pi) - math.pi
        r.append(abs(math.degrees(dy) / dt))
    return r


def deltas(logdir, label, veh):
    p = os.path.join(logdir, f'cuvslam_{label}_{veh}.log')
    rows = []
    if not os.path.exists(p):
        return rows
    for line in open(p, errors='ignore'):
        m = DELTA_RE.search(line)
        if m:
            rows.append((float(m.group(1)), float(m.group(2))))
    return rows


def plans(logdir, label, veh):
    p = os.path.join(logdir, f'planner_{label}_{veh}.log')
    if not os.path.exists(p):
        return 0, 0
    txt = open(p, errors='ignore').read()
    return txt.count('PLAN OK'), txt.count('Path complete')


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'radio4'
    logdir = sys.argv[2] if len(sys.argv) > 2 else BASE
    print(f'==== {label}  ({logdir}) ====')
    hdr = (f'{"veh":<15}{"t(err>2m)":>10}{"maxErr":>9}{"err@150":>9}'
           f'{"%>3m":>7}{"yawP99":>8}{"yawMax":>8}{"dtP50":>7}{"dtP90":>7}'
           f'{"stut/min":>9}{"PLAN":>6}{"DONE":>6}{"done%":>7}')
    print(hdr)
    print('-' * len(hdr))
    tot_plan = tot_done = 0
    agg = {}
    for v in VEHS:
        B = load_belief(logdir, label, v)
        G = load_gt(logdir, label, v)
        po, pc = plans(logdir, label, v)
        tot_plan += po
        tot_done += pc
        t2 = maxe = e150 = pct3 = float('nan')
        pr = pair(B, G)
        if pr is not None and len(pr[0]):
            R, ysign, t0 = pr
            t, e = R[:, 0], R[:, 1]
            over = t[e > 2.0]
            t2 = float(over[0]) if len(over) else float('inf')
            maxe = float(e.max())
            k = int(np.argmin(np.abs(t - 150.0)))
            e150 = float(e[k])
            pct3 = 100.0 * float((e > 3.0).mean())
        yr = yaw_rates(G)
        d = [x[1] for x in deltas(logdir, label, v)]
        dr = deltas(logdir, label, v)
        spm = float('nan')
        if len(dr) > 1:
            span = dr[-1][0] - dr[0][0]
            after = [x for x in dr if x[0] > dr[0][0] + 30]
            spm = len(after) / max(1e-6, (span - 30) / 60.0)
        agg[v] = dict(t2=t2, maxe=maxe, e150=e150, pct3=pct3,
                      yawp99=q(yr, .99) if yr else float('nan'),
                      yawmax=max(yr) if yr else float('nan'),
                      dtp50=q(d, .5) if d else float('nan'),
                      dtp90=q(d, .9) if d else float('nan'),
                      spm=spm, plan=po, done=pc)
        a = agg[v]
        print(f'{v:<15}{a["t2"]:>10.1f}{a["maxe"]:>9.1f}{a["e150"]:>9.1f}'
              f'{a["pct3"]:>7.1f}{a["yawp99"]:>8.1f}{a["yawmax"]:>8.1f}'
              f'{a["dtp50"]:>7.1f}{a["dtp90"]:>7.1f}{a["spm"]:>9.0f}'
              f'{po:>6}{pc:>6}{(100.0*pc/po if po else 0):>7.1f}')
    print('-' * len(hdr))
    fin = [a for a in agg.values() if math.isfinite(a['maxe'])]
    ay = [x for v in VEHS for x in yaw_rates(load_gt(logdir, label, v))]
    print(f'FLEET  PLAN OK={tot_plan}  Path complete={tot_done}  '
          f'-> {100.0*tot_done/tot_plan if tot_plan else 0:.1f}%')
    if fin:
        print(f'       max|err| worst={max(a["maxe"] for a in fin):.1f} m   '
              f'median={np.median([a["maxe"] for a in fin]):.1f} m   '
              f'err@150 worst={max(a["e150"] for a in fin):.1f} m')
        t2s = [a['t2'] for a in fin]
        print(f'       t(err>2m): ' + ' '.join(
            ('never' if not math.isfinite(x) else f'{x:.1f}s') for x in t2s) +
            f'   n>60s = {sum(1 for x in t2s if x > 60)}/{len(t2s)}')
    if ay:
        print(f'       GT yaw rate fleet: p50={q(ay,.5):.1f} p90={q(ay,.9):.1f} '
              f'p95={q(ay,.95):.1f} p99={q(ay,.99):.1f} max={max(ay):.1f} deg/s  '
              f'(>80: {100.0*sum(1 for x in ay if x>80)/len(ay):.1f}%)')


if __name__ == '__main__':
    main()
