#!/usr/bin/env python3
"""One-line-per-step coverage report for the FIS/coordination A/B sequence.

usage: coverage_report.py <label> [logdir]
       logdir defaults to e1_frames/runs/<label> if it exists, else e1_frames.

Emits every number the step table needs:
  fleet_unique_m2 fleet_effort_m2 fleet_overlap_pct fleet_arena_cells
  per-drone worst/median belief-vs-truth divergence
  Path complete de-duplicated by goal (and raw), PLAN OK
  cuVSLAM stutter/min, host load1 (mean/max over the run)
  claim release reason histogram
"""
import glob
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import vio_metrics as vm

E1 = '/home/lucas/hercules-sim/e1_frames'
VEHS = vm.VEHS
DONE_RE = re.compile(r'Path complete at \(([-\d.]+),([-\d.]+)\)')
PLAN_RE = re.compile(r'PLAN OK: .*goal=\(([-\d.]+),([-\d.]+)\)')
REL_RE = re.compile(r'claim released: (\w+)')


def scorecard(logdir, label):
    p = os.path.join(logdir, 'scorecard_%s.txt' % label)
    out = {}
    if not os.path.exists(p):
        return out
    # fixed-width rows: "%-38s %-16s %-16s %-7s %s" -> metric, ref, sim, -, status
    for line in open(p, errors='ignore'):
        metric = line[:38].strip()
        if metric.startswith(('fleet_', 'overlap_floor', 'coverage_m2',
                              'odom_divergence')):
            tok = line[38:].split()
            out[metric] = tok[-3] if len(tok) >= 3 and tok[-2] == '-' else '?'
    return out


def load1(logdir, label):
    p = os.path.join(logdir, 'metrics_%s.csv' % label)
    if not os.path.exists(p):
        return (float('nan'), float('nan'))
    v = []
    for line in open(p, errors='ignore').readlines()[1:]:
        f = line.strip().split(',')
        if len(f) >= 4:
            try:
                v.append(float(f[3]))
            except ValueError:
                pass
    return (float(np.mean(v)), float(np.max(v))) if v else (float('nan'),) * 2


def main():
    label = sys.argv[1]
    logdir = sys.argv[2] if len(sys.argv) > 2 else None
    if logdir is None:
        cand = os.path.join(E1, 'runs', label)
        logdir = cand if os.path.isdir(cand) else E1
    print('==== %s   (%s) ====' % (label, logdir))

    sc = scorecard(logdir, label)
    for k in ('fleet_arena_cells', 'fleet_effort_m2', 'fleet_unique_m2',
              'overlap_floor_pct', 'fleet_overlap_pct',
              'fleet_overlap_concurrent_pct'):
        print('  %-30s %s' % (k, sc.get(k, '-')))

    l1m, l1x = load1(logdir, label)
    print('  %-30s mean %.2f  max %.2f' % ('host_load1', l1m, l1x))

    hdr = ('%-15s%9s%9s%9s%7s%7s%7s%9s%9s' %
           ('veh', 'divWorst', 'divMed', 'div@150', 'PLAN', 'DONE', 'uDONE',
            'done%', 'stut/min'))
    print(hdr)
    print('-' * len(hdr))
    tot = dict(plan=0, done=0, udone=0)
    worsts, meds, stuts = [], [], []
    for v in VEHS:
        B = vm.load_belief(logdir, label, v)
        G = vm.load_gt(logdir, label, v)
        dw = dm = d150 = float('nan')
        pr = vm.pair(B, G)
        if pr is not None and len(pr[0]):
            R, _ys, _t0 = pr
            t, e = R[:, 0], R[:, 1]
            dw, dm = float(e.max()), float(np.median(e))
            k = int(np.argmin(np.abs(t - 150.0)))
            d150 = float(e[k])
            worsts.append(dw)
            meds.append(dm)
        p = os.path.join(logdir, 'planner_%s_%s.log' % (label, v))
        txt = open(p, errors='ignore').read() if os.path.exists(p) else ''
        plan = len(PLAN_RE.findall(txt))
        dones = DONE_RE.findall(txt)
        udone = len({(round(float(a)), round(float(b))) for a, b in dones})
        dr = vm.deltas(logdir, label, v)
        spm = float('nan')
        if len(dr) > 1:
            span = dr[-1][0] - dr[0][0]
            after = [x for x in dr if x[0] > dr[0][0] + 30]
            spm = len(after) / max(1e-6, (span - 30) / 60.0)
            stuts.append(spm)
        tot['plan'] += plan
        tot['done'] += len(dones)
        tot['udone'] += udone
        print('%-15s%9.1f%9.1f%9.1f%7d%7d%7d%9.1f%9.0f' %
              (v, dw, dm, d150, plan, len(dones), udone,
               100.0 * udone / plan if plan else 0.0, spm))
    print('-' * len(hdr))
    print('FLEET  PLAN=%d  DONE=%d  uDONE=%d  uDONE/PLAN=%.1f%%  '
          'divWorst=%.1f m  divMed=%.1f m  stut/min=%.0f' %
          (tot['plan'], tot['done'], tot['udone'],
           100.0 * tot['udone'] / tot['plan'] if tot['plan'] else 0.0,
           max(worsts) if worsts else float('nan'),
           float(np.median(meds)) if meds else float('nan'),
           float(np.mean(stuts)) if stuts else float('nan')))

    rel = {}
    for f in glob.glob(os.path.join(logdir, 'coord_%s_*.log' % label)):
        for line in open(f, errors='ignore'):
            m = REL_RE.search(line)
            if m:
                rel[m.group(1)] = rel.get(m.group(1), 0) + 1
    nset = sum(len(re.findall('claim set:', open(f, errors='ignore').read()))
               for f in glob.glob(os.path.join(logdir, 'coord_%s_*.log' % label)))
    n = sum(rel.values())
    print('  claims set=%d  released=%d  %s' %
          (nset, n, '  '.join('%s=%d(%.0f%%)' % (k, c, 100.0 * c / n)
                              for k, c in sorted(rel.items(),
                                                 key=lambda x: -x[1]))))


if __name__ == '__main__':
    main()
