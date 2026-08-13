#!/usr/bin/env python3
"""One row per archived run: the numbers this investigation is graded on.

excess = fleet_overlap_pct - overlap_floor_pct.  When effort > arena this is
algebraically identical to (arena_cells - unique_cells)/effort, i.e. UNCOVERED
ARENA normalised by effort -- so the scorecard's pass bar (excess <= 5) is a
coverage target in disguise, not a duplication target.

usage: compare_runs.py [label ...]
"""
import glob
import os
import re
import subprocess
import sys

RUNS = '/home/lucas/hercules-sim/e1_frames/runs'
KEYS = ('fleet_arena_cells', 'fleet_effort_m2', 'fleet_unique_m2',
        'overlap_floor_pct', 'fleet_overlap_pct',
        'fleet_overlap_concurrent_pct')


def scrape(label):
    sc = os.path.join(RUNS, label, 'scorecard_%s.txt' % label)
    if not os.path.exists(sc):
        return None
    txt = open(sc).read()
    out = {}
    for k in KEYS:
        m = re.search(r'^%s\s+\S+(?:\s+\S+)*?\s+([\d.]+)\s' % re.escape(k),
                      txt, re.M)
        if m:
            out[k] = float(m.group(1))
    div = [float(m) for m in
           re.findall(r'^odom_divergence_m\[\w+\]\s+\S+\s+([\d.]+)', txt, re.M)]
    out['maxdiv_odom'] = max(div) if div else float('nan')
    # load1
    lp = os.path.join(RUNS, label, 'load_%s.txt' % label)
    out['load'] = ''
    if os.path.exists(lp):
        t = open(lp).read()
        m = re.search(r'median=([\d.]+)', t)
        p = re.search(r'load1_pre=([\d.]+)', t)
        out['load'] = '%s/%s' % (p.group(1) if p else '?',
                                 m.group(1) if m else '?')
    # vio metrics: goal completion + max divergence vs ground truth
    try:
        r = subprocess.run(
            ['python3', '/home/lucas/hercules-sim/investigation/vio/vio_metrics.py',
             label, os.path.join(RUNS, label)],
            capture_output=True, text=True, timeout=180)
        m = re.search(r'PLAN OK=(\d+)\s+Path complete=(\d+)\s+->\s+([\d.]+)%',
                      r.stdout)
        if m:
            out['plan'] = int(m.group(1))
            out['done'] = int(m.group(2))
            out['done_pct'] = float(m.group(3))
        m = re.search(r'max\|err\| worst=([\d.]+)', r.stdout)
        if m:
            out['maxdiv'] = float(m.group(1))
        m = re.search(r'n>60s = (\d+)/(\d+)', r.stdout)
        if m:
            out['track60'] = '%s/%s' % (m.group(1), m.group(2))
    except Exception:
        pass
    return out


def main():
    labels = sys.argv[1:] or sorted(
        os.path.basename(d) for d in glob.glob(RUNS + '/*') if os.path.isdir(d))
    print('%-16s %-11s %6s %6s %6s %6s %6s %6s  %5s %6s %7s %6s'
          % ('label', 'load1 pre/med', 'arena', 'effort', 'unique', 'floor',
             'ovlp%', 'EXCESS', 'div', 'trk60', 'plan/ok', 'done%'))
    print('-' * 108)
    for lb in labels:
        s = scrape(lb)
        if not s or 'fleet_effort_m2' not in s:
            continue
        exc = s.get('fleet_overlap_pct', 0) - s.get('overlap_floor_pct', 0)
        print('%-16s %-11s %6d %6d %6d %6.1f %6.1f %6.1f  %5s %6s %7s %6s'
              % (lb, s['load'], s.get('fleet_arena_cells', 0),
                 s['fleet_effort_m2'], s.get('fleet_unique_m2', 0),
                 s.get('overlap_floor_pct', 0), s.get('fleet_overlap_pct', 0),
                 exc,
                 ('%.1f' % s['maxdiv']) if 'maxdiv' in s else '-',
                 s.get('track60', '-'),
                 '%d/%d' % (s.get('plan', 0), s.get('done', 0))
                 if 'plan' in s else '-',
                 ('%.0f' % s['done_pct']) if 'done_pct' in s else '-'))


if __name__ == '__main__':
    main()
