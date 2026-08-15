#!/usr/bin/env python3
"""Anatomy of fleet_overlap_pct: where does the duplicated coverage come from?

Reproduces fidelity_scorecard.sh's coverage model EXACTLY (4 m disc, int()
truncation, spawn offsets, arena clip) and then decomposes the duplicate cells
into buckets that map onto different fixes:

  A. LAUNCH-GEOMETRY  duplicates present in the first T_LAUNCH s of the run,
     i.e. cells shared purely because the 4 spawns are 3 m apart in a line and
     the scoring disc is 4 m.  No coordination mechanism can remove these.
  B. CONCURRENT       two drones occupy the cell within DT of each other
     (excluding A).  This is the assignment-collision bucket.
  C. SEQUENTIAL       one drone re-sweeps ground a peer covered > DT ago
     (excluding A).  This is the stale-knowledge / no-map-sharing bucket.

Also reports:
  * pairwise |A n B| and the min inter-drone time gap per shared cell
  * per-drone SELF re-sweep (path samples landing on cells the same drone
    already covered) -- note this does NOT enter fleet_overlap_pct at all,
    because effort is a sum over per-drone SETS.  Reported to size the
    wasted-time bucket that shows up as low fleet_unique_m2 instead.
  * a geometric floor: overlap the fleet would score if each drone flew its
    OWN measured path length but the paths were perfectly partitioned.

usage: overlap_anatomy.py <logdir> <label> [arena_half]
"""
import glob
import json
import math
import os
import re
import sys
from collections import defaultdict

ARENA_HALF = 10.0
DT = 10.0
T_LAUNCH = 20.0
DISC = [(dx, dy) for dx in range(-4, 5) for dy in range(-4, 5)
        if dx * dx + dy * dy <= 16]


def load_spawn(base, names):
    for path in sorted(glob.glob('%s/settings-fleet-*.json' % base)):
        txt = re.sub(r'^\s*//.*$', '', open(path).read(), flags=re.M)
        veh = json.loads(txt).get('Vehicles', {})
        if set(names) <= set(veh):
            return {k: (float(v.get('X', 0.0)), float(v.get('Y', 0.0)))
                    for k, v in veh.items()}
    return {n: (0.0, 0.0) for n in names}


def main():
    logdir = sys.argv[1]
    label = sys.argv[2]
    half = float(sys.argv[3]) if len(sys.argv) > 3 else ARENA_HALF
    base = '/home/lucas/hercules-sim'

    tracks = {}
    for d in sorted(glob.glob('%s/%s_*/' % (logdir, label))):
        v = d.rstrip('/').split('_')[-1]
        pts = []
        for m in sorted(glob.glob(d + 'meta_*.json')):
            try:
                j = json.load(open(m))
            except Exception:
                continue
            ned = j.get('ned') or []
            if len(ned) < 2:
                continue
            pts.append((float(j.get('t', 0.0)), float(ned[0]), float(ned[1])))
        if pts:
            pts.sort()
            tracks[v] = pts
    if not tracks:
        print('no tracks for %s in %s' % (label, logdir))
        return
    drones = sorted(tracks)
    spawn = load_spawn(base, drones)

    arena = set()
    for v in drones:
        ox, oy = spawn[v]
        for cx in range(int(ox - half), int(ox + half) + 1):
            for cy in range(int(oy - half), int(oy + half) + 1):
                arena.add((cx, cy))

    # ---- per-drone coverage, first-touch time, and self re-sweep ----
    cells = {}          # v -> set of arena cells
    first_t = {}        # v -> {cell: first time touched}
    ivals = {}          # v -> {cell: [[t_in, t_out], ...]}
    self_resweep = {}   # v -> # samples whose whole disc was already covered
    pathlen = {}
    for v in drones:
        ox, oy = spawn[v]
        cs, ft, iv = set(), {}, {}
        reswept = 0
        plen = 0.0
        prev = None
        for t, x, y in tracks[v]:
            ax, ay = x + ox, y + oy
            if prev is not None:
                plen += math.hypot(ax - prev[0], ay - prev[1])
            prev = (ax, ay)
            cx, cy = int(ax), int(ay)
            newc = 0
            for dx, dy in DISC:
                c = (cx + dx, cy + dy)
                if c not in arena:
                    continue
                if c not in cs:
                    newc += 1
                    cs.add(c)
                    ft[c] = t
                tl = iv.get(c)
                if tl is None:
                    iv[c] = [[t, t]]
                elif t - tl[-1][1] > DT:
                    tl.append([t, t])
                else:
                    tl[-1][1] = t
            if newc == 0:
                reswept += 1
        cells[v], first_t[v], ivals[v] = cs, ft, iv
        self_resweep[v] = (reswept, len(tracks[v]))
        pathlen[v] = plen

    effort = sum(len(cells[v]) for v in drones)
    unique = set().union(*[cells[v] for v in drones])
    overlap = 100.0 * (1.0 - len(unique) / effort)
    arith_floor = 100.0 * max(0.0, 1.0 - len(arena) / effort)

    print('=' * 78)
    print('OVERLAP ANATOMY  label=%s  logdir=%s' % (label, logdir))
    print('=' * 78)
    print('arena %d cells   effort %d   unique %d   overlap %.1f%%   '
          'arith floor %.1f%%' % (len(arena), effort, len(unique), overlap,
                                  arith_floor))
    print('duplicate cell-assignments (effort - unique) = %d' % (effort - len(unique)))
    print()

    # ---- per drone ----
    print('--- per drone ---')
    print('%-15s %6s %8s %9s %9s %9s' %
          ('drone', 'cells', 'path_m', 'samples', 'resweep', 'dur_s'))
    for v in drones:
        rs, n = self_resweep[v]
        dur = tracks[v][-1][0] - tracks[v][0][0]
        print('%-15s %6d %8.1f %9d %8.0f%% %9.1f' %
              (v, len(cells[v]), pathlen[v], n, 100.0 * rs / max(n, 1), dur))
    print()

    # ---- pairwise ----
    print('--- pairwise shared cells (|A n B|, and share of all duplicates) ---')
    dupcells = [c for c in unique
                if sum(1 for v in drones if c in cells[v]) >= 2]
    total_dup = effort - len(unique)
    pairs = []
    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            a, b = drones[i], drones[j]
            inter = cells[a] & cells[b]
            if not inter:
                continue
            gaps = []
            for c in inter:
                g = min(max(x[0], y[0]) - min(x[1], y[1])
                        for x in ivals[a][c] for y in ivals[b][c])
                gaps.append(max(0.0, g))
            gaps.sort()
            conc = sum(1 for g in gaps if g <= DT)
            pairs.append((len(inter), a, b, conc, gaps[len(gaps) // 2]))
    pairs.sort(reverse=True)
    print('%-28s %7s %7s %10s %12s' %
          ('pair', 'shared', '%dup', 'concurrent', 'median_gap_s'))
    for n, a, b, conc, med in pairs:
        print('%-28s %7d %6.0f%% %9d%s %12.1f' %
              ('%s + %s' % (a, b), n, 100.0 * n / max(total_dup, 1), conc,
               ' (%.0f%%)' % (100.0 * conc / n), med))
    print()

    # ---- bucket decomposition over DUPLICATE ASSIGNMENTS ----
    # For each cell covered by k>=2 drones, it contributes (k-1) duplicates.
    # Attribute each duplicate to the bucket of the pair that produced it,
    # walking drones in first-touch order (the 2nd..kth toucher is the dupe).
    bucket = defaultdict(int)
    launch_dup = 0
    for c in dupcells:
        holders = sorted([v for v in drones if c in cells[v]],
                         key=lambda v: first_t[v][c])
        for k in range(1, len(holders)):
            v = holders[k]
            tv = first_t[v][c]
            # nearest earlier toucher
            prevs = holders[:k]
            gap = min(tv - first_t[p][c] for p in prevs)
            t_earliest = min(first_t[p][c] for p in prevs)
            if tv <= T_LAUNCH and t_earliest <= T_LAUNCH:
                bucket['A_launch'] += 1
                launch_dup += 1
            elif gap <= DT:
                bucket['B_concurrent'] += 1
            else:
                bucket['C_sequential'] += 1
    print('--- duplicate-assignment buckets (total %d) ---' % total_dup)
    for k in ('A_launch', 'B_concurrent', 'C_sequential'):
        n = bucket[k]
        print('%-16s %6d  %5.1f%% of dupes   %5.1f pts of the %.1f%% headline'
              % (k, n, 100.0 * n / max(total_dup, 1),
                 100.0 * n / effort, overlap))
    print()

    # ---- launch-geometry floor: what 4 parked drones already score ----
    park = {}
    for v in drones:
        ox, oy = spawn[v]
        cx, cy = int(ox), int(oy)
        park[v] = set((cx + dx, cy + dy) for dx, dy in DISC
                      if (cx + dx, cy + dy) in arena)
    p_eff = sum(len(s) for s in park.values())
    p_uni = set().union(*park.values())
    print('--- launch geometry (4 drones parked at spawns, 4 m disc) ---')
    print('parked effort %d  unique %d  overlap %.1f%%  '
          '(duplicates %d, unavoidable at t=0)'
          % (p_eff, len(p_uni), 100.0 * (1 - len(p_uni) / p_eff),
             p_eff - len(p_uni)))
    print('  -> at THIS run\'s effort of %d, those %d duplicates alone are '
          '%.1f pts of overlap' % (effort, p_eff - len(p_uni),
                                   100.0 * (p_eff - len(p_uni)) / effort))
    print()

    # ---- geometric floor: same per-drone cell counts, perfect partition ----
    print('--- achievable floor for THIS effort ---')
    print('arithmetic (scorecard): 1 - arena/effort = %.1f%%' % arith_floor)
    excess = effort - len(arena)
    print('  interpretation: %d cells of effort into a %d-cell arena forces '
          '>= %d duplicates' % (effort, len(arena), max(0, excess)))
    print('  BUT that assumes the fleet can tile the arena perfectly from 4 '
          'spawn points 3 m apart.')
    lg = p_eff - len(p_uni)
    print('  launch-geometry duplicates that no assignment policy removes: %d'
          ' (%.1f pts)' % (lg, 100.0 * lg / effort))
    print('  => practical floor ~ %.1f%%'
          % (max(arith_floor, 100.0 * lg / effort)))
    print()

    # ---- where in space are the duplicates? ----
    print('--- spatial distribution of duplicates (distance from spawn line) ---')
    binsz = 3.0
    hist = defaultdict(int)
    tot = defaultdict(int)
    for c in unique:
        k = sum(1 for v in drones if c in cells[v])
        # distance from the launch line x=-7
        d = abs(c[0] - (-7.0))
        b = int(d // binsz) * binsz
        tot[b] += k
        hist[b] += (k - 1)
    print('%10s %10s %10s %8s' % ('dist_x_m', 'effort', 'dupes', 'dup%'))
    for b in sorted(tot):
        print('%7.0f-%2.0f %10d %10d %7.0f%%' %
              (b, b + binsz, tot[b], hist[b],
               100.0 * hist[b] / max(tot[b], 1)))
    print()

    # ---- coverage fraction of arena ----
    print('arena coverage: %d/%d = %.1f%% of the arena was ever swept'
          % (len(unique), len(arena), 100.0 * len(unique) / len(arena)))


if __name__ == '__main__':
    main()
