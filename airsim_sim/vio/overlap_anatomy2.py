#!/usr/bin/env python3
"""Second-pass overlap diagnosis: time-resolved, arena-frame.

Fixes the time base (meta 't' is absolute epoch, and cuvslam_sim_bridge staggers
starts by 4 s per drone), then answers:
  * where is each drone over time, in ARENA frame?
  * how far apart are the drones from each other over time?
  * are duplicates concurrent (assignment collision) or sequential (stale map)?
  * do the drones all migrate in the same direction / pile on the same wall?

usage: overlap_anatomy2.py <logdir> <label> [arena_half]
"""
import glob
import json
import math
import re
import sys
from collections import defaultdict

DT = 10.0
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
    logdir, label = sys.argv[1], sys.argv[2]
    half = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
    base = '/home/lucas/hercules-sim'

    raw = {}
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
            raw[v] = pts
    drones = sorted(raw)
    spawn = load_spawn(base, drones)
    # common wall-clock origin so cross-drone time comparisons are honest
    t0 = min(p[0][0] for p in raw.values())

    trk = {}   # v -> [(t_rel, arena_x, arena_y)]
    for v in drones:
        ox, oy = spawn[v]
        trk[v] = [(t - t0, x + ox, y + oy) for t, x, y in raw[v]]

    arena = set()
    for v in drones:
        ox, oy = spawn[v]
        for cx in range(int(ox - half), int(ox + half) + 1):
            for cy in range(int(oy - half), int(oy + half) + 1):
                arena.add((cx, cy))

    print('=' * 78)
    print('TIME-RESOLVED OVERLAP DIAGNOSIS  label=%s' % label)
    print('=' * 78)

    # ---- trajectory sample every 15 s in arena frame ----
    print('--- arena-frame position every 15 s (x_north, y_east) ---')
    hdr = '%6s' % 't_s'
    for v in drones:
        hdr += ' %18s' % v
    print(hdr)
    for tq in range(0, 155, 15):
        line = '%6d' % tq
        for v in drones:
            best = min(trk[v], key=lambda p: abs(p[0] - tq))
            if abs(best[0] - tq) > 8:
                line += ' %18s' % '-'
            else:
                line += ' %18s' % ('(%.1f,%.1f)' % (best[1], best[2]))
        print(line)
    print()

    # ---- pairwise separation over time ----
    print('--- pairwise separation (m) every 15 s ---')
    prs = [(drones[i], drones[j])
           for i in range(len(drones)) for j in range(i + 1, len(drones))]
    hdr = '%6s' % 't_s'
    for a, b in prs:
        hdr += ' %13s' % ('%s/%s' % (a[:5], b[:5]))
    print(hdr)
    sep_acc = defaultdict(list)
    for tq in range(0, 155, 15):
        line = '%6d' % tq
        for a, b in prs:
            pa = min(trk[a], key=lambda p: abs(p[0] - tq))
            pb = min(trk[b], key=lambda p: abs(p[0] - tq))
            if abs(pa[0] - tq) > 8 or abs(pb[0] - tq) > 8:
                line += ' %13s' % '-'
                continue
            d = math.hypot(pa[1] - pb[1], pa[2] - pb[2])
            sep_acc[(a, b)].append(d)
            line += ' %13.1f' % d
        print(line)
    print('%6s' % 'mean', end='')
    for a, b in prs:
        s = sep_acc[(a, b)]
        print(' %13.1f' % (sum(s) / len(s) if s else 0), end='')
    print('\n')

    # ---- coverage + buckets with the corrected clock ----
    cells, ivals, first_t = {}, {}, {}
    for v in drones:
        cs, iv, ft = set(), {}, {}
        for t, x, y in trk[v]:
            cx, cy = int(x), int(y)
            for dx, dy in DISC:
                c = (cx + dx, cy + dy)
                if c not in arena:
                    continue
                if c not in cs:
                    cs.add(c)
                    ft[c] = t
                tl = iv.get(c)
                if tl is None:
                    iv[c] = [[t, t]]
                elif t - tl[-1][1] > DT:
                    tl.append([t, t])
                else:
                    tl[-1][1] = t
        cells[v], ivals[v], first_t[v] = cs, iv, ft

    effort = sum(len(cells[v]) for v in drones)
    unique = set().union(*[cells[v] for v in drones])
    overlap = 100.0 * (1 - len(unique) / effort)
    total_dup = effort - len(unique)

    # bucket by MINIMUM inter-drone occupancy gap (the honest test)
    bins = [(0.0, 'simultaneous  (gap 0 s)'),
            (5.0, 'near-sim      (0-5 s)'),
            (10.0, 'concurrent    (5-10 s)'),
            (30.0, 'lagged        (10-30 s)'),
            (60.0, 'stale         (30-60 s)'),
            (1e9, 'sequential    (>60 s)')]
    hist = defaultdict(int)
    for c in unique:
        holders = [v for v in drones if c in cells[v]]
        if len(holders) < 2:
            continue
        # each of the (k-1) duplicates attributed by its own min gap
        order = sorted(holders, key=lambda v: first_t[v][c])
        for k in range(1, len(order)):
            v = order[k]
            gap = 1e9
            for p in order[:k]:
                for A in ivals[v][c]:
                    for B in ivals[p][c]:
                        g = max(A[0], B[0]) - min(A[1], B[1])
                        gap = min(gap, max(0.0, g))
            for lim, name in bins:
                if gap <= lim:
                    hist[name] += 1
                    break

    print('--- duplicates by inter-drone time gap (total %d, headline %.1f%%) ---'
          % (total_dup, overlap))
    cum = 0
    for lim, name in bins:
        n = hist[name]
        cum += n
        print('%-26s %6d  %5.1f%%  cum %5.1f%%   = %4.1f pts of headline'
              % (name, n, 100.0 * n / max(total_dup, 1),
                 100.0 * cum / max(total_dup, 1), 100.0 * n / effort))
    print()

    # ---- how many drones share each duplicated cell ----
    mult = defaultdict(int)
    for c in unique:
        k = sum(1 for v in drones if c in cells[v])
        mult[k] += 1
    print('--- cell multiplicity ---')
    for k in sorted(mult):
        print('  covered by %d drone(s): %4d cells  (%d duplicate assignments)'
              % (k, mult[k], mult[k] * (k - 1)))
    print()

    # ---- direction of travel: did everyone run the same way? ----
    print('--- net displacement from spawn (arena frame) ---')
    print('%-15s %10s %10s %10s %10s' %
          ('drone', 'dx_north', 'dy_east', 'net_m', 'bearing'))
    for v in drones:
        ox, oy = spawn[v]
        ex, ey = trk[v][-1][1], trk[v][-1][2]
        dx, dy = ex - ox, ey - oy
        print('%-15s %10.1f %10.1f %10.1f %9.0f°'
              % (v, dx, dy, math.hypot(dx, dy),
                 math.degrees(math.atan2(dy, dx)) % 360))
    print()

    # ---- time spent within r of ANOTHER drone ----
    print('--- fraction of flight time each drone spent within R of a peer ---')
    for R in (4.0, 8.0):
        print('  R = %.0f m:' % R, end='')
        for v in drones:
            n = 0
            for t, x, y in trk[v]:
                close = False
                for w in drones:
                    if w == v:
                        continue
                    p = min(trk[w], key=lambda q: abs(q[0] - t))
                    if abs(p[0] - t) < 2.0 and math.hypot(p[1] - x, p[2] - y) < R:
                        close = True
                        break
                if close:
                    n += 1
            print('  %s %.0f%%' % (v[:5], 100.0 * n / len(trk[v])), end='')
        print()


if __name__ == '__main__':
    main()
