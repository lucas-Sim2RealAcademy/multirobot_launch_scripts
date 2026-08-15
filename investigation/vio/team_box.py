#!/usr/bin/env python3
"""Per-vehicle exploration box for the sim fleet, one line per vehicle.

BUG-LIST-VERIFIED.md #8 (fis-bbox): FIS queries nvblox over a hardcoded
+/-10 m AABB pinned to its own launch origin (frontier_info_structure_node.cpp
:18-23) unless a launcher overrides it, and run_fleet_radio.sh overrides
nothing -- it launches `fis.launch.py flight_height:=1.0` and the planner with
no bbox at all, so BOTH consumers silently take the +/-10 m default. That box
is smaller than the mission area the fleet is scored on (the union of every
vehicle's box around its own spawn) and is pinned to a different origin per
vehicle, so each drone hits a wall ~10 m out and mills at the edge.

This emits the TEAM box -- the union of every vehicle's +/-half_m box around
its spawn, i.e. exactly the arena fidelity_scorecard.sh scores -- expressed in
each vehicle's OWN odom frame, so FIS and the planner can be handed the same
numbers.

Frames: AirSim spawn coords are NED (x north, y east). The planner and FIS
work the cuVSLAM odom frame, which is FLU (x forward/north, y LEFT), so
y_flu = -y_ned. Verified empirically: vio_metrics.pair() fits ysign = -1 for
all four vehicles on every run. All spawns share yaw 0 (settings-fleet-*.json),
so no rotation is needed.

usage: team_box.py <settings.json> <half_m> <veh> [<veh> ...]
       -> one "min_x min_y max_x max_y" line per vehicle, in argument order

CORRIDOR OVERRIDE (street maps): HERC_CORRIDOR="<ahead_m> <behind_m> <lat_half_m>"
plus optional HERC_STREET_AXIS=-x|+x|-y|+y (default -x) replaces the square
team box with a street-shaped TEAM corridor: from behind_m behind the rearmost
spawn to ahead_m beyond the foremost spawn along the street axis, +/-lat_half_m
laterally around the spawn line.  Same NED->FLU emission as the square box, so
run_fleet_radio.sh needs no edits -- export the env var and run.
  JapanFest_Street: down-street is NED -X (verified: every mrq1/mrq2 drone
  pinned the -X box edge; voxel probe shows 115 m of clear street at 0-2 m AGL
  toward -X, while +2 m clips the overhead banners every ~10 m).
"""
import json
import os
import re
import sys


def main():
    path, half = sys.argv[1], float(sys.argv[2])
    names = sys.argv[3:]
    txt = re.sub(r'^\s*//.*$', '', open(path).read(), flags=re.M)
    veh = json.loads(txt)['Vehicles']
    sp = {n: (float(veh[n].get('X', 0.0)), float(veh[n].get('Y', 0.0)))
          for n in names}
    xs = [x for x, _ in sp.values()]
    ys = [y for _, y in sp.values()]
    corridor = os.environ.get('HERC_CORRIDOR', '').strip()
    if corridor:
        ahead, behind, lat = (float(v) for v in corridor.split())
        axis = os.environ.get('HERC_STREET_AXIS', '-x')
        if axis in ('-x', '+x'):
            if axis == '-x':
                xmin, xmax = min(xs) - ahead, max(xs) + behind
            else:
                xmin, xmax = min(xs) - behind, max(xs) + ahead
            ymin, ymax = min(ys) - lat, max(ys) + lat
        elif axis in ('-y', '+y'):
            if axis == '-y':
                ymin, ymax = min(ys) - ahead, max(ys) + behind
            else:
                ymin, ymax = min(ys) - behind, max(ys) + ahead
            xmin, xmax = min(xs) - lat, max(xs) + lat
        else:
            raise SystemExit(f'bad HERC_STREET_AXIS {axis!r}')
        for n in names:
            sx, sy = sp[n]
            print('%.3f %.3f %.3f %.3f'
                  % (xmin - sx, sy - ymax, xmax - sx, sy - ymin))
        return
    xmin, xmax = min(xs) - half, max(xs) + half
    ymin, ymax = min(ys) - half, max(ys) + half
    for n in names:
        sx, sy = sp[n]
        print('%.3f %.3f %.3f %.3f'
              % (xmin - sx, sy - ymax, xmax - sx, sy - ymin))


if __name__ == '__main__':
    main()
