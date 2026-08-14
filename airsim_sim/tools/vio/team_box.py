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
"""
import json
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
    xmin, xmax = min(xs) - half, max(xs) + half
    ymin, ymax = min(ys) - half, max(ys) + half
    for n in names:
        sx, sy = sp[n]
        print('%.3f %.3f %.3f %.3f'
              % (xmin - sx, sy - ymax, xmax - sx, sy - ymin))


if __name__ == '__main__':
    main()
