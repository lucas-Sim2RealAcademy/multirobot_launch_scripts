# v2: walk the whole vertical column so foliage canopies do not masquerade as "ground".
import json, math, os, collections
import unreal

OUT = os.environ["CP_OUT"]
MAP = os.environ["CP_MAP"]
MODE = os.environ.get("CP_MODE", "temple")
PROG = open(OUT + ".log", "a")
def P(*a):
    s = " ".join(str(x) for x in a); print(s); PROG.write(s + "\n"); PROG.flush()

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
les.load_level(MAP)
world = ues.get_editor_world()
actors = eas.get_all_level_actors()
P("LOADED", MAP, len(actors))

VIS = unreal.TraceTypeQuery.TRACE_TYPE_QUERY1
NONE_DBG = unreal.DrawDebugTrace.NONE
SL = unreal.SystemLibrary

SOFT = ("Tree", "Foliage", "Ivy", "Fern", "Bush", "Monstera", "Creeper", "Leaf",
        "Grass", "Plant", "Vine", "Ceiba", "PlaneMesh", "Cube214", "Cube217")


def is_soft(lbl):
    return any(k in lbl for k in SOFT)


def lt(sx, sy, sz, ex, ey, ez):
    h = SL.line_trace_single(world, unreal.Vector(sx, sy, sz), unreal.Vector(ex, ey, ez),
                             VIS, True, [], NONE_DBG, True)
    if not h:
        return None
    t = h.to_tuple()
    try:
        lbl = t[9].get_actor_label()
    except Exception:
        lbl = "?"
    return (t[4].x, t[4].y, t[4].z, lbl)


def column(x, y, ztop, zbot, maxhits=14):
    """all surfaces top->bottom"""
    hits = []
    z = ztop
    for _ in range(maxhits):
        h = lt(x, y, z, x, y, zbot)
        if h is None:
            break
        hits.append((h[2], h[3]))
        z = h[2] - 2.0
        if z <= zbot:
            break
    return hits


NRAY = 16
RAY_LEN = 300.0
DIRS = [(math.cos(2 * math.pi * i / NRAY), math.sin(2 * math.pi * i / NRAY)) for i in range(NRAY)]


def star(x, y, z, skip_soft=False):
    """returns (min_all, nblocked_all, min_hard, nblocked_hard)"""
    da, dh = [], []
    for (dx, dy) in DIRS:
        h = lt(x, y, z, x + dx * RAY_LEN, y + dy * RAY_LEN, z)
        if h is None:
            da.append(RAY_LEN); dh.append(RAY_LEN); continue
        d = math.hypot(h[0] - x, h[1] - y)
        da.append(d)
        if is_soft(h[3]):
            # keep pushing past soft geometry to find the first hard blocker
            cx, cy, dd = h[0], h[1], d
            for _ in range(6):
                sx2 = x + dx * (dd + 3.0)
                sy2 = y + dy * (dd + 3.0)
                h2 = lt(sx2, sy2, z, x + dx * RAY_LEN, y + dy * RAY_LEN, z)
                if h2 is None:
                    dd = RAY_LEN; break
                dd = math.hypot(h2[0] - x, h2[1] - y)
                if not is_soft(h2[3]):
                    break
            dh.append(min(dd, RAY_LEN))
        else:
            dh.append(d)
    return (min(da), sum(1 for v in da if v < RAY_LEN - 1),
            min(dh), sum(1 for v in dh if v < RAY_LEN - 1))


if MODE == "temple":
    XS = list(range(3000, 7601, 100)); YS = list(range(-4400, 1001, 100))
    ZT, ZB = 4000.0, -2000.0
else:
    ps = [a for a in actors if a.get_class().get_name() == "PlayerStart"]
    pl = ps[0].get_actor_location()
    cx, cy = int(round(pl.x / 100) * 100), int(round(pl.y / 100) * 100)
    XS = list(range(cx - 2300, cx + 2301, 100)); YS = list(range(cy - 2300, cy + 2301, 100))
    ZT, ZB = 5000.0, -3000.0

P("GRID", len(XS), len(YS))
cells = {}
soft_top = 0
for x in XS:
    for y in YS:
        fx, fy = float(x), float(y)
        col = column(fx, fy, ZT, ZB)
        if not col:
            cells["%d,%d" % (x, y)] = {"g": None}
            continue
        top_z, top_lbl = col[0]
        if is_soft(top_lbl):
            soft_top += 1
        # hard floor = topmost NON-soft surface in the column
        hard = [(z, l) for (z, l) in col if not is_soft(l)]
        if hard:
            fz, flbl = hard[0]
        else:
            fz, flbl = col[-1]
        rec = {"g": round(fz, 1), "gl": flbl[:36], "tz": round(top_z, 1),
               "tl": top_lbl[:36], "nhit": len(col), "soft_top": int(is_soft(top_lbl))}
        a, na, hh, nh = star(fx, fy, fz + 100.0)
        rec["c"] = round(a, 1); rec["nb"] = na           # counting foliage as solid
        rec["ch"] = round(hh, 1); rec["nbh"] = nh        # foliage treated as passable
        cells["%d,%d" % (x, y)] = rec
    P("col", x)

P("SOFT_TOP_CELLS", soft_top, "of", len(XS) * len(YS))
json.dump({"mode": MODE, "map": MAP, "xs": XS, "ys": YS, "cells": cells,
           "soft_top": soft_top}, open(OUT, "w"))
P("PROBE2_OK")
