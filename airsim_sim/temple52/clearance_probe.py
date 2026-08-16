# Headless UE5.2 clearance probe.
# Measures ACTUAL free space at flight altitude, independent of the robot stack.
# Driven by env vars:
#   CP_OUT   = output json path
#   CP_MAP   = level package path
#   CP_MODE  = "temple" | "blocks"
import json, math, os, collections
import unreal

OUT = os.environ["CP_OUT"]
MAP = os.environ["CP_MAP"]
MODE = os.environ.get("CP_MODE", "temple")
PROG = open(OUT + ".log", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    PROG.write(s + "\n")
    PROG.flush()


les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

ok = les.load_level(MAP)
world = ues.get_editor_world()
P("LOADED", ok, world.get_name() if world else None)

# ---- force every streaming sublevel loaded+visible ------------------------
sub_report = []
try:
    streams = world.get_editor_property("streaming_levels")
    for sl in streams:
        nm = str(sl.get_editor_property("world_asset"))
        try:
            sl.set_editor_property("should_be_loaded", True)
            sl.set_editor_property("should_be_visible", True)
        except Exception as e:
            sub_report.append("SETFAIL %s %s" % (nm, e))
        sub_report.append("STREAM %s loaded=%s visible=%s" % (
            nm, sl.get_editor_property("should_be_loaded"),
            sl.get_editor_property("should_be_visible")))
except Exception as e:
    sub_report.append("no streaming_levels: %s" % e)
for s in sub_report:
    P(s)

try:
    levels = unreal.EditorLevelUtils.get_levels(world)
    P("LEVELS_IN_WORLD", len(levels))
except Exception as e:
    P("get_levels failed", e)

actors = eas.get_all_level_actors()
cc = collections.Counter(a.get_class().get_name() for a in actors)
P("ACTOR_TOTAL", len(actors))
P("ACTOR_TOP", json.dumps(dict(cc.most_common(15))))

# count static mesh components that actually have collision enabled
n_smc = 0
n_smc_nocoll = 0
for a in actors[:200000]:
    for c in a.get_components_by_class(unreal.StaticMeshComponent):
        n_smc += 1
        try:
            ce = str(c.get_editor_property("collision_enabled"))
            if "NO_COLLISION" in ce.upper():
                n_smc_nocoll += 1
        except Exception:
            pass
P("SMC_TOTAL", n_smc, "SMC_NOCOLLISION", n_smc_nocoll)

VIS = unreal.TraceTypeQuery.TRACE_TYPE_QUERY1   # Visibility
CAM = unreal.TraceTypeQuery.TRACE_TYPE_QUERY2   # Camera
NONE_DBG = unreal.DrawDebugTrace.NONE
SL = unreal.SystemLibrary


def lt(sx, sy, sz, ex, ey, ez, chan=VIS, complexf=True):
    """single line trace -> (hitloc_tuple, actor_label) or None"""
    h = SL.line_trace_single(world, unreal.Vector(sx, sy, sz), unreal.Vector(ex, ey, ez),
                             chan, complexf, [], NONE_DBG, True)
    if not h:
        return None
    t = h.to_tuple()
    loc = t[4]
    try:
        lbl = t[9].get_actor_label()
    except Exception:
        lbl = "?"
    return ((loc.x, loc.y, loc.z), lbl)


def lt_multi(sx, sy, sz, ex, ey, ez, chan=VIS):
    try:
        r = SL.line_trace_multi(world, unreal.Vector(sx, sy, sz), unreal.Vector(ex, ey, ez),
                                chan, True, [], NONE_DBG, True)
    except Exception:
        return None
    # python binding returns (bool, [HitResult]) or [HitResult]
    hits = None
    if isinstance(r, tuple):
        for item in r:
            if isinstance(item, (list, unreal.Array)):
                hits = item
    elif isinstance(r, (list, unreal.Array)):
        hits = r
    if hits is None:
        return None
    out = []
    for h in hits:
        t = h.to_tuple()
        out.append(t[4].z)
    return out


# ---------------- grid definition ----------------------------------------
if MODE == "temple":
    XS = list(range(3000, 7601, 100))
    YS = list(range(-4400, 1001, 100))
    Z_TOP, Z_BOT = 4000.0, -2000.0
    FIXED_ALT = 960.0          # PlayerStart z 860 + 1 m
    spawns = {
        "ghost": (6150, -2150),
        "delta": (4050, -2750),
        "buckshee": (4800, -500),
        "thunderstrike": (5550, -2600),
    }
else:
    # find PlayerStart to anchor the blocks grid
    ps = [a for a in actors if a.get_class().get_name() == "PlayerStart"]
    if ps:
        pl = ps[0].get_actor_location()
        px, py, pz = pl.x, pl.y, pl.z
    else:
        px, py, pz = 0.0, 0.0, 0.0
    P("BLOCKS_PLAYERSTART", px, py, pz)
    cx, cy = int(round(px / 100.0) * 100), int(round(py / 100.0) * 100)
    XS = list(range(cx - 2300, cx + 2301, 100))
    YS = list(range(cy - 2300, cy + 2301, 100))
    Z_TOP, Z_BOT = 5000.0, -3000.0
    FIXED_ALT = pz + 100.0
    spawns = {
        "ghost": (px - 700, py - 700),
        "delta": (px - 700, py - 400),
        "buckshee": (px - 700, py - 100),
        "thunderstrike": (px - 700, py + 200),
    }

P("GRID", len(XS), "x", len(YS), "=", len(XS) * len(YS))
P("FIXED_ALT", FIXED_ALT)

NRAY = 16
RAY_LEN = 300.0
DIRS = [(math.cos(2 * math.pi * i / NRAY), math.sin(2 * math.pi * i / NRAY)) for i in range(NRAY)]


def star_clearance(x, y, z):
    """16-ray horizontal star; returns (min_dist, median_dist, n_blocked)"""
    ds = []
    for (dx, dy) in DIRS:
        h = lt(x, y, z, x + dx * RAY_LEN, y + dy * RAY_LEN, z)
        if h is None:
            ds.append(RAY_LEN)
        else:
            hx, hy, _ = h[0]
            ds.append(math.hypot(hx - x, hy - y))
    ds_s = sorted(ds)
    return min(ds), ds_s[len(ds_s) // 2], sum(1 for d in ds if d < RAY_LEN - 1.0)


cells = {}
t_done = 0
for x in XS:
    for y in YS:
        fx, fy = float(x), float(y)
        g = lt(fx, fy, Z_TOP, fx, fy, Z_BOT)
        rec = {}
        if g is None:
            rec["g"] = None
            cells["%d,%d" % (x, y)] = rec
            t_done += 1
            continue
        gz = g[0][2]
        rec["g"] = round(gz, 1)
        rec["gl"] = g[1][:40]
        fz = gz + 100.0
        mn, md, nb = star_clearance(fx, fy, fz)
        rec["c"] = round(mn, 1)
        rec["cm"] = round(md, 1)
        rec["nb"] = nb
        # head clearance straight up
        up = lt(fx, fy, fz, fx, fy, fz + 300.0)
        rec["up"] = 300.0 if up is None else round(up[0][2] - fz, 1)
        # clearance at fixed absolute altitude (planner slice height)
        if FIXED_ALT > gz + 5.0:
            mn2, md2, nb2 = star_clearance(fx, fy, FIXED_ALT)
            rec["cf"] = round(mn2, 1)
        else:
            rec["cf"] = -1.0   # ground is above the fixed slice -> solid
        cells["%d,%d" % (x, y)] = rec
        t_done += 1
    P("col x=%d done cells=%d" % (x, t_done))

# ---- channel audit on a subsample: visibility vs camera vs simple-collision
audit = []
sample = [(XS[len(XS) // 2], YS[len(YS) // 2]),
          (XS[len(XS) // 3], YS[len(YS) // 3]),
          (XS[2 * len(XS) // 3], YS[2 * len(YS) // 3])]
for (x, y) in sample:
    a = lt(float(x), float(y), Z_TOP, float(x), float(y), Z_BOT, VIS, True)
    b = lt(float(x), float(y), Z_TOP, float(x), float(y), Z_BOT, CAM, True)
    c = lt(float(x), float(y), Z_TOP, float(x), float(y), Z_BOT, VIS, False)  # simple collision only
    col = lt_multi(float(x), float(y), Z_TOP, float(x), float(y), Z_BOT)
    audit.append({"xy": [x, y],
                  "vis_complex": a, "camera_complex": b, "vis_simple": c,
                  "multi_z": col})
P("AUDIT", json.dumps(audit, default=str))

# spawn point detail
spawn_detail = {}
for nm, (sx, sy) in spawns.items():
    g = lt(float(sx), float(sy), Z_TOP, float(sx), float(sy), Z_BOT)
    if g is None:
        spawn_detail[nm] = {"xy": [sx, sy], "ground": None}
        continue
    gz = g[0][2]
    mn, md, nb = star_clearance(float(sx), float(sy), gz + 100.0)
    spawn_detail[nm] = {"xy": [sx, sy], "ground": round(gz, 1), "on": g[1][:40],
                        "clr_min": round(mn, 1), "clr_med": round(md, 1), "n_blocked": nb}
P("SPAWNS", json.dumps(spawn_detail))

json.dump({"mode": MODE, "map": MAP, "xs": XS, "ys": YS,
           "fixed_alt": FIXED_ALT, "ray_len": RAY_LEN, "nray": NRAY,
           "actor_total": len(actors), "smc_total": n_smc, "smc_nocoll": n_smc_nocoll,
           "sublevels": sub_report, "audit": json.loads(json.dumps(audit, default=str)),
           "spawns": spawn_detail, "cells": cells}, open(OUT, "w"))
P("CLEARANCE_PROBE_OK", OUT)
