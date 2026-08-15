# Runs HEADLESS in UE 5.6 (KungfuRender project): load a temple map READ-ONLY
# (never saved), grid line-trace the ground to build a height map for anchor
# selection, and summarize the actor population. Writes SCOUT_OUT json.
import collections
import json
import os
import unreal

MAP = os.environ.get("SCOUT_MAP", "/Game/AncientTempleRuins/Levels/L_Showcase_01")
OUT = os.environ["SCOUT_OUT"]
GRID = os.environ.get("SCOUT_GRID", "1800,7001,-5200,-601,200")
x0, x1, y0, y1, step = (int(v) for v in GRID.split(","))
PROG = open("/home/lucas/UE5/hercules-sim-big/mrq_work/temple/scout_progress.txt", "a")


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
P("LOADED", MAP, ok, world.get_name() if world else None)

counts = collections.Counter()
for a in eas.get_all_level_actors():
    counts[a.get_class().get_name()] += 1
P("ACTORS", json.dumps(dict(counts.most_common(30))))


def trace(x, y, z0=4000.0, z1=-2000.0):
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(x, y, z0), unreal.Vector(x, y, z1),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
        unreal.DrawDebugTrace.NONE, True)
    if hit:
        return round(hit.to_tuple()[4].z, 1)
    return None


grid = {}
n = 0
for x in range(x0, x1, step):
    for y in range(y0, y1, step):
        grid["%d,%d" % (x, y)] = trace(float(x), float(y))
        n += 1
P("TRACED", n)
json.dump({"map": MAP, "grid": grid, "step": step}, open(OUT, "w"))
P("SCOUT_OK", OUT)
