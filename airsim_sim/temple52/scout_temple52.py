# Headless UE 5.2.1 Blocks: load the temple showcase level, census + ground grid.
import collections, json, os, unreal

OUT = os.environ["SCOUT_OUT"]
PROG = open(os.environ.get("SCOUT_PROG", "/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/scout_t52_progress.txt"), "a")
def P(*a):
    s = " ".join(str(x) for x in a); print(s); PROG.write(s + "\n"); PROG.flush()

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
ok = les.load_level("/Game/AncientTempleRuins/Levels/L_Showcase_01")
world = ues.get_editor_world()
P("LOADED", ok, world.get_name() if world else None)
counts = collections.Counter(a.get_class().get_name() for a in eas.get_all_level_actors())
P("ACTORS", json.dumps(dict(counts.most_common(20))))
def trace(x, y):
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(x, y, 15000.0), unreal.Vector(x, y, -5000.0),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
        unreal.DrawDebugTrace.NONE, True)
    if hit:
        t = hit.to_tuple()
        return round(t[4].z, 0)
    return None
grid = {}
for x in range(1000, 9001, 400):
    for y in range(-6000, 2001, 400):
        grid[f"{x},{y}"] = trace(float(x), float(y))
P("TRACED", len(grid))
json.dump({"grid": grid}, open(OUT, "w"))
P("SCOUT52_OK")
