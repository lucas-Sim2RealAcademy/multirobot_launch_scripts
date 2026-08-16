"""Prepare the bamboo forest for 4-drone flight in UE 5.2.1.

Two blockers from the acquisition audit:
  1. bamboo meshes are complex-collision-only -> AirSim physics falls through
     (exactly the temple failure); generate convex hulls.
  2. demo placement is far too dense to fly: 116k clumps, median 2.31 m
     centre spacing between clumps that are 2-3 m wide, so no reliable >=1 m
     corridor at 1-2 m AGL.  Thin the instances and save a FLIGHT map.

Saving is safe here: the map is monolithic (14 actors, no OFPA/World
Partition), unlike the temple level whose save corrupted it.
"""
import math
import random
import unreal

OUT = "/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/bamboo.txt"
P = open(OUT, "a")

def log(msg):
    P.write(msg + "\n")
    P.flush()

al = unreal.EditorAssetLibrary
esl = unreal.EditorStaticMeshLibrary

# ---------- 1. collision on the bamboo meshes ----------
n_gen = 0
for p in al.list_assets("/Game/Bamboo_Forest/Meshes", recursive=True):
    ap = p.split(".")[0]
    a = al.load_asset(ap)
    if not isinstance(a, unreal.StaticMesh):
        continue
    name = ap.rsplit("/", 1)[-1].lower()
    if "fern" in name or "grass" in name:
        continue                       # soft ground clutter: leave alone
    try:
        if esl.get_simple_collision_count(a) > 0:
            continue
        ok = esl.set_convex_decomposition_collisions(a, 4, 12, 100000)
        if not ok:
            esl.add_simple_collisions(a, unreal.ScriptingCollisionShapeType.CAPSULE)
        al.save_asset(ap, only_if_is_dirty=False)
        n_gen += 1
    except Exception as e:
        log("collide FAIL %s: %s" % (name, e))
log("COLLISION generated=%d" % n_gen)

# ---------- 2. thin the forest and save a flight map ----------
les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
les.load_level("/Game/Bamboo_Forest/Maps/Bamboo_Forest_LOD_Map")

KEEP = 0.22          # 0.081 clumps/m2 * 0.22 -> ~4.9 m median spacing
rng = random.Random(7)
tot_before = tot_after = 0
for a in eas.get_all_level_actors():
    for c in a.get_components_by_class(unreal.InstancedStaticMeshComponent):
        sm = c.static_mesh
        nm = (sm.get_name().lower() if sm else "")
        n = c.get_instance_count()
        if n == 0:
            continue
        tot_before += n
        keep = KEEP if ("bamboo" in nm or "sapling" in nm) else 0.5
        drop = [i for i in range(n) if rng.random() > keep]
        # remove from the end so indices stay valid
        for i in sorted(drop, reverse=True):
            try:
                c.remove_instance(i)
            except Exception:
                pass
        tot_after += c.get_instance_count()
log("THINNED instances %d -> %d (keep bamboo=%.2f)" % (tot_before, tot_after, KEEP))

# PlayerStart at the landscape centre so AirSim has a spawn datum
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
hit = unreal.SystemLibrary.line_trace_single(
    world, unreal.Vector(0, 0, 20000), unreal.Vector(0, 0, -20000),
    unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
    unreal.DrawDebugTrace.NONE, True)
gz = hit.to_tuple()[4].z if hit else 0.0
have = [a for a in eas.get_all_level_actors()
        if a.get_class().get_name() == "PlayerStart"]
if have:
    have[0].set_actor_location(unreal.Vector(0, 0, gz + 150), False, False)
else:
    eas.spawn_actor_from_class(unreal.PlayerStart, unreal.Vector(0, 0, gz + 150))
log("PLAYERSTART at z=%.0f (ground %.0f)" % (gz + 150, gz))

ok = unreal.EditorLevelLibrary.save_current_level()
log("SAVED_FLIGHT_MAP %s" % ok)
log("BAMBOO_PREP_DONE")
