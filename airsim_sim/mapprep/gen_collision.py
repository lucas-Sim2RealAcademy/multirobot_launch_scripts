"""Generate SIMPLE collision for the temple meshes a drone can hit.

The Fab pack is Nanite-first and ships no simple collision primitives, so
AirSim's flight physics falls straight through the world (line traces hit,
rigid bodies do not).  Auto-generate convex hulls for structural meshes only
(floors, stairs, walls, pillars, roofs, rocks) -- vegetation stays
pass-through, which is both cheaper and physically sensible for a drone.
"""
import unreal

OUT = "/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/gencoll.txt"
P = open(OUT, "a")
al = unreal.EditorAssetLibrary
esl = unreal.EditorStaticMeshLibrary

STRUCT = ("floor", "stair", "step", "wall", "pillar", "column", "platform",
          "tile", "brick", "roof", "ground", "dirt", "stone", "rock", "path",
          "_gs_", "hw_", "slab", "base", "arch", "ruin", "block", "head",
          "cliff", "mountain", "border", "fence", "beam", "railing")
SKIP = ("ivy", "fern", "monstera", "bush", "palm", "creeper", "leaf", "vine",
        "moss", "cattail", "reed", "flower", "lilly", "ceiba", "tree", "grass",
        "cloth", "banner", "rope", "decal")

done = fail = skipped = 0
for p in al.list_assets("/Game/AncientTempleRuins", recursive=True):
    ap = p.split(".")[0]
    name = ap.rsplit("/", 1)[-1].lower()
    if any(k in name for k in SKIP):
        continue
    if not any(k in name for k in STRUCT):
        continue
    a = al.load_asset(ap)
    if not isinstance(a, unreal.StaticMesh):
        continue
    try:
        if esl.get_simple_collision_count(a) > 0:
            skipped += 1
            continue
        ok = esl.set_convex_decomposition_collisions(a, 8, 16, 100000)
        if not ok:
            ok = esl.add_simple_collisions(a, unreal.ScriptingCollisionShapeType.BOX)
        al.save_asset(ap, only_if_is_dirty=False)
        done += 1
        if done % 20 == 0:
            P.write("progress %d generated\n" % done)
            P.flush()
    except Exception as e:
        fail += 1
        if fail <= 5:
            P.write("FAIL %s: %s\n" % (name, e))
P.write("GENCOLL_DONE generated=%d already_had=%d failed=%d\n"
        % (done, skipped, fail))
P.flush()
