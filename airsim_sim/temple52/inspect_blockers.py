# Inspect the actors that dominate the temple's top-collision surface.
import json, os, unreal

OUT = os.environ["CP_OUT"]
PROG = open(OUT + ".log", "a")
def P(*a):
    s = " ".join(str(x) for x in a); print(s); PROG.write(s + "\n"); PROG.flush()

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
les.load_level("/Game/AncientTempleRuins/Levels/L_Showcase_01")
world = ues.get_editor_world()
actors = eas.get_all_level_actors()

TARGETS = ["SM_Tree_02_Blockout", "S_BCK_Head_01", "Cube214", "Cube217",
           "BPP_PLA_Rock_01_A7", "BPP_LI_FoliageCluster", "SM_Stairs_Round_800_800"]

rows = []
# also: global census of hidden-but-colliding, and Blockout/BCK naming
n_hidden_colliding = 0
hidden_examples = []
name_buckets = {}

for a in actors:
    lbl = a.get_actor_label()
    hit = any(t in lbl for t in TARGETS)
    smcs = a.get_components_by_class(unreal.StaticMeshComponent)
    if not smcs:
        continue
    for c in smcs:
        try:
            hidden = bool(c.get_editor_property("hidden_in_game"))
        except Exception:
            hidden = None
        try:
            vis = bool(c.get_editor_property("visible"))
        except Exception:
            vis = None
        try:
            ce = str(c.get_editor_property("collision_enabled"))
        except Exception:
            ce = "?"
        try:
            prof = str(c.get_editor_property("collision_profile_name"))
        except Exception:
            prof = "?"
        colliding = ("NO_COLLISION" not in ce.upper())
        if colliding and (hidden or vis is False):
            n_hidden_colliding += 1
            if len(hidden_examples) < 40:
                hidden_examples.append([lbl[:50], ce, prof, hidden, vis])
        for key in ("Blockout", "BCK", "Foliage", "Ivy", "Tree", "Rock", "Stairs", "Cube"):
            if key in lbl:
                b = name_buckets.setdefault(key, [0, 0, 0])
                b[0] += 1
                if colliding: b[1] += 1
                if hidden: b[2] += 1
        if hit:
            try:
                sm = c.get_editor_property("static_mesh")
                mesh_path = sm.get_path_name() if sm else None
                nanite = None
                try:
                    ns = sm.get_editor_property("nanite_settings")
                    nanite = bool(ns.get_editor_property("enabled"))
                except Exception:
                    pass
                bs = sm.get_editor_property("body_setup") if sm else None
                ctf = str(bs.get_editor_property("collision_trace_flag")) if bs else None
                nprim = None
                if bs:
                    ag = bs.get_editor_property("agg_geom")
                    try:
                        nprim = (len(ag.get_editor_property("box_elems")),
                                 len(ag.get_editor_property("sphere_elems")),
                                 len(ag.get_editor_property("convex_elems")))
                    except Exception:
                        nprim = None
            except Exception as e:
                mesh_path, ctf, nprim, nanite = "ERR " + str(e), None, None, None
            o = a.get_actor_bounds(False)
            org, ext = o[0], o[1]
            rows.append({"label": lbl[:60], "class": a.get_class().get_name(),
                         "mesh": str(mesh_path).split(".")[0][-70:],
                         "ctf": ctf, "simple_prims": nprim, "nanite": nanite,
                         "collision_enabled": ce, "profile": prof,
                         "hidden_in_game": hidden, "visible": vis,
                         "origin": [round(org.x), round(org.y), round(org.z)],
                         "extent": [round(ext.x), round(ext.y), round(ext.z)]})

P("N_HIDDEN_BUT_COLLIDING_COMPONENTS", n_hidden_colliding)
P("HIDDEN_EXAMPLES", json.dumps(hidden_examples))
P("NAME_BUCKETS (key -> [total_smc, colliding, hidden])", json.dumps(name_buckets))
P("TARGET_ROWS", len(rows))
for r in rows[:80]:
    P("ROW " + json.dumps(r))

json.dump({"hidden_colliding": n_hidden_colliding, "hidden_examples": hidden_examples,
           "name_buckets": name_buckets, "rows": rows}, open(OUT, "w"))
P("INSPECT_OK")
