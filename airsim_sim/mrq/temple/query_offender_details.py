# Headless UE 5.6 read-only query: details for the foliage-sweep offenders.
#  - Plane28: does it render at all (hidden flags, materials, bounds)?
#  - world AABBs of the flagged foliage instances (for the deterministic dodge)
import json
import unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple"
_PROG = open(W + "/query_offenders_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    _PROG.write(s + "\n")
    _PROG.flush()


les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
P("[q] load:", les.load_level("/Game/AncientTempleRuins/Levels/L_Showcase_01"))

ML = unreal.MathLibrary
WANT_ACTORS = {"Plane28", "LillyPadGiant35", "S_BCK_Head_01_PM3D_Cube3D3_8",
               "SM_Tree_02_Blockout5"}
WANT_ISM = {  # (actor_label, mesh_name, inst) from foliage_sweep_v2.json
    ("BPP_LI_FoliageCluster_15", "SM_Fern_01_F", 0),
    ("BPP_LI_FoliageCluster_15", "SM_Fern_01_G", 0),
    ("BPP_LI_FoliageCluster_15", "SM_Fern_01_B", 0),
    ("BPP_LI_FoliageCluster_15", "SM_Fern_01_B", 1),
    ("BPP_LI_FoliageCluster_15", "SM_Fern_01_C", 0),
    ("BPP_LI_FoliageCluster_15", "SM_Monstera_01_Cluster_02", 0),
    ("BPP_LI_FoliageCluster_15", "SM_Monstera_01_Cluster_03", 0),
    ("BPP_LI_FoliageCluster_15", "SM_Bush_01_B", 2),
    ("BPP_LI_FoliageCluster_46", "SM_Fern_01_H", 0),
}
out = {"ism": []}


def world_aabb(xf, bb):
    wmin = [1e18] * 3
    wmax = [-1e18] * 3
    for cx in (bb.min.x, bb.max.x):
        for cy in (bb.min.y, bb.max.y):
            for cz in (bb.min.z, bb.max.z):
                w = ML.transform_location(xf, unreal.Vector(cx, cy, cz))
                for i, v in enumerate((w.x, w.y, w.z)):
                    wmin[i] = min(wmin[i], v)
                    wmax[i] = max(wmax[i], v)
    return [round(v, 1) for v in wmin], [round(v, 1) for v in wmax]


for a in eas.get_all_level_actors():
    try:
        lbl = a.get_actor_label()
    except Exception:
        continue
    if lbl in WANT_ACTORS:
        P(f"[q] ===== {lbl} ({a.get_class().get_name()})")
        try:
            P("   actor hidden(game):", a.get_editor_property("hidden"))
            P("   actor hidden_ed:", a.is_temporarily_hidden_in_editor())
        except Exception as e:
            P("   hidden query fail:", e)
        o, ext = a.get_actor_bounds(False)
        P(f"   bounds origin=({o.x:.0f},{o.y:.0f},{o.z:.0f}) "
          f"ext=({ext.x:.0f},{ext.y:.0f},{ext.z:.0f})")
        for comp in a.get_components_by_class(unreal.StaticMeshComponent):
            try:
                P("   comp:", comp.get_name(),
                  "visible=", comp.get_editor_property("visible"),
                  "hidden_in_game=", comp.get_editor_property("hidden_in_game"),
                  "cast_shadow=", comp.get_editor_property("cast_shadow"))
                sm = comp.get_editor_property("static_mesh")
                P("   mesh:", sm.get_name() if sm else None)
                for mi, mat in enumerate(comp.get_materials()):
                    P(f"   mat[{mi}]:", mat.get_name() if mat else None)
                xf = comp.get_component_transform()
                t = xf.translation
                r = xf.rotation.rotator()
                s = xf.scale3d
                P(f"   xf loc=({t.x:.1f},{t.y:.1f},{t.z:.1f}) "
                  f"rot=({r.pitch:.1f},{r.yaw:.1f},{r.roll:.1f}) "
                  f"scale=({s.x:.2f},{s.y:.2f},{s.z:.2f})")
                if sm:
                    wmin, wmax = world_aabb(xf, sm.get_bounding_box())
                    P("   world_aabb:", wmin, wmax)
            except Exception as e:
                P("   comp dump fail:", e)
    # flagged ISM instances
    try:
        isms = list(a.get_components_by_class(unreal.InstancedStaticMeshComponent))
    except Exception:
        isms = []
    for comp in isms:
        sm = comp.get_editor_property("static_mesh")
        if sm is None:
            continue
        for (al, mn, ii) in WANT_ISM:
            if al == lbl and mn == sm.get_name() and ii < comp.get_instance_count():
                xf = comp.get_instance_transform(ii, True)
                wmin, wmax = world_aabb(xf, sm.get_bounding_box())
                rec = dict(actor=al, mesh=mn, inst=ii, wmin=wmin, wmax=wmax)
                out["ism"].append(rec)
                P(f"[q] ISM {al}/{mn}#{ii} aabb {wmin} {wmax}")

json.dump(out, open(W + "/offender_aabbs.json", "w"), indent=1)
P("[q] QUERY_OK")
