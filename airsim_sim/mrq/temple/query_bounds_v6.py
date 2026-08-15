# Headless UE 5.6 (KungfuRender, -nullrhi): dump world AABBs of every
# renderable component (SMA + ISM/HISM instances) within RANGE of the v6
# footprint center and reaching above the ground plane — the design-time
# occluder set for the s4/s5 camera solvers.  READ-ONLY.
import json
import os
import unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple"
_PROG = open(W + "/query_bounds_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    _PROG.write(s + "\n")
    _PROG.flush()


OUT = os.environ.get("OUT_JSON", W + "/occluder_aabbs_v6.json")
CX, CY = 5119.0, -1727.0
RANGE = 4500.0
Z_MIN = 950.0            # ignore ground clutter below drone/camera space
MAP_PATH = "/Game/AncientTempleRuins/Levels/L_Showcase_01"

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
ok = les.load_level(MAP_PATH)
P("QB_LOADED", ok)
ML = unreal.MathLibrary

out = []
n_act = 0
for a in eas.get_all_level_actors():
    n_act += 1
    try:
        lbl = a.get_actor_label()
    except Exception:
        lbl = "?"
    comps = []
    try:
        comps = list(a.get_components_by_class(unreal.StaticMeshComponent))
    except Exception:
        pass
    for comp in comps:
        try:
            sm = comp.get_editor_property("static_mesh")
        except Exception:
            sm = None
        if sm is None:
            continue
        bb = sm.get_bounding_box()
        if isinstance(comp, unreal.InstancedStaticMeshComponent):
            n = comp.get_instance_count()
            for i in range(n):
                xf = comp.get_instance_transform(i, True)
                wmin = [1e18] * 3
                wmax = [-1e18] * 3
                for cx in (bb.min.x, bb.max.x):
                    for cy in (bb.min.y, bb.max.y):
                        for cz in (bb.min.z, bb.max.z):
                            w = ML.transform_location(xf, unreal.Vector(cx, cy, cz))
                            for j, val in enumerate((w.x, w.y, w.z)):
                                wmin[j] = min(wmin[j], val)
                                wmax[j] = max(wmax[j], val)
                mx = (wmin[0] + wmax[0]) / 2
                my = (wmin[1] + wmax[1]) / 2
                if abs(mx - CX) > RANGE + (wmax[0] - wmin[0]) / 2:
                    continue
                if abs(my - CY) > RANGE + (wmax[1] - wmin[1]) / 2:
                    continue
                if wmax[2] < Z_MIN:
                    continue
                out.append({"mesh": sm.get_name(), "actor": lbl, "inst": i,
                            "wmin": [round(v, 1) for v in wmin],
                            "wmax": [round(v, 1) for v in wmax]})
        else:
            try:
                xf = comp.get_world_transform()
            except Exception:
                xf = a.get_actor_transform()
            wmin = [1e18] * 3
            wmax = [-1e18] * 3
            for cx in (bb.min.x, bb.max.x):
                for cy in (bb.min.y, bb.max.y):
                    for cz in (bb.min.z, bb.max.z):
                        w = ML.transform_location(xf, unreal.Vector(cx, cy, cz))
                        for j, val in enumerate((w.x, w.y, w.z)):
                            wmin[j] = min(wmin[j], val)
                            wmax[j] = max(wmax[j], val)
            mx = (wmin[0] + wmax[0]) / 2
            my = (wmin[1] + wmax[1]) / 2
            if abs(mx - CX) > RANGE + (wmax[0] - wmin[0]) / 2:
                continue
            if abs(my - CY) > RANGE + (wmax[1] - wmin[1]) / 2:
                continue
            if wmax[2] < Z_MIN:
                continue
            out.append({"mesh": sm.get_name(), "actor": lbl, "inst": -1,
                        "wmin": [round(v, 1) for v in wmin],
                        "wmax": [round(v, 1) for v in wmax]})

json.dump({"center": [CX, CY], "range": RANGE, "z_min": Z_MIN, "boxes": out},
          open(OUT, "w"))
P(f"QB_OK actors={n_act} boxes={len(out)} -> {OUT}")
