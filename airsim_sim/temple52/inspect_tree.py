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

WANT = ["SM_Tree_02_Blockout5", "S_BCK_Head_01_PM3D_Cube3D3_8", "BPP_PLA_Rock_01_A7", "Cube217"]
out = []
for a in eas.get_all_level_actors():
    lbl = a.get_actor_label()
    if lbl not in WANT:
        continue
    o, e = a.get_actor_bounds(False)
    rec = {"label": lbl, "class": a.get_class().get_name(),
           "loc": [round(a.get_actor_location().x), round(a.get_actor_location().y), round(a.get_actor_location().z)],
           "scale": [round(a.get_actor_scale3d().x,2), round(a.get_actor_scale3d().y,2), round(a.get_actor_scale3d().z,2)],
           "bounds_origin": [round(o.x), round(o.y), round(o.z)],
           "bounds_extent": [round(e.x), round(e.y), round(e.z)],
           "components": []}
    for c in a.get_components_by_class(unreal.SceneComponent):
        cd = {"cls": c.get_class().get_name(), "name": c.get_name()}
        try: cd["hidden_in_game"] = bool(c.get_editor_property("hidden_in_game"))
        except Exception: pass
        try: cd["visible"] = bool(c.get_editor_property("visible"))
        except Exception: pass
        if isinstance(c, unreal.StaticMeshComponent):
            sm = c.get_editor_property("static_mesh")
            cd["mesh"] = sm.get_path_name() if sm else None
            try: cd["collision_enabled"] = str(c.get_collision_enabled())
            except Exception as ex: cd["collision_enabled"] = "ERR " + str(ex)
            try: cd["collision_object_type"] = str(c.get_collision_object_type())
            except Exception: pass
            try: cd["collision_profile"] = str(c.get_collision_profile_name())
            except Exception: pass
            try: cd["resp_visibility"] = str(c.get_collision_response_to_channel(unreal.CollisionChannel.ECC_VISIBILITY))
            except Exception: pass
            try: cd["resp_pawn"] = str(c.get_collision_response_to_channel(unreal.CollisionChannel.ECC_PAWN))
            except Exception: pass
            if sm:
                bs = sm.get_editor_property("body_setup")
                if bs:
                    cd["ctf"] = str(bs.get_editor_property("collision_trace_flag"))
                    ag = bs.get_editor_property("agg_geom")
                    try:
                        cd["simple"] = [len(ag.get_editor_property("box_elems")),
                                        len(ag.get_editor_property("sphere_elems")),
                                        len(ag.get_editor_property("convex_elems")),
                                        len(ag.get_editor_property("sphyl_elems"))]
                    except Exception: pass
                try: cd["num_tris"] = sm.get_num_triangles(0)
                except Exception: pass
            if isinstance(c, unreal.InstancedStaticMeshComponent):
                try: cd["instances"] = c.get_instance_count()
                except Exception: pass
        rec["components"].append(cd)
    out.append(rec)
    P("FOUND " + json.dumps(rec)[:4000])

json.dump(out, open(OUT, "w"), indent=1)
P("TREE_OK")
