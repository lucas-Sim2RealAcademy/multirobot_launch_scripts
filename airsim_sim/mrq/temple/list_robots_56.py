# HEADLESS (nullrhi): list all actors in L_Showcase_01 that carry a skeletal or
# poseable mesh component (the kungfu robots), with label, class and location.
import unreal

PROG = open("/home/lucas/UE5/hercules-sim-big/mrq_work/temple/robots_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    PROG.write(s + "\n")
    PROG.flush()


les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
les.load_level("/Game/AncientTempleRuins/Levels/L_Showcase_01")
world = ues.get_editor_world()
P("LOADED", world.get_name())

n = 0
for a in eas.get_all_level_actors():
    comps = []
    try:
        for c in a.get_components_by_class(unreal.SkeletalMeshComponent):
            comps.append("Skel:" + c.get_name())
    except Exception:
        pass
    try:
        for c in a.get_components_by_class(unreal.PoseableMeshComponent):
            comps.append("PMC:" + c.get_name())
    except Exception:
        pass
    if comps:
        L = a.get_actor_location()
        hid = ""
        try:
            hid = " hidden_ed=%s hidden_game=%s" % (a.is_temporarily_hidden_in_editor(),
                                                    a.get_editor_property("actor_hidden_in_game"))
        except Exception:
            pass
        P(f"ROBOT label={a.get_actor_label()} class={a.get_class().get_name()} "
          f"loc=({L.x:.0f},{L.y:.0f},{L.z:.0f}) comps={comps}{hid}")
        n += 1
P("TOTAL", n)
P("LIST_OK")
