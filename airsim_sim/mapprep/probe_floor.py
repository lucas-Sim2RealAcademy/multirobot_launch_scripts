"""What is physically under each spawn, and is it solid?"""
import unreal

OUT = "/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/floor.txt"
P = open(OUT, "a")
les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
les.load_level("/Game/AncientTempleRuins/Levels/L_Showcase_01")
world = ues.get_editor_world()

SPAWNS = {"ghost": (6150, -2150), "delta": (4400, -2300),
          "buckshee": (4600, -1300), "thunderstrike": (5300, -2300)}
for name, (x, y) in SPAWNS.items():
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(x, y, 4000), unreal.Vector(x, y, -3000),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
        unreal.DrawDebugTrace.NONE, True)
    if not hit:
        P.write("%s: NO HIT\n" % name)
        continue
    t = hit.to_tuple()
    comp = t[8]
    actor = t[9]
    try:
        lbl = actor.get_actor_label()
    except Exception:
        lbl = "?"
    line = "%s: z=%.0f actor=%s" % (name, t[4].z, lbl)
    try:
        sm = comp.static_mesh
        line += " mesh=%s" % (sm.get_name() if sm else "None")
        line += " enabled=%s profile=%s" % (comp.get_collision_enabled(),
                                            comp.get_collision_profile_name())
        if sm:
            bs = sm.get_editor_property("body_setup")
            line += " trace=%s" % (bs.get_editor_property("collision_trace_flag")
                                   if bs else "NOBODY")
    except Exception as e:
        line += " introspect_fail=%s" % e
    P.write(line + "\n")
P.write("FLOOR_DONE\n")
P.flush()
