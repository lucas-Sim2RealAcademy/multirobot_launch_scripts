import unreal
les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
les.load_level("/Game/AncientTempleRuins/Levels/L_Showcase_01")
world = ues.get_editor_world()
have = [a for a in eas.get_all_level_actors() if a.get_class().get_name() == "PlayerStart"]
if have:
    have[0].set_actor_location(unreal.Vector(5200, -1500, 860), False, False)
    print("MOVED existing PlayerStart")
else:
    ps = eas.spawn_actor_from_class(unreal.PlayerStart, unreal.Vector(5200, -1500, 860))
    print("SPAWNED PlayerStart", ps.get_actor_label())
ok = les.save_current_level()
print("SAVED", ok)
