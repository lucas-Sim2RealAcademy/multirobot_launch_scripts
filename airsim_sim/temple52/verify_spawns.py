import unreal
P = open("/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/vsp.txt", "a")
les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
les.load_level("/Game/AncientTempleRuins/Levels/L_Showcase_01")
world = ues.get_editor_world()
def top(x, y):
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(x, y, 4000), unreal.Vector(x, y, -3000),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [], unreal.DrawDebugTrace.NONE, True)
    if not hit: return None
    t = hit.to_tuple()
    try: lbl = t[9].get_actor_label()
    except Exception: lbl = "?"
    return (t[4].z, lbl)
cands = {
 "ghost":      [(6450,-2450),(6300,-2300),(6150,-2150),(6000,-2450)],
 "delta":      [(4050,-2750),(4200,-2600),(4350,-2450)],
 "buckshee":   [(5550,-200),(5100,-350),(4800,-500),(5250,-650),(4950,-200)],
 "thunderstrike": [(5850,-2750),(5700,-2900),(5550,-2600),(6000,-2900)],
}
for name, pts in cands.items():
    for (x, y) in pts:
        r = top(x, y)
        if r and 700 <= r[0] <= 880 and ("Floor" in r[1] or "floor" in r[1] or "Tile" in r[1] or "Ground" in r[1] or "S_" in r[1]):
            P.write(f"{name}: OK ({x},{y}) z={r[0]:.0f} on {r[1][:30]}\n")
            break
        P.write(f"{name}: reject ({x},{y}) -> {r}\n")
P.flush()
