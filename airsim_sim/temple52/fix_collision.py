import unreal
P = open("/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/fixcoll.txt", "a")
al = unreal.EditorAssetLibrary
n = n_set = 0
for path in al.list_assets("/Game/AncientTempleRuins", recursive=True):
    ap = path.split(".")[0]
    a = al.load_asset(ap)
    if not isinstance(a, unreal.StaticMesh):
        continue
    n += 1
    bs = a.get_editor_property("body_setup")
    if bs is None:
        continue
    try:
        if str(bs.get_editor_property("collision_trace_flag")) != "CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE":
            bs.set_editor_property("collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
            a.modify()
            al.save_asset(ap, only_if_is_dirty=False)
            n_set += 1
    except Exception as e:
        P.write(f"FAIL {ap}: {e}\n")
    if n % 200 == 0:
        P.write(f"progress {n} meshes, {n_set} updated\n"); P.flush()
P.write(f"DONE meshes={n} updated={n_set}\n")
P.flush()
