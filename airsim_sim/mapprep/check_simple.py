"""Do the temple meshes have SIMPLE collision primitives at all?"""
import unreal

OUT = "/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/simple.txt"
P = open(OUT, "a")
al = unreal.EditorAssetLibrary
targets = ["SM_GS_DirtStony_01c", "SM_Floor_01_Stairs_Linear_256_256_01_A",
           "SM_Floor_01_Linear_512_512_01_A"]
n_zero = n_have = 0
checked = 0
for p in al.list_assets("/Game/AncientTempleRuins", recursive=True):
    ap = p.split(".")[0]
    a = al.load_asset(ap)
    if not isinstance(a, unreal.StaticMesh):
        continue
    checked += 1
    try:
        nprim = unreal.EditorStaticMeshLibrary.get_simple_collision_count(a)
    except Exception as e:
        P.write("api_fail %s\n" % e)
        break
    if nprim == 0:
        n_zero += 1
        if n_zero <= 6:
            P.write("no-simple: %s\n" % ap.rsplit("/", 1)[-1])
    else:
        n_have += 1
    if checked >= 120:
        break
P.write("SIMPLE_AUDIT checked=%d zero_simple=%d have_simple=%d\n"
        % (checked, n_zero, n_have))
P.flush()
