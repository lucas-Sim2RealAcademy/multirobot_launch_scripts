# Headless UE 5.2 Blocks: dump PlayerStart + all StaticMeshActor world AABBs.
import json, unreal

OUT = "/home/lucas/hercules-sim/tools/gt/blocks_gt.json"
PROG = open("/home/lucas/hercules-sim/tools/gt/harvest_progress.txt", "a")
def P(*a):
    s = " ".join(str(x) for x in a); print(s); PROG.write(s + "\n"); PROG.flush()

eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
les.load_level("/Game/FlyingCPP/Maps/FlyingExampleMap")
boxes, pstart = [], None
for a in eas.get_all_level_actors():
    cn = a.get_class().get_name()
    lbl = a.get_actor_label()
    if cn == "PlayerStart":
        l = a.get_actor_location()
        pstart = [l.x, l.y, l.z]
        continue
    if not a.get_components_by_class(unreal.StaticMeshComponent):
        continue
    o, e = a.get_actor_bounds(False)
    if max(e.x, e.y) > 30000:   # sky sphere / ground plane megameshes
        boxes.append({"label": lbl, "cls": cn, "big": True,
                      "min": [o.x-e.x, o.y-e.y, o.z-e.z], "max": [o.x+e.x, o.y+e.y, o.z+e.z]})
        continue
    boxes.append({"label": lbl, "cls": cn,
                  "min": [round(o.x-e.x,1), round(o.y-e.y,1), round(o.z-e.z,1)],
                  "max": [round(o.x+e.x,1), round(o.y+e.y,1), round(o.z+e.z,1)]})
P("PSTART", pstart, "boxes", len(boxes))
json.dump({"player_start": pstart, "boxes": boxes}, open(OUT, "w"))
P("GT_OK")
