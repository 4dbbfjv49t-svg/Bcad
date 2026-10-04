#!/usr/bin/env python3
"""Bcad's print files read the way slicers read them: python3 test/printcheck.py DIR

DIR holds the self-test's print samples (NAME.3mf and NAME.stl) and expect.json saying what each holds. Each 3MF is read
by lib3mf, the 3MF Consortium's own reader, in strict mode, and each object in it must be:
  - named and coloured as its body;
  - manifold and oriented (lib3mf's own check), watertight and wound one way (trimesh's);
  - the body's volume;
  - on the bed, once its build item puts the bed's front left corner at the origin (as Cura and Bambu Studio place it).
The 3MF must also carry a thumbnail. Each STL, its points joined by place as slicers join them, must hold every triangle,
the bodies' volume, and be watertight where no parts touch along a line (an STL can't keep them apart there).
Needs: pip install lib3mf trimesh numpy
"""
import json
import os
import struct
import sys

import lib3mf
import numpy as np
import trimesh

failures = 0


def check(name, ok, note=""):
    global failures
    print(("✓" if ok else "✗"), name, note)
    if not ok:
        failures += 1


def close(a, b, rel=1e-6):
    return abs(a - b) <= rel * max(abs(a), abs(b), 1e-9)


def read3mf(path):
    """The model's objects (name, colour, points, triangles, manifold), its items' moves, warnings, thumbnail."""
    wrapper = lib3mf.get_wrapper()
    model = wrapper.CreateModel()
    reader = model.QueryReader("3mf")
    reader.SetStrictModeActive(True)
    reader.ReadFromFile(path)
    warnings = [reader.GetWarning(i)[1] for i in range(reader.GetWarningCount())]
    objects = {}
    it = model.GetMeshObjects()
    while it.MoveNext():
        o = it.GetCurrentMeshObject()
        res, prop, has = o.GetObjectLevelProperty()
        color = None
        if has:
            c = model.GetBaseMaterialGroupByID(res).GetDisplayColor(prop)
            color = "#%02X%02X%02X" % (c.Red, c.Green, c.Blue)
        points = np.array([v.Coordinates[0:3] for v in o.GetVertices()], dtype=np.float64)
        triangles = np.array([t.Indices[0:3] for t in o.GetTriangleIndices()], dtype=np.int64)
        objects[o.GetResourceID()] = {"name": o.GetName(), "color": color, "points": points, "triangles": triangles,
                                      "manifold": o.IsManifoldAndOriented()}
    items = []
    bi = model.GetBuildItems()
    while bi.MoveNext():
        b = bi.GetCurrent()
        f = b.GetObjectTransform().Fields
        m = np.array([[f[r][c] for c in range(3)] for r in range(4)])  # 3MF's rows: three of turning, the move last
        items.append((b.GetObjectResourceID(), m))
    return objects, items, warnings, model.HasPackageThumbnailAttachment(), model.GetUnit()


def readstl(path):
    """Triangles as slicers take them: points joined where they're one in place."""
    data = open(path, "rb").read()
    n = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * n:
        return None
    raw = np.frombuffer(data, dtype=np.uint8, offset=84).reshape(n, 50)[:, 12:48].copy().view("<f4").reshape(n, 3, 3)
    return trimesh.Trimesh(vertices=raw.reshape(-1, 3).astype(np.float64), faces=np.arange(3 * n).reshape(n, 3), process=True)


def main(folder):
    expect = json.load(open(os.path.join(folder, "expect.json")))
    bed = expect["bed"]
    for sample in expect["samples"]:
        name, want = sample["name"], sample["objects"]
        print(f"— {name}")
        path = os.path.join(folder, name + ".3mf")
        try:
            objects, items, warnings, thumbnail, unit = read3mf(path)
        except Exception as e:  # noqa: BLE001 (any refusal is the finding)
            check(f"{name}.3mf read by lib3mf in strict mode", False, str(e))
            continue
        check(f"{name}.3mf read by lib3mf in strict mode, without warnings", not warnings and unit == lib3mf.ModelUnit.MilliMeter, "; ".join(warnings))
        check(f"{name}.3mf has a thumbnail", thumbnail)
        check(f"{name}.3mf: an item for each body", len(items) == len(want) == len(objects), f"{len(items)} items, {len(objects)} objects, {len(want)} bodies")
        for (rid, move), w in zip(items, want):
            o = objects.get(rid)
            if o is None:
                check(f"{name}: {w['name']}", False, "no object for its item")
                continue
            mesh = trimesh.Trimesh(vertices=o["points"], faces=o["triangles"], process=False)
            placed = o["points"] @ move[:3] + move[3]
            lo, hi = placed.min(axis=0), placed.max(axis=0)
            on_bed = all(lo >= -1e-6) and hi[0] <= bed[0] + 1e-6 and hi[1] <= bed[1] + 1e-6 and hi[2] <= bed[2] + 1e-6
            notes = []
            if o["name"] != w["name"]:
                notes.append(f"named {o['name']!r}")
            if o["color"] != w["color"]:
                notes.append(f"coloured {o['color']}")
            if not o["manifold"]:
                notes.append("not manifold and oriented (lib3mf)")
            if not (mesh.is_watertight and mesh.is_winding_consistent):
                notes.append("not watertight or wound two ways (trimesh)")
            if not close(mesh.volume, w["volume"]):
                notes.append(f"volume {mesh.volume:.6f}, not {w['volume']:.6f}")
            if not np.allclose(move[:3], np.eye(3)) or not np.allclose(move[3], [bed[0] / 2, bed[1] / 2, 0]):
                notes.append(f"moved by {move.tolist()}")
            if sample.get("onBed", True) and not on_bed:
                notes.append(f"off the bed: {lo.round(3).tolist()} … {hi.round(3).tolist()}")
            check(f"{name}: {w['name']} as made, {len(o['triangles'])} triangles", not notes, "; ".join(notes))
        stl = readstl(os.path.join(folder, name + ".stl"))
        triangles = sum(len(o["triangles"]) for o in objects.values())
        volume = sum(w["volume"] for w in want)
        if stl is None:
            check(f"{name}.stl reads", False, "its length isn't its triangles'")
            continue
        notes = []
        if len(stl.faces) != triangles:
            notes.append(f"{len(stl.faces)} triangles, not {triangles}")
        if not close(stl.volume, volume):
            notes.append(f"volume {stl.volume:.6f}, not {volume:.6f}")
        if sample.get("stlWatertight", True) and not (stl.is_watertight and stl.is_winding_consistent):
            notes.append("not watertight or wound two ways")
        check(f"{name}.stl as slicers join it", not notes, "; ".join(notes))
    print("ALL OK" if failures == 0 else f"{failures} FAILED")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "."))
