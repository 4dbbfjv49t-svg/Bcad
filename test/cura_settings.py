#!/usr/bin/env python3
"""Every setting UltiMaker Cura's engine reads, worked out as Cura itself does, each `key=value` ended by a NUL (some
values, such as the start G-code, run over several lines):
python3 test/cura_settings.py fdmprinter.def.json fdmextruder.def.json [key=value ...]

The engine on its own takes only the defaults written in the files, and stops on any setting left without one; Cura works
out each from its formula (most follow from a few others: the layer height, the nozzle, the walls). Given key=value pairs
are fixed first. A formula this can't work out (it names something only Cura's window knows) keeps its default."""
import json
import math
import re
import sys

leaves = {}


def walk(node):
    for key, s in node.items():
        if s.get("type") != "category":
            leaves[key] = s
        walk(s.get("children", {}))


for path in sys.argv[1:3]:
    walk(json.load(open(path, encoding="utf-8")).get("settings", {}))
fixed = dict(a.split("=", 1) for a in sys.argv[3:])

values = {}
busy = set()


def typed(s, v):
    t = s.get("type")
    try:
        if t == "int":
            return int(round(float(v)))
        if t == "float":
            return float(v)
        if t == "bool":
            return v if isinstance(v, bool) else str(v).lower() in ("true", "1", "yes", "on")
        if t in ("extruder", "optional_extruder"):
            return int(v)
    except (TypeError, ValueError):
        pass
    return v


def resolve(key):
    if key in values:
        return values[key]
    s = leaves.get(key)
    if s is None:
        raise KeyError(key)
    if key in fixed:
        values[key] = typed(s, fixed[key])
        return values[key]
    if key in busy:
        raise RecursionError(key)
    busy.add(key)
    v = s.get("default_value")
    formula = s.get("value")
    if isinstance(formula, str):
        scope = {"math": math, "extruderValue": lambda e, k: resolve(k), "extruderValues": lambda k: [resolve(k)],
                 "resolveOrValue": resolve, "anyExtruderWithMaterial": lambda k: 0, "anyExtruderNrWithOrDefault": lambda k: 0,
                 "defaultExtruderPosition": lambda: "0", "valueFromContainer": lambda i: None, "valueFromExtruderContainer": lambda i: None}
        try:
            for name in set(re.findall(r"[A-Za-z_]\w*", formula)):
                if name in leaves and name not in scope:
                    scope[name] = resolve(name)
            v = eval(formula, scope)
        except Exception:
            pass
    elif formula is not None:
        v = formula
    busy.discard(key)
    values[key] = typed(s, v)
    return values[key]


def text(v):
    if isinstance(v, bool):
        return "True" if v else "False"
    if isinstance(v, float):
        return repr(v)
    return str(v)


for key in leaves:
    try:
        v = resolve(key)
    except Exception:
        continue
    if v is None:
        continue
    print(f"{key}={text(v)}", end="\0")
