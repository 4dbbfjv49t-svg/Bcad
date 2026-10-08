#!/usr/bin/env python3
"""The app's Info.plist texts in each language: OUT/<lang>.lproj/InfoPlist.strings (UTF-16), from i18n.json.
Usage: python3 appstore/plist_strings.py OUT LANG..."""
import json
import os
import sys

root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
table = json.load(open(os.path.join(root, "i18n.json"), encoding="utf-8"))
esc = lambda v: v.replace("\\", "\\\\").replace('"', '\\"')
out = sys.argv[1]
for lang in sys.argv[2:]:
    folder = os.path.join(out, f"{lang}.lproj")
    os.makedirs(folder, exist_ok=True)
    lines = ""
    for name in ["3MF model", "STL model", "OBJ model", "PLY model", "USDZ model"]:
        text = name if lang == "en" else table.get(name, {}).get(lang, name)
        lines += f'"{name}" = "{esc(text)}";\n'
    open(os.path.join(folder, "InfoPlist.strings"), "w", encoding="utf-16").write(lines)
