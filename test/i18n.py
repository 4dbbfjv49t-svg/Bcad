#!/usr/bin/env python3
"""Bcad's translations checked: python3 test/i18n.py

Every key in i18n.json has a text in each of the 19 languages besides English, with the same {placeholders} as the key
(and, where a text has plural forms, in each form). Every key the app's source names as a literal L("…") is there."""
import glob
import json
import os
import re
import sys

root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
table = json.load(open(os.path.join(root, "i18n.json"), encoding="utf-8"))
languages = ["uk", "cs", "de", "es", "fr", "it", "hu", "nl", "nb", "pl", "ro", "fi", "sv", "kk", "ka", "ar", "hi", "zh-Hans", "ja"]
holes = re.compile(r"\{[A-Za-z]+\}")
problems = []

for key, texts in table.items():
    want = sorted(holes.findall(key))
    for lang in languages:
        t = texts.get(lang)
        forms = t.values() if isinstance(t, dict) else [t]
        for form in forms:
            if not isinstance(form, str) or not form.strip():
                problems.append(f"{key!r}: no {lang} text")
            elif sorted(holes.findall(form)) != want and not (isinstance(t, dict) and "{n}" not in form and want == ["{n}"]):
                problems.append(f"{key!r}: {lang} has {sorted(holes.findall(form))}, not {want}")

used = set()
for path in glob.glob(os.path.join(root, "*.swift")) + glob.glob(os.path.join(root, "test", "*.swift")):
    source = open(path, encoding="utf-8").read()
    for m in re.finditer(r'\bL\("((?:[^"\\]|\\.)*)"', source):
        used.add(m.group(1).replace('\\"', '"'))
for key in sorted(used - set(table)):
    problems.append(f"{key!r}: used but not in i18n.json")

print(f"{len(table)} keys in {len(languages)} languages besides English; {len(used)} used by name")
for p in problems:
    print("✗", p)
print("ALL OK" if not problems else f"{len(problems)} problems")
sys.exit(1 if problems else 0)
