#!/usr/bin/env python3
"""Copies the shared design block of Design.swift (between the "Shared look" markers) into BPlayer and BSynth,
which keep an identical copy inside their single source files. Run it after changing the block here."""
import os, sys

BEGIN = '// MARK: - Shared look · begin'
END = '// MARK: - Shared look · end'
here = os.path.dirname(os.path.abspath(__file__))
design = open(os.path.join(here, 'Design.swift')).read()
block = design[design.index(BEGIN):design.index(END) + len(END)]
for path in ('../BPlayer/BPlayer.swift', '../BSynth/BSynth.swift'):
    p = os.path.normpath(os.path.join(here, path))
    src = open(p).read()
    if BEGIN not in src or END not in src:
        sys.exit(f'{p}: no shared block markers')
    new = src[:src.index(BEGIN)] + block + src[src.index(END) + len(END):]
    if new != src:
        open(p, 'w').write(new)
    print(('updated ' if new != src else 'same    ') + p)
