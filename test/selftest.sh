#!/bin/zsh
# Self-test: kernel (primitives, tori, booleans, split, rounding, M3/M24 fasteners), 3MF/STL/STEP, resizing and mouse events.
exec "$(dirname "$0")/../build.sh" --selftest
