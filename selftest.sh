#!/bin/zsh
# Kernel + file self-test (primitives, booleans, split, rounding, M3/M24 fasteners, 3MF/STL/STEP).
exec "$(dirname "$0")/../build.sh" --selftest
