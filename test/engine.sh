#!/bin/zsh
# Bcad's geometry engine against exact maths: ./test/engine.sh
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="${TMPDIR:-/tmp}/bcad-engine-test"
c++ -std=c++17 -O2 -ffp-contract=off -I. test/engine.cpp Engine/*.cpp -o "$OUT"
"$OUT"
