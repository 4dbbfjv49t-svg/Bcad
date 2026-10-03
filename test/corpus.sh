#!/bin/bash
# Bcad's own engine on the saved corpus of cases (test/corpus/cases.txt): ./test/corpus.sh [--perturb] [--digest FILE]
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="${TMPDIR:-/tmp}/bcad-corpus"
c++ -std=c++17 -O2 -ffp-contract=off ${CORPUS_FLAGS:-} -I. test/corpus.cpp Engine/*.cpp -o "$OUT"
"$OUT" "$@"
