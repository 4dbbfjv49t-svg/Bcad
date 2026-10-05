#!/bin/bash
# A built Bcad as a person uses it: launched empty, a file opened into it, a cold launch with the file (a Finder
# double-click), the file opened again. Each time: running, a window on screen, the file opened; no crash at the end.
# With "save", Bcad saves the file back each time it opens it (BCAD_CHECK_SAVE), which shows it may write there
# (inside the App Store's sandbox too). Screenshots, logs and crash reports go to RESULTS/NAME.
# Usage: test/launch.sh APP FILE RESULTS NAME [save]
set -u
APP="$1" FILE="$2" NAME="$4" SAVE="${5:-}"
R="$3/$NAME"
mkdir -p "$R/shots" "$R/crashes"
ls -la "$FILE"
WINDOWS="$R/windows"
swiftc -O "$(dirname "$0")/windows.swift" -o "$WINDOWS"
rm -f ~/Library/Logs/DiagnosticReports/*Bcad* 2>/dev/null || true
START=$(date "+%Y-%m-%d %H:%M:%S")
ENV=()
[ "$SAVE" = save ] && ENV=(--env BCAD_CHECK_SAVE=1)
fail=0

snap() {
  sleep "$2"
  screencapture -x "$R/shots/$NAME-$1.png" || echo "screencapture failed"
}
count() {
  log show --start "$START" --style compact --predicate 'subsystem == "Bcad"' 2>/dev/null | grep -c "$1 $(basename "$FILE")" || true
}
# Running, with a window on screen, and the file opened (and saved back) as often as asked by now.
expect() {
  local windows opened saved
  windows=$("$WINDOWS" Bcad)
  opened=$(count Opened)
  saved=$(count Saved)
  local want_saved=0
  [ "$SAVE" = save ] && want_saved=$2
  if pgrep -x Bcad >/dev/null && [ "$windows" -ge 1 ] && [ "$opened" -ge "$2" ] && [ "$saved" -ge "$want_saved" ]; then
    echo "✓ $NAME, $1: running, $windows window(s), opened $opened time(s), saved $saved"
  else
    echo "✗ $NAME, $1: running $(pgrep -x Bcad >/dev/null && echo yes || echo no), $windows window(s), opened $opened time(s), saved $saved, wanted $2"
    fail=1
  fi
}
quit() {
  pkill -TERM -x Bcad || true
  for i in $(seq 1 50); do pgrep -x Bcad >/dev/null || break; sleep 0.2; done
  pkill -KILL -x Bcad 2>/dev/null || true
}

echo "▸ $NAME 1 launch empty"
open ${ENV[@]+"${ENV[@]}"} --stdout "$R/1.out" --stderr "$R/1.err" "$APP"
snap 1-empty 15
expect "launched" 0

echo "▸ $NAME 2 open the file in the running app"
open -a "$APP" "$FILE"
snap 2-open-running-10s 10
snap 2-open-running-40s 30
expect "a file opened in the running app" 1
quit

echo "▸ $NAME 3 cold launch with the file (Finder double-click)"
open ${ENV[@]+"${ENV[@]}"} -a "$APP" --stdout "$R/3.out" --stderr "$R/3.err" "$FILE"
snap 3-cold-10s 10
snap 3-cold-40s 30
expect "launched with a file" 2

echo "▸ $NAME 4 open the same file again"
open -a "$APP" "$FILE"
snap 4-reopen-30s 30
expect "the same file opened again" 3
quit

log show --start "$START" --style compact --predicate 'process == "Bcad"' > "$R/unified.log" 2>&1 || true
tail -60 "$R/unified.log"
if cp ~/Library/Logs/DiagnosticReports/*Bcad* "$R/crashes/" 2>/dev/null; then echo "✗ $NAME: Bcad crashed"; fail=1; else echo "✓ $NAME: no crash reports"; fi
for f in "$R"/*.out "$R"/*.err; do echo "── $f"; cat "$f"; done
exit $fail
