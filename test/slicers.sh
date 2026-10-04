#!/bin/bash
# The print samples opened by real slicers on macOS: ./test/slicers.sh SAMPLES_DIR
# Downloads the latest Bambu Studio, PrusaSlicer (Bambu Studio's parent: the same 3MF and STL readers) and UltiMaker Cura
# releases (gh, with GH_TOKEN set), then has each read every sample: Bambu Studio and PrusaSlicer report on each file
# (--info: size, volume, whether it's manifold); Cura's engine slices each STL. Prints what each says; exits non-zero when
# a reader that ran calls a sample broken.
set -uo pipefail
SAMPLES="$(cd "$1" && pwd)"
WORK="${TMPDIR:-/tmp}/bcad-slicers"
mkdir -p "$WORK"
bad=0

# Mounts the newest macOS release image of a repository; prints the mount point.
mount_latest() {
  local repo="$1" pattern="$2" name dmg mnt
  name=$(gh release view -R "$repo" --json assets -q '.assets[].name' | grep -iE "$pattern" | head -1)
  [ -n "$name" ] || { echo "no release asset of $repo matches $pattern" >&2; return 1; }
  dmg="$WORK/$name"
  [ -f "$dmg" ] || gh release download -R "$repo" -p "$name" -D "$WORK" >&2 || return 1
  mnt="$WORK/mnt-${repo##*/}"
  mkdir -p "$mnt"
  yes | hdiutil attach -nobrowse -readonly -noautoopen -mountpoint "$mnt" "$dmg" >/dev/null || return 1
  echo "$mnt"
  echo "  $repo: $name" >&2
}

# Bambu Studio and PrusaSlicer: --info on each file.
info() {
  local label="$1" app="$2" f out
  echo "— $label"
  "$app" --help 2>&1 | head -5
  for f in "$SAMPLES"/*.3mf "$SAMPLES"/*.stl; do
    out=$("$app" --info "$f" 2>&1 | tail -40)
    echo "  $(basename "$f"): $(echo "$out" | grep -iE "manifold|open_edges|volume|number_of_facets|error" | tr '\n' ' ')"
    # An STL of parts touching along a line can't be manifold (it has no point numbers to tell them apart).
    if echo "$out" | grep -qi "manifold = no" && [[ "$f" != *touching.stl ]]; then echo "  ✗ $label: $(basename "$f") not manifold"; bad=1; fi
  done
}

if mnt=$(mount_latest bambulab/BambuStudio 'mac.*\.dmg$'); then
  app=$(find "$mnt" -maxdepth 4 -path '*Contents/MacOS/*' -type f -perm -u+x | grep -i bambu | head -1)
  [ -n "$app" ] && info "Bambu Studio" "$app" || echo "Bambu Studio: no program in the image"
fi
if mnt=$(mount_latest prusa3d/PrusaSlicer 'mac.*\.dmg$'); then
  app=$(find "$mnt" -maxdepth 4 -path '*Contents/MacOS/*' -type f -perm -u+x | grep -i prusa | head -1)
  [ -n "$app" ] && info "PrusaSlicer" "$app" || echo "PrusaSlicer: no program in the image"
fi

# Cura: its engine slices each STL with its plain printer definition.
if mnt=$(mount_latest Ultimaker/Cura 'mac.*(arm|aarch).*\.dmg$|macos.*\.dmg$'); then
  echo "— UltiMaker Cura (its engine)"
  engine=$(find "$mnt" -maxdepth 5 -name CuraEngine -type f | head -1)
  defs=$(find "$mnt" -maxdepth 8 -name fdmprinter.def.json | head -1)
  if [ -n "$engine" ] && [ -n "$defs" ]; then
    defs=$(dirname "$defs")
    extr=$(dirname "$(find "$mnt" -maxdepth 8 -name fdmextruder.def.json | head -1)")
    for f in "$SAMPLES"/*.stl; do
      out=$(CURA_ENGINE_SEARCH_PATH="$defs:$extr" "$engine" slice -j "$defs/fdmprinter.def.json" -s machine_width=256 -s machine_depth=256 \
            -s machine_height=256 -e0 -j "$extr/fdmextruder.def.json" -l "$f" -o "$WORK/$(basename "$f" .stl).gcode" 2>&1)
      code=$?
      lines=$(wc -l < "$WORK/$(basename "$f" .stl).gcode" 2>/dev/null || echo 0)
      echo "  $(basename "$f"): exit $code, $lines lines of G-code $(echo "$out" | grep -iE "error|warning" | head -2 | tr '\n' ' ')"
      if [ "$code" -ne 0 ] || [ "$lines" -lt 100 ]; then echo "  ✗ Cura: $(basename "$f") not sliced"; bad=1; fi
    done
  else
    echo "Cura: no engine or definitions in the image"
  fi
fi
exit $bad
