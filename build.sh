#!/bin/zsh
# Builds /Applications/Bcad.app (or the given .app). ./build.sh --selftest builds and runs the kernel/file self-test instead.
# --engine own: on Bcad's own geometry engine (Engine/, nothing from OpenCascade) instead of OpenCascade; so far it makes,
# measures, merges and splits shapes, and says what it can't do yet (the self-test skips those checks and says so).
set -euo pipefail

# Interface languages, in the order of the in-app menu (English first).
LANGS=(en uk cs de es fr it hu nl nb pl ro fi sv kk ka ar hi zh-Hans ja)

cd "$(dirname "$0")"
SELFTEST=0
ENGINE=occt
while [[ "${1:-}" == --* ]]; do
  case "$1" in
    --selftest) SELFTEST=1 ;;
    --engine) ENGINE="${2:-}"; shift ;;
    *) echo "✗ Unknown option $1"; exit 1 ;;
  esac
  shift
done
if [[ "$ENGINE" != occt && "$ENGINE" != own ]]; then
  echo "✗ --engine is occt or own"
  exit 1
fi
TARGET="${1:-/Applications/Bcad.app}"
if [[ "$TARGET" != *.app ]]; then
  echo "✗ Target must end with .app"
  exit 1
fi
SDKV="$(xcrun --sdk macosx --show-sdk-version)"
if (( ${SDKV%%.*} < 26 )); then
  echo "✗ Needs the macOS 26 SDK or newer (Xcode 26 or its Command Line Tools); found $SDKV"
  exit 1
fi

OCCT="$PWD/Vendor/occt"
[[ "$ENGINE" == occt ]] && ./occt.sh
SDK="$(xcrun --sdk macosx --show-sdk-path)"

WORK="$(mktemp -d "${TMPDIR:-/tmp}/bcad.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT

echo "▸ Compiling the geometry kernel ($ENGINE)"
CXX=(clang++ -std=c++17 -isysroot "$SDK" -target arm64-apple-macos26.0 -I"$PWD")
KERNEL=()
LIBS=()
FLAGS=()
if [[ "$ENGINE" == occt ]]; then
  "${CXX[@]}" -O2 -isystem "$OCCT/include/opencascade" -c BcadKernel.cpp -o "$WORK/BcadKernel.o"
  KERNEL+=("$WORK/BcadKernel.o")
  KSOURCES=(Engine/Fasteners.cpp)
  # One -l per OpenCascade library (libTKernel.dylib; the numbered names are links to it); only those Bcad uses are kept.
  LIBS=(-L"$OCCT/lib" -Xlinker -dead_strip_dylibs)
  for f in "$OCCT"/lib/lib*.dylib; do b=${f:t:r}; [[ $b == *.* ]] || LIBS+=(-l${b#lib}); done
  # The app finds OpenCascade in its own Frameworks folder, the self-test where it was built.
  FLAGS=(-Xlinker -rpath -Xlinker @executable_path/../Frameworks)
  (( SELFTEST )) && FLAGS=(-Xlinker -rpath -Xlinker "$OCCT/lib")
else
  KSOURCES=(Engine/*.cpp)
fi
# No fused multiply-adds: the engine's results then come out bit for bit as on any other machine (its tests run on Linux).
for f in $KSOURCES; do
  "${CXX[@]}" -O3 -ffp-contract=off -c "$f" -o "$WORK/${f:t:r}.o"
  KERNEL+=("$WORK/${f:t:r}.o")
done
SOURCES=(Bcad.swift Design.swift Viewport.swift Views.swift Files.swift)
if (( SELFTEST )); then SOURCES+=(test/SelfTest.swift); FLAGS+=(-D SELFTEST); fi

echo "▸ Compiling"
swiftc -O -swift-version 5 -parse-as-library -sdk "$SDK" -target arm64-apple-macos26.0 $FLAGS \
  -import-objc-header BcadKernel.h $SOURCES $KERNEL $LIBS -lc++ -o "$WORK/Bcad"

if (( SELFTEST )); then
  OUT="${TMPDIR:-/tmp}/bcad-selftest"
  mkdir -p "$OUT"
  cp "$WORK/Bcad" "$OUT/BcadTest"
  echo "▸ Self-test ($OUT)"
  APP_STRINGS="$PWD/i18n.json" "$OUT/BcadTest" --selftest "$OUT"
  exit $?
fi

echo "▸ Rendering icon"
mkdir -p "$WORK/icon.iconset"
# Run before the app is put together: OpenCascade is still where it was built.
DYLD_LIBRARY_PATH="$OCCT/lib" "$WORK/Bcad" --render-icon "$WORK/icon.png"
for s in 16 32 128 256 512; do
  sips -z $s $s "$WORK/icon.png" --out "$WORK/icon.iconset/icon_${s}x${s}.png" >/dev/null
  sips -z $((s*2)) $((s*2)) "$WORK/icon.png" --out "$WORK/icon.iconset/icon_${s}x${s}@2x.png" >/dev/null
done
iconutil -c icns "$WORK/icon.iconset" -o "$WORK/AppIcon.icns"

cat > "$WORK/Info.plist" <<'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key><string>Bcad</string>
  <key>CFBundleIdentifier</key><string>local.bohdan.bcad</string>
  <key>CFBundleName</key><string>Bcad</string>
  <key>CFBundleDisplayName</key><string>Bcad</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundleDevelopmentRegion</key><string>en</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>1.0</string>
  <key>CFBundleVersion</key><string>1</string>
  <key>LSMinimumSystemVersion</key><string>26.0</string>
  <key>LSApplicationCategoryType</key><string>public.app-category.graphics-design</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>NSPrincipalClass</key><string>NSApplication</string>
  <key>NSSupportsAutomaticTermination</key><false/>
  <key>NSSupportsSuddenTermination</key><false/>
  <key>CFBundleDocumentTypes</key>
  <array>
    <dict>
      <key>CFBundleTypeName</key><string>3MF model</string>
      <key>CFBundleTypeRole</key><string>Editor</string>
      <key>LSHandlerRank</key><string>Alternate</string>
      <key>CFBundleTypeExtensions</key><array><string>3mf</string></array>
    </dict>
  </array>
</dict>
</plist>
PLIST
plutil -insert CFBundleLocalizations -json "[$(printf '"%s",' "${LANGS[@]}" | sed 's/,$//')]" "$WORK/Info.plist"
plutil -lint "$WORK/Info.plist" >/dev/null

echo "▸ Localizing"
python3 - "$WORK" "${LANGS[@]}" <<'PY'
import json, os, sys
work = sys.argv[1]
table = json.load(open('i18n.json'))
esc = lambda v: v.replace('\\', '\\\\').replace('"', '\\"')
for lang in sys.argv[2:]:
    folder = os.path.join(work, 'lproj', f'{lang}.lproj')
    os.makedirs(folder, exist_ok=True)
    text = '3MF model' if lang == 'en' else table.get('3MF model', {}).get(lang, '3MF model')
    open(os.path.join(folder, 'InfoPlist.strings'), 'w', encoding='utf-16').write(f'"3MF model" = "{esc(text)}";\n')
PY
for f in "$WORK"/lproj/*.lproj/InfoPlist.strings; do plutil -lint "$f" >/dev/null; done

echo "▸ Updating $TARGET"
if pgrep -xq Bcad; then
  # A normal quit, so Bcad asks about unsaved changes; choosing Cancel there stops the build.
  osascript -e 'tell application id "local.bohdan.bcad" to quit' >/dev/null 2>&1 || true
  for i in {1..20}; do pgrep -xq Bcad || break; sleep 0.5; done
  if pgrep -xq Bcad; then
    echo "✗ Bcad is still open. Quit it and build again."
    exit 1
  fi
fi
rm -rf "$TARGET"
FW="$TARGET/Contents/Frameworks"
mkdir -p "$TARGET/Contents/MacOS" "$TARGET/Contents/Resources"
cp "$WORK/Bcad" "$TARGET/Contents/MacOS/Bcad"
cp "$WORK/Info.plist" "$TARGET/Contents/Info.plist"
cp "$WORK/AppIcon.icns" "$TARGET/Contents/Resources/AppIcon.icns"
cp i18n.json "$TARGET/Contents/Resources/i18n.json"
cp -R "$WORK"/lproj/*.lproj "$TARGET/Contents/Resources/"
if [[ "$ENGINE" == occt ]]; then
  mkdir -p "$FW"
  # OpenCascade's libraries Bcad uses, and the ones those use, as separate files anyone can replace with their own build
  # (LGPL 2.1), with its licence.
  todo=("$TARGET/Contents/MacOS/Bcad")
  while (( $#todo )); do
    for dep in $(otool -L "$todo[1]" | awk 'NR > 1 && $1 ~ /^@rpath\// { print substr($1, 8) }'); do
      [[ -e "$FW/$dep" ]] && continue
      cp -L "$OCCT/lib/$dep" "$FW/$dep"
      todo+=("$FW/$dep")
    done
    shift todo
  done
  if otool -L "$TARGET/Contents/MacOS/Bcad" "$FW"/*.dylib | awk '$1 !~ /:$/ && $1 !~ /^(@rpath\/|\/usr\/lib\/|\/System\/)/' | grep -q .; then
    echo "✗ Bcad would load a library from outside the app and the system:"
    otool -L "$TARGET/Contents/MacOS/Bcad" "$FW"/*.dylib
    exit 1
  fi
  cat "$OCCT"/share/doc/opencascade*/OCCT_LGPL_EXCEPTION.txt "$OCCT"/share/doc/opencascade*/LICENSE_LGPL_21.txt > "$TARGET/Contents/Resources/OpenCASCADE-License.txt"
  codesign --force -s - "$FW"/*.dylib
fi
codesign --force -s - "$TARGET"
/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister -f "$TARGET" 2>/dev/null || true
echo "✓ $TARGET"
