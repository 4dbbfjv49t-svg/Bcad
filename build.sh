#!/bin/zsh
# Builds /Applications/Bcad.app (or the given .app). ./build.sh --selftest builds and runs the kernel/file self-test instead.
set -euo pipefail

# Interface languages, in the order of the in-app menu (English first).
LANGS=(en uk cs de es fr it hu nl nb pl ro fi sv kk ka ar hi zh-Hans ja)

cd "$(dirname "$0")"
SELFTEST=0
if [[ "${1:-}" == "--selftest" ]]; then SELFTEST=1; shift; fi
TARGET="${1:-/Applications/Bcad.app}"
if [[ "$TARGET" != *.app ]]; then
  echo "✗ Target must end with .app"
  exit 1
fi
HOLD="$(dirname "$TARGET")/.bcad-library-hold"
if [[ -e "$HOLD" && -d "$TARGET/Contents/Library" ]]; then
  echo "✗ $HOLD exists from an interrupted build and the app also has a library. Resolve by hand."
  exit 1
fi

./occt.sh
OCCT="$PWD/Vendor/occt"
SDK="$(ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX26*.sdk | tail -1)"

WORK="$(mktemp -d "${TMPDIR:-/tmp}/bcad.XXXXXX")"
restore() {
  if [[ -d "$HOLD" && ! -e "$TARGET/Contents/Library" ]]; then
    mkdir -p "$TARGET/Contents"
    mv "$HOLD" "$TARGET/Contents/Library"
  fi
  rm -rf "$WORK"
}
trap restore EXIT

echo "▸ Compiling the geometry kernel"
clang++ -O2 -std=c++17 -isysroot "$SDK" -target arm64-apple-macos26.0 -w -I "$OCCT/include/opencascade" -c BcadKernel.cpp -o "$WORK/BcadKernel.o"

LIBS=()
for f in "$OCCT"/lib/*.a; do LIBS+=(-Xlinker "$f"); done
SOURCES=(Bcad.swift Design.swift Viewport.swift Views.swift Files.swift)
FLAGS=()
if (( SELFTEST )); then SOURCES+=(test/SelfTest.swift); FLAGS+=(-D SELFTEST); fi

echo "▸ Compiling"
swiftc -O -swift-version 5 -parse-as-library -sdk "$SDK" -target arm64-apple-macos26.0 $FLAGS \
  -import-objc-header BcadKernel.h $SOURCES "$WORK/BcadKernel.o" $LIBS -lc++ -o "$WORK/Bcad"

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
"$WORK/Bcad" --render-icon "$WORK/icon.png"
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
  pkill -TERM -x Bcad || true
  for i in {1..60}; do pgrep -xq Bcad || break; sleep 0.2; done
  pkill -KILL -x Bcad 2>/dev/null || true
fi
if [[ -d "$TARGET/Contents/Library" ]]; then
  mv "$TARGET/Contents/Library" "$HOLD"
fi
rm -rf "$TARGET"
mkdir -p "$TARGET/Contents/MacOS" "$TARGET/Contents/Resources"
cp "$WORK/Bcad" "$TARGET/Contents/MacOS/Bcad"
cp "$WORK/Info.plist" "$TARGET/Contents/Info.plist"
cp "$WORK/AppIcon.icns" "$TARGET/Contents/Resources/AppIcon.icns"
cp i18n.json "$TARGET/Contents/Resources/i18n.json"
cp -R "$WORK"/lproj/*.lproj "$TARGET/Contents/Resources/"
codesign --force -s - "$TARGET" 2>/dev/null
restore
trap - EXIT
/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister -f "$TARGET" 2>/dev/null || true
echo "✓ $TARGET"
