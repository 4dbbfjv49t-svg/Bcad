#!/bin/zsh
# Builds /Applications/Bcad.app (or the given .app) on Bcad's own geometry engine (Engine/):
#   ./build.sh             for development (plans bought in a test store with no real payment)
#   ./build.sh --personal  your own copy: everything unlocked, no plans
#   ./build.sh --appstore  as the Mac App Store has it: sandboxed, hardened runtime, plans through StoreKit (sign and
#                          upload it from the Xcode project, see appstore/README.md)
#   ./build.sh --selftest  builds and runs the kernel/file self-test instead
set -euo pipefail

# Interface languages, in the order of the in-app menu (English first).
LANGS=(en uk cs de es fr it hu nl nb pl ro fi sv kk ka ar hi zh-Hans ja)

cd "$(dirname "$0")"
SELFTEST=0
MODE=dev
while [[ "${1:-}" == --* ]]; do
  case "$1" in
    --selftest) SELFTEST=1 ;;
    --personal) MODE=personal ;;
    --appstore) MODE=appstore ;;
    *) echo "✗ Unknown option $1"; exit 1 ;;
  esac
  shift
done
VERSION="$(tr -d '[:space:]' < VERSION.txt)"
BUILD="${BUILD_NUMBER:-1}"
case $MODE in
  personal) BUNDLE_ID=com.bohdan.bcad.personal ;;
  appstore) BUNDLE_ID=com.bohdan.bcad ;;
  *) BUNDLE_ID=local.bohdan.bcad ;;
esac
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

SDK="$(xcrun --sdk macosx --show-sdk-path)"

WORK="$(mktemp -d "${TMPDIR:-/tmp}/bcad.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT

echo "▸ Compiling the geometry engine"
CXX=(clang++ -std=c++17 -isysroot "$SDK" -target arm64-apple-macos26.0 -I"$PWD")
KERNEL=()
FLAGS=()
# No fused multiply-adds: the engine's results then come out bit for bit as on any other machine (its tests run on Linux).
for f in Engine/*.cpp; do
  "${CXX[@]}" -O3 -ffp-contract=off -c "$f" -o "$WORK/${f:t:r}.o"
  KERNEL+=("$WORK/${f:t:r}.o")
done
SOURCES=(Bcad.swift Design.swift Viewport.swift Views.swift Files.swift Sculpt.swift Plans.swift Import.swift Sketch.swift Sketching.swift)
if (( SELFTEST )); then
  SOURCES+=(test/SelfTest.swift)
  FLAGS+=(-D SELFTEST)
elif [[ $MODE == personal ]]; then
  FLAGS+=(-D UNLOCKED)
elif [[ $MODE == appstore ]]; then
  FLAGS+=(-D APPSTORE)
fi

echo "▸ Compiling"
swiftc -O -swift-version 5 -parse-as-library -sdk "$SDK" -target arm64-apple-macos26.0 $FLAGS \
  -import-objc-header BcadKernel.h $SOURCES $KERNEL -lc++ -o "$WORK/Bcad"

if (( SELFTEST )); then
  OUT="${TMPDIR:-/tmp}/bcad-selftest"
  mkdir -p "$OUT"
  cp "$WORK/Bcad" "$OUT/BcadTest"
  echo "▸ Self-test ($OUT)"
  APP_STRINGS="$PWD/i18n.json" APP_STOREKIT="$PWD/Bcad.storekit" "$OUT/BcadTest" --selftest "$OUT"
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

sed -e "s/@BUNDLE_ID@/$BUNDLE_ID/" -e "s/@VERSION@/$VERSION/" -e "s/@BUILD@/$BUILD/" Info.plist.in > "$WORK/Info.plist"
plutil -insert CFBundleLocalizations -json "[$(printf '"%s",' "${LANGS[@]}" | sed 's/,$//')]" "$WORK/Info.plist"
plutil -lint "$WORK/Info.plist" >/dev/null

echo "▸ Localizing"
python3 appstore/plist_strings.py "$WORK/lproj" "${LANGS[@]}"
for f in "$WORK"/lproj/*.lproj/InfoPlist.strings; do plutil -lint "$f" >/dev/null; done

echo "▸ Updating $TARGET ($MODE, $BUNDLE_ID $VERSION ($BUILD))"
running() { pgrep -xq Bcad && [[ "$(osascript -e "application id \"$BUNDLE_ID\" is running" 2>/dev/null)" == true ]]; }
if running; then
  # A normal quit, so Bcad asks about unsaved changes; choosing Cancel there stops the build.
  osascript -e "tell application id \"$BUNDLE_ID\" to quit" >/dev/null 2>&1 || true
  for i in {1..20}; do running || break; sleep 0.5; done
  if running; then
    echo "✗ Bcad is still open. Quit it and build again."
    exit 1
  fi
fi
rm -rf "$TARGET"
mkdir -p "$TARGET/Contents/MacOS" "$TARGET/Contents/Resources"
cp "$WORK/Bcad" "$TARGET/Contents/MacOS/Bcad"
cp "$WORK/Info.plist" "$TARGET/Contents/Info.plist"
cp "$WORK/AppIcon.icns" "$TARGET/Contents/Resources/AppIcon.icns"
cp i18n.json "$TARGET/Contents/Resources/i18n.json"
case $MODE in
  # The test store's products and prices (development builds sell nothing for real).
  dev) cp Bcad.storekit "$TARGET/Contents/Resources/Bcad.storekit" ;;
  # What the app does with data, as the App Store asks.
  appstore) cp appstore/PrivacyInfo.xcprivacy "$TARGET/Contents/Resources/PrivacyInfo.xcprivacy" ;;
esac
cp -R "$WORK"/lproj/*.lproj "$TARGET/Contents/Resources/"
# Nothing but the system's own libraries.
if otool -L "$TARGET/Contents/MacOS/Bcad" | awk 'NR > 1 && $1 !~ /^(\/usr\/lib\/|\/System\/)/' | grep -q .; then
  echo "✗ Bcad would load a library from outside the system:"
  otool -L "$TARGET/Contents/MacOS/Bcad"
  exit 1
fi
if [[ $MODE == appstore ]]; then
  # Sandboxed with the hardened runtime, signed for this Mac only: the App Store copy is signed from Xcode with your account.
  codesign --force --options runtime --entitlements appstore/Bcad.entitlements -s - "$TARGET"
else
  codesign --force -s - "$TARGET"
fi
/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister -f "$TARGET" 2>/dev/null || true
echo "✓ $TARGET"
