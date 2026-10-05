#!/bin/zsh
# Makes the Xcode project for the App Store build, appstore/Bcad.xcodeproj: the icon as an asset catalog, Info.plist and
# its translations, then the project itself. Needs a Mac with Xcode 26 and XcodeGen (brew install xcodegen).
# Usage: appstore/prepare.sh [a built Bcad binary that isn't sandboxed, to draw the icon]
# (Without one, a personal build is made first.)
set -euo pipefail
cd "$(dirname "$0")"
# Interface languages (as in build.sh).
LANGS=(en uk cs de es fr it hu nl nb pl ro fi sv kk ka ar hi zh-Hans ja)
WORK="$(mktemp -d "${TMPDIR:-/tmp}/bcad-prepare.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT

DRAW="${1:-}"
if [[ -z "$DRAW" ]]; then
  ../build.sh --personal "$WORK/Bcad.app"
  DRAW="$WORK/Bcad.app/Contents/MacOS/Bcad"
fi

echo "▸ Icon"
rm -rf Resources
ICONS=Resources/Assets.xcassets/AppIcon.appiconset
mkdir -p "$ICONS"
"$DRAW" --render-icon "$WORK/icon.png"
[[ -s "$WORK/icon.png" ]] || { echo "✗ The icon wasn't drawn"; exit 1; }
images=()
for s in 16 32 128 256 512; do
  sips -z $s $s "$WORK/icon.png" --out "$ICONS/icon_${s}.png" >/dev/null
  sips -z $((s*2)) $((s*2)) "$WORK/icon.png" --out "$ICONS/icon_${s}@2x.png" >/dev/null
  images+=("{\"idiom\":\"mac\",\"size\":\"${s}x${s}\",\"scale\":\"1x\",\"filename\":\"icon_${s}.png\"}")
  images+=("{\"idiom\":\"mac\",\"size\":\"${s}x${s}\",\"scale\":\"2x\",\"filename\":\"icon_${s}@2x.png\"}")
done
echo "{\"images\":[${(j:,:)images}],\"info\":{\"author\":\"xcode\",\"version\":1}}" > "$ICONS/Contents.json"
echo '{"info":{"author":"xcode","version":1}}' > Resources/Assets.xcassets/Contents.json

echo "▸ Info.plist and its translations"
# The bundle id and version come from the project's settings (Signing & Capabilities, General).
sed -e 's/@BUNDLE_ID@/$(PRODUCT_BUNDLE_IDENTIFIER)/' -e 's/@VERSION@/$(MARKETING_VERSION)/' -e 's/@BUILD@/$(CURRENT_PROJECT_VERSION)/' \
  ../Info.plist.in > Info.plist
plutil -insert CFBundleLocalizations -json "[$(printf '"%s",' "${LANGS[@]}" | sed 's/,$//')]" Info.plist
plutil -lint Info.plist >/dev/null
python3 plist_strings.py Resources "${LANGS[@]}"

echo "▸ Xcode project"
export BCAD_VERSION="$(tr -d '[:space:]' < ../VERSION.txt)" BCAD_BUILD="${BUILD_NUMBER:-1}"
xcodegen generate --spec project.yml
echo "✓ appstore/Bcad.xcodeproj ($BCAD_VERSION, build $BCAD_BUILD)"
