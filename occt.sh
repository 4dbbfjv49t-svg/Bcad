#!/bin/zsh
# Builds a static OpenCascade (modeling + STEP) into Vendor/occt once. Called by build.sh; safe to rerun.
# Tools are downloaded into Vendor/ (portable CMake), nothing is installed on the Mac.
set -euo pipefail
cd "$(dirname "$0")"
VENDOR="$PWD/Vendor"
CMAKE_VERSION="4.4.3"
OCCT_TAG="V7_9_3"
OUT="$VENDOR/occt"
[[ -f "$OUT/.done-$OCCT_TAG" ]] && exit 0

fetch() {
  [[ -s "$2" ]] && return
  echo "▸ Downloading $(basename "$2")"
  curl -fL --retry 3 -o "$2.part" "$1"
  mv "$2.part" "$2"
}
fetch "https://github.com/Kitware/CMake/releases/download/v$CMAKE_VERSION/cmake-$CMAKE_VERSION-macos-universal.tar.gz" "$VENDOR/cmake-$CMAKE_VERSION.tar.gz"
fetch "https://github.com/Open-Cascade-SAS/OCCT/archive/refs/tags/$OCCT_TAG.tar.gz" "$VENDOR/occt-$OCCT_TAG.tar.gz"

CMAKE_DIR="$VENDOR/cmake-$CMAKE_VERSION"
if [[ ! -x "$CMAKE_DIR/CMake.app/Contents/bin/cmake" ]]; then
  mkdir -p "$CMAKE_DIR"
  tar -xzf "$VENDOR/cmake-$CMAKE_VERSION.tar.gz" -C "$CMAKE_DIR" --strip-components 1
  xattr -cr "$CMAKE_DIR" 2>/dev/null || true
fi
CMAKE="$CMAKE_DIR/CMake.app/Contents/bin/cmake"

SRC="$VENDOR/occt-src"
if [[ ! -f "$SRC/CMakeLists.txt" ]]; then
  rm -rf "$SRC"; mkdir -p "$SRC"
  tar -xzf "$VENDOR/occt-$OCCT_TAG.tar.gz" -C "$SRC" --strip-components 1
fi

SDK="$(ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX26*.sdk | tail -1)"
BUILD="$VENDOR/occt-build"
echo "▸ Building OpenCascade $OCCT_TAG (static; first time only)"
"$CMAKE" -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DBUILD_LIBRARY_TYPE=Static \
  -DCMAKE_INSTALL_PREFIX="$OUT" -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=26.0 -DCMAKE_OSX_SYSROOT="$SDK" \
  -DBUILD_MODULE_Draw=OFF -DBUILD_DOC_Overview=OFF -DBUILD_SAMPLES_QT=OFF \
  -DUSE_FREETYPE=OFF -DUSE_FREEIMAGE=OFF -DUSE_OPENVR=OFF -DUSE_FFMPEG=OFF -DUSE_TBB=OFF -DUSE_VTK=OFF \
  -DUSE_RAPIDJSON=OFF -DUSE_DRACO=OFF -DUSE_TK=OFF -DUSE_OPENGL=OFF -DUSE_GLES2=OFF -DUSE_D3D=OFF >"$VENDOR/occt-cmake.log" 2>&1 \
  || { tail -30 "$VENDOR/occt-cmake.log"; exit 1; }
"$CMAKE" --build "$BUILD" -j "$(sysctl -n hw.ncpu)" >"$VENDOR/occt-build.log" 2>&1 || { grep -m5 -B2 -A5 "error" "$VENDOR/occt-build.log"; exit 1; }
"$CMAKE" --install "$BUILD" >/dev/null
touch "$OUT/.done-$OCCT_TAG"
echo "✓ OpenCascade in $OUT"
