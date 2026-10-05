# Bcad

A small CAD app for macOS for making parts to 3D print. You build shapes from boxes, cylinders, spheres, tori, bolts and
nuts, then merge, cut, split, round, bevel and hollow them. Its geometry engine is its own (`Engine/`, C++).

## Files
- **3MF**: Bcad's own files. They print as they are: each body is an object of its own, named and coloured, a closed
  solid, placed on the bed as in Bcad. The editable model is kept inside too.
- **STL**: all chosen bodies in one file (binary, millimetres).
- **STEP**: each body as a closed solid (AP214). Flat faces are exact; curved ones are 0.01 mm facets.

## Plans
- **Free:** one document a day (saved and exported as often as you like that day), no ads, no human figures.
- **Pro** ($9 a month or $90 a year): unlimited saving and exporting.
- **Studio** ($15 a month or $150 a year): Pro plus human figures.

Each starts with a 7-day free trial (once). Products and prices are in `Bcad.storekit`. Development builds sell through a
test store with no real payment (kept in `~/Library/Application Support/Bcad/test-store.json`; the Plans card has
Expire now, Next day and Reset). The App Store build uses StoreKit. The free plan's file of the day is kept in the
Keychain. Privacy policy: `docs/privacy.md`.

## Build
On macOS 26 with Xcode 26 (installs `/Applications/Bcad.app`, or the `.app` path given):
- `./build.sh`: for development; plans are bought in the test store (no real payment).
- `./build.sh --personal`: your own copy, everything unlocked, no plans (`com.bohdan.bcad.personal`).
- `./build.sh --appstore`: as the Mac App Store has it: sandboxed, hardened runtime, plans through StoreKit
  (`com.bohdan.bcad`). To publish, `appstore/prepare.sh` makes the Xcode project; see `appstore/README.md`.

The version is in `VERSION`; the build number comes from `BUILD_NUMBER` (CI's run number).

## Test
- `./test/engine.sh`: the engine against exact maths (Linux or macOS).
- `./test/corpus.sh`: 456 saved cases; `--perturb` also tries each one moved, turned and rescaled.
- `./test/selftest.sh`: the app's own checks (macOS): kernel, files, editing, mouse.
- `python3 test/printcheck.py DIR`: the self-test's print samples read as slicers read them (needs `lib3mf trimesh numpy`).
- `./test/slicers.sh DIR`: the samples opened in Bambu Studio, PrusaSlicer and Cura's engine.

CI (`.github/workflows/build.yml`) runs all of these. Every push also builds `Bcad.app` and keeps the `print-samples`.
