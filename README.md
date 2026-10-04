# Bcad

A small CAD app for macOS for making parts to 3D print. You build shapes from boxes, cylinders, spheres, tori, bolts and
nuts, then merge, cut, split, round, bevel and hollow them. Its geometry engine is its own (`Engine/`, C++).

## Files
- **3MF**: Bcad's own files. They print as they are: each body is an object of its own, named and coloured, a closed
  solid, placed on the bed as in Bcad. The editable model is kept inside too.
- **STL**: all chosen bodies in one file (binary, millimetres).
- **STEP**: each body as a closed solid (AP214). Flat faces are exact; curved ones are 0.01 mm facets.

## Build
On macOS 26 with Xcode 26: `./build.sh` (installs `/Applications/Bcad.app`), or `./build.sh path/to/Bcad.app`.

## Test
- `./test/engine.sh`: the engine against exact maths (Linux or macOS).
- `./test/corpus.sh`: 416 saved cases; `--perturb` also tries each one moved, turned and rescaled.
- `./test/selftest.sh`: the app's own checks (macOS): kernel, files, editing, mouse.
- `python3 test/printcheck.py DIR`: the self-test's print samples read as slicers read them (needs `lib3mf trimesh numpy`).
- `./test/slicers.sh DIR`: the samples opened in Bambu Studio, PrusaSlicer and Cura's engine.

CI (`.github/workflows/build.yml`) runs all of these. Every push also builds `Bcad.app` and keeps the `print-samples`.
