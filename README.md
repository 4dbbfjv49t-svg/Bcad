# Bcad

A small CAD app for macOS for making parts to 3D print. You build shapes from boxes, cylinders, spheres, tori, bolts and
nuts, then merge, cut, split, round, bevel and hollow them. Its geometry engine is its own (`Engine/`, C++).

## Files
- **3MF**: Bcad's own files. They print as they are: each body is an object of its own, named and coloured, a closed
  solid, placed on the bed as in Bcad. The editable model is kept inside too.
- **STL**: all chosen bodies in one file (binary, millimetres).
- **STEP**: each body as a closed solid (AP214). Flat faces are exact; curved ones are 0.01 mm facets.

### Importing scans and other apps' models
File → Import… (⌘⇧I), Open, or drop a file on the window: **STL** (binary or text), **OBJ**, **PLY**, another app's
**3MF**, and **USDZ** (what Apple's Object Capture makes). Each becomes a closed body, ready to sculpt, merge, cut and
print:
- Points that should be one are welded, triangles are turned to face out, loose bits of scan noise are taken off, and
  holes are filled. The mesh is kept at full resolution where it can be.
- Where the surface passes through itself, a small fold is mended in place and overlapping parts are joined. Otherwise
  it is made again on a fine grid.
- Over 2 million triangles, it is simplified. The note says how far the result strays (usually a few micrometres).
- A file with no unit that is under 2 units across is read as metres.

## Sketches
Press **Sketch** (K), then click the bed or a flat face. The view turns to face it; a face's edges are there to draw
and measure to.
- **Draw:** Line (L, one after another), Rectangle (R), Circle (C), Arc (A, through 3 points), Construction (X: lines
  that bound nothing). Clicks snap to points, middles, curves and the grid (⌘: free) and add the matching constraint.
- **Constrain:** choose curves or points and pick a constraint: coincident, on the curve, horizontal, vertical,
  parallel, perpendicular, tangent, equal, concentric, midpoint, collinear, symmetric, fix.
- **Dimension (D):** click a line, a circle, or two items, then where the value goes, and type it. Double-click a value
  to change it. Free geometry is in the accent colour, fully constrained geometry in ink; the bar counts what's left.
- **Extrude (E) / Revolve (V):** click the regions (a lone one is chosen already), set the distance or the angle and
  axis, then New body, Merge, Subtract or Intersect. Enter makes it, as one undo step.
- Results are exact like the built-in shapes (flat faces, true cylinders and turned faces), so rounding, hollowing,
  measuring, STEP and printing work on them. The layer row changes the distance and opens the sketch again.

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

The version is in `VERSION.txt` (not `VERSION`: that name would stand in for the C++ `<version>` header on a Mac); the build number comes from `BUILD_NUMBER` (CI's run number).

## Test
- `./test/engine.sh`: the engine against exact maths (Linux or macOS).
- `./test/corpus.sh`: 497 saved cases; `--perturb` also tries each one moved, turned and rescaled.
- `./test/selftest.sh`: the app's own checks (macOS): kernel, files, editing, mouse.
- `python3 test/printcheck.py DIR`: the self-test's print samples read as slicers read them (needs `lib3mf trimesh numpy`).
- `./test/slicers.sh DIR`: the samples opened in Bambu Studio, PrusaSlicer and Cura's engine.

CI (`.github/workflows/build.yml`) runs all of these. Every push also builds `Bcad.app` and keeps the `print-samples`.
