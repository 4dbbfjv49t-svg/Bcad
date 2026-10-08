// Meshes from other apps and from scanners: read from their files (ScanRead.cpp) as they hold them, in millimetres, then
// made into closed bodies (Scan.cpp).
#pragma once
#include "Engine/Math.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace bce {

// The most a file may hold.
struct ScanLimits {
  uint64_t triangles = 50000000, points = 50000000;
};

// A mesh as a file holds it: its points (mm, those one to the last bit made one, numbered as they first come) and its
// triangles, in parts (each part's triangles together: separate bodies, where each is closed; otherwise one).
struct Soup {
  std::vector<float> p;                  // 3 per point
  std::vector<uint32_t> tri;             // 3 per triangle
  std::vector<uint32_t> partStart{0};    // per part, and the end (in triangles)
  std::vector<std::string> partName;     // per part ("" where the file names none)
  double scale = 1;                      // mm per unit of the file
  bool guessed = false;                  // the file says no unit and is under 2 units across: taken as metres
  uint64_t skipped = 0;                  // faces left out (fewer than three corners)
  size_t parts() const { return partName.size(); }
};

// An STL (binary or text), OBJ or PLY file (by its extension, any case): false with `why` ("stl: cut short", "obj: line
// 12: not a number", "ply: has points but no surface", "scan: too large: …" …) where it can't be read.
bool readScan(const uint8_t *bytes, size_t n, const std::string &extension, const ScanLimits &limits, Soup &out, std::string &why);

// A 3MF package's parts (unzipped: the model files and _rels/.rels, by their names in the package), as any app writes them:
// objects and their components, placed as the build places them, in the package's unit. false with `why` ("3mf: …").
struct PackagePart {
  std::string name;
  const uint8_t *bytes;
  size_t n;
};
bool readScan3mf(const std::vector<PackagePart> &parts, const ScanLimits &limits, Soup &out, std::string &why);

// Points made one where they're one to the last bit (−0 as 0), numbered as they first come: a flat table, grown as it
// fills, its slots chosen from all the bits (as weld() does).
class PointWeld {
 public:
  explicit PointWeld(std::vector<float> &p) : p(p) {}
  uint32_t add(float x, float y, float z) {
    float q[3] = {x + 0.0f, y + 0.0f, z + 0.0f};
    if ((size_t)(n + 1) * 2 > table.size()) grow();
    size_t mask = table.size() - 1;
    for (size_t k = slot(q) & mask;; k = (k + 1) & mask) {
      uint32_t at = table[k];
      if (at == UINT32_MAX) {
        table[k] = n;
        p.insert(p.end(), q, q + 3);
        return n++;
      }
      if (std::memcmp(&p[3 * (size_t)at], q, sizeof q) == 0) return at;
    }
  }
  uint32_t size() const { return n; }

 private:
  std::vector<float> &p;
  std::vector<uint32_t> table;
  uint32_t n = 0;
  static uint64_t slot(const float *q) {
    uint32_t b[3];
    std::memcpy(b, q, sizeof b);
    uint64_t h = (b[0] * 0x9E3779B97F4A7C15ull) ^ (b[1] * 0xC2B2AE3D27D4EB4Full) ^ (b[2] * 0x165667B19E3779F9ull);
    h ^= h >> 33, h *= 0xFF51AFD7ED558CCDull, h ^= h >> 33, h *= 0xC4CEB9FE1A85EC53ull, h ^= h >> 33;
    return h;
  }
  void grow() {
    table.assign(std::max<size_t>(64, table.size() * 2), UINT32_MAX);
    size_t mask = table.size() - 1;
    for (uint32_t i = 0; i < n; i++)
      for (size_t k = slot(&p[3 * (size_t)i]) & mask;; k = (k + 1) & mask)
        if (table[k] == UINT32_MAX) {
          table[k] = i;
          break;
        }
  }
};

// How a mesh is made a closed body.
struct ScanOptions {
  uint64_t maxTriangles = 2000000;  // simplified to at most this many (0: kept at any size)
  bool fillHoles = true, removeIslands = true, remake = true;
  double islandShare = 0.01;  // loose open bits of fewer of the triangles than this (and small beside the whole) taken off
  std::function<bool(double)> progress;  // told how far along (0 … 1); false: stop
};
// What was done to it.
struct ScanReport {
  uint64_t trianglesIn = 0, pointsIn = 0, trianglesOut = 0, pointsOut = 0;
  uint64_t welded = 0;      // points made one with another a hair away (where a file's pieces didn't quite meet)
  uint64_t dropped = 0;     // triangles left out: corners not numbers or not there, a corner twice
  uint64_t duplicates = 0;  // triangles there twice (or both ways round: both gone)
  uint64_t flipped = 0;     // triangles turned round to face the way the rest do
  uint64_t crowded = 0;     // edges more than two triangles met at
  uint64_t holes = 0, largestHole = 0;  // holes closed, the most sides one had
  uint64_t islands = 0;     // triangles of loose bits taken off
  int remade = 0;           // 0: kept as it was; 1: a fold mended; 2: parts overlapping made one; 3: made again on a grid;
                            // 4: made solid on a grid (where it couldn't be closed as it was)
  double remadeDetail = 0;  // the grid's spacing then
  uint64_t simplifiedFrom = 0;  // the triangles before it was simplified (0: it wasn't)
  double deviation = 0;     // the furthest the simplified surface strays from the full one
  V3 offset;                // where its middle was (the body is made about the origin: placed back by this)
  V3 size;
  double volume = 0;
  double detail = 1;        // the detail to sculpt it at: about its triangles' size
};

// A mesh from a file (points in mm, each triangle's corners) made into a closed body bk_mesh_shape takes: welded,
// every triangle turned to face out, loose bits taken off, edges more than two triangles meet at paired, holes filled;
// where it then passes through itself, mended, its overlapping parts made one, or made again on a grid. Simplified
// where it's more than options.maxTriangles. Its points are made about the origin (report.offset). False with `why`
// ("scan: larger than 10 m", "scan: can't be closed", "scan: stopped", …) where it can't be.
bool repairScan(const float *pos, size_t points, const uint32_t *tri, size_t triangles, const ScanOptions &options, std::vector<float> &outPos,
                std::vector<uint32_t> &outTri, ScanReport &report, std::string &why);

// A mesh made solid on a grid h apart: inside wherever it wraps round more than halfway (whatever its holes and however
// it passes through itself), the surface where that changes; closed by how it's made (false with `why` where nothing's
// inside, or it would take more than about 1.5 million triangles).
bool solidify(const std::vector<V3> &pts, const std::vector<uint32_t> &tris, double h, std::vector<V3> &outPts, std::vector<uint32_t> &outTris,
              std::string &why);

// A closed mesh simplified to at most `most` triangles: edges collapsed, the one that moves the surface least first
// (Garland and Heckbert's quadrics), never one that would tear it, turn a triangle over or leave one of no area; each
// point moved put on a float. Stays closed (whether it then passes through itself is to be checked). False where
// `going` says to stop.
bool simplify(std::vector<V3> &pts, std::vector<uint32_t> &tris, size_t most, const std::function<bool(double)> &going);
// Points in one cell of a grid `cell` apart made one (at their middle, on a float), triangles left with a corner twice
// gone: a mesh too large to work on, made coarser at once (whatever it leaves broken is mended after).
void cluster(std::vector<V3> &pts, std::vector<uint32_t> &tris, double cell);
// The furthest any of the points `from` lies from a mesh's surface.
double deviation(const std::vector<V3> &from, const std::vector<V3> &pts, const std::vector<uint32_t> &tris);

// A number as text ([±] digits [. digits] [e [±] digits], or nan, inf, infinity), the same on every machine and in every
// locale: false where there's none at s. Moves s past it.
bool readNumber(const char *&s, const char *end, double &out);

}  // namespace bce
