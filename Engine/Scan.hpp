// Meshes from other apps and from scanners: read from their files (ScanRead.cpp) as they hold them, in millimetres.
#pragma once
#include <cstddef>
#include <cstdint>
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

// A number as text ([±] digits [. digits] [e [±] digits], or nan, inf, infinity), the same on every machine and in every
// locale: false where there's none at s. Moves s past it.
bool readNumber(const char *&s, const char *end, double &out);

}  // namespace bce
