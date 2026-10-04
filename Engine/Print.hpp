// Closed meshes as files hold them (Print.cpp): a mesh taken apart into its shells (for STEP), and a body as printers take
// it (for 3MF and STL).
#pragma once
#include "Engine/Model.hpp"

#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace bce {

// The chord error files are written at (STEP, 3MF, STL).
constexpr double fileDeflection = 0.01;

struct Find {
  std::vector<uint32_t> up;
  explicit Find(size_t n) : up(n) { std::iota(up.begin(), up.end(), 0); }
  uint32_t operator()(uint32_t x) {
    while (up[x] != x) x = up[x] = up[up[x]];
    return x;
  }
  void join(uint32_t a, uint32_t b) {
    a = (*this)(a), b = (*this)(b);
    if (a != b) up[std::max(a, b)] = std::min(a, b);
  }
};

// A closed mesh as its shells: points one where they're one to the last bit; each side of each triangle (side 3t + k runs
// from its corner k to the next) met by one running back along it that bounds the same wedge of material (where parts
// touch along a line, more than two triangles meet there); the shells those joins make, and the solid each void is in.
struct Shells {
  std::vector<V3> P;
  std::vector<uint32_t> T;                           // 3 per triangle, into P
  std::vector<uint32_t> mate;                        // per side: the side it meets
  std::vector<uint32_t> edgeOf;                      // per side: its edge
  std::vector<std::pair<uint32_t, uint32_t>> edges;  // each edge's ends (lower point number first)
  std::vector<int> shellAt;                          // per triangle
  std::vector<double> vol;                           // per shell: its volume (less than nothing: a void)
  std::vector<int> voidOf;                           // per shell: a void's solid, -1 for a solid
  double size = 1;                                   // the largest coordinate (at least 1)
  uint32_t from(uint32_t side) const { return T[side]; }
  uint32_t to(uint32_t side) const { return T[side / 3 * 3 + (side % 3 + 1) % 3]; }
  uint32_t across(uint32_t side) const { return T[side / 3 * 3 + (side % 3 + 2) % 3]; }
};

// false with `why` ("empty", "open", "inside out") where the mesh isn't a closed solid.
bool shells(const Solid &s, Shells &out, std::string &why);

// A body as printers take it: points as files hold them (float), every edge between exactly two triangles (where parts
// touch along a line or at a point, each part has its own copy of the points there).
struct PrintMesh {
  std::vector<float> pos;  // 3 per point
  std::vector<uint32_t> tri;
  double volume = 0;       // of the triangles as given
  int crowded = 0;         // edges still between more than two triangles (a part touching itself round a point there)
  int slivers = 0;         // triangles left thinner than float holds (flat: corners all but in a line)
};

// The shape's mesh at fileDeflection as printers take it: false with `why` ("empty", "open", "inside out", or "thin": a
// triangle that float flattens or turns over, or two points float makes one) where it isn't a sound solid, the triangles
// still in `out`.
bool printMesh(const Shape &shape, PrintMesh &out, std::string &why);

}  // namespace bce
