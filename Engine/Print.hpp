// Closed meshes as files hold them (Print.cpp): a mesh taken apart into its shells (for STEP), and a body as printers take
// it (for 3MF and STL).
#pragma once
#include "Engine/Model.hpp"
#include "Engine/Trig.hpp"

#include <algorithm>
#include <cmath>
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

// The sides along one line from u to v (`fwd` running from u to v, `back` the other way) paired so each pair bounds one
// wedge of material: in the turning order round the line, material lies just past a side running back along it and just
// short of one running forward. across(side) is the corner facing the side. A lone pair is paired at once; where the
// sides don't alternate round the line (never so for a closed solid), they're paired in the order given, and false.
template <class Across>
bool pairAround(V3 u, V3 v, const std::vector<uint32_t> &fwd, const std::vector<uint32_t> &back, Across across,
                std::vector<std::pair<uint32_t, uint32_t>> &pairs) {
  if (fwd.size() == 1 && back.size() == 1) {
    pairs.push_back({fwd[0], back[0]});
    return true;
  }
  V3 d = unit(v - u);
  V3 e1 = unit(std::fabs(d.x) < 0.9 ? cross(d, V3{1, 0, 0}) : cross(d, V3{0, 1, 0})), e2 = cross(d, e1);
  struct Round {
    double angle;
    uint32_t side;
    bool fwd;
  };
  std::vector<Round> round;
  for (const std::vector<uint32_t> *group : {&fwd, &back})
    for (uint32_t side : *group) {
      V3 w = across(side) - u;
      w = w - d * dot(w, d);
      round.push_back({trig::atan2(dot(w, e2), dot(w, e1)), side, group == &fwd});
    }
  std::sort(round.begin(), round.end(), [](const Round &a, const Round &b) { return a.angle != b.angle ? a.angle < b.angle : a.side < b.side; });
  size_t n = round.size(), first = 0;
  while (first < n && round[first].fwd) first++;
  bool alternate = n % 2 == 0 && first < n;
  for (size_t k = 0; k < n && alternate; k++) alternate = round[(first + k) % n].fwd == (k % 2 == 1);
  if (alternate) {
    for (size_t k = 0; k < n; k += 2) pairs.push_back({round[(first + k + 1) % n].side, round[(first + k) % n].side});
    return true;
  }
  for (size_t k = 0; k < std::min(fwd.size(), back.size()); k++) pairs.push_back({fwd[k], back[k]});
  return false;
}

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
