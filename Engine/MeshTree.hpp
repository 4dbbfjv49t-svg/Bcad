// A tree of boxes over a mesh's triangles (MeshTree.cpp): the nearest point of the surface to any point, and the winding
// number there (how many times the surface wraps round it: 1 inside a closed solid, 0 outside, in between near a hole),
// quickly for millions of triangles, the same on every machine.
#pragma once
#include "Engine/Math.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace bce {

class MeshTree {
 public:
  // Over the mesh as given (its points all numbers), which must outlive the tree.
  MeshTree(const std::vector<V3> &pts, const std::vector<uint32_t> &tris);

  struct Near {
    double d2 = INFINITY;  // the distance, squared
    V3 at;
    uint32_t tri = UINT32_MAX;
  };
  // The nearest point of the surface to q (the lowest-numbered triangle's where two are as near), looked for only nearer
  // than `within2` (squared).
  Near nearest(V3 q, double within2 = INFINITY) const;
  // The winding number at q: each triangle near it exactly, each group of them far off as one (within about 0.05: what
  // telling inside from outside needs).
  double winding(V3 q) const;
  // The same, every triangle exactly (for tests).
  double windingExactly(V3 q) const;

 private:
  struct Node {
    V3 lo, hi;
    uint32_t first = 0, count = 0;  // a leaf: its triangles in `order`
    uint32_t left = 0, right = 0;   // otherwise its two halves
    V3 area;                        // its triangles' areas as vectors (out from each), summed
    double mass = 0;                // their sizes summed
    V3 centre;                      // their middle, by size
    double radius = 0;              // the furthest its box reaches from there
  };
  const std::vector<V3> &P;
  const std::vector<uint32_t> &T;
  std::vector<uint32_t> order;
  std::vector<Node> nodes;
  uint32_t build(uint32_t first, uint32_t count, std::vector<V3> &mids);
  double solidAngle(uint32_t t, V3 q) const;
};

}  // namespace bce
