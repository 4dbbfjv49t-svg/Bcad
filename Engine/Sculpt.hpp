// Sculpted bodies (Sculpt.cpp): a closed mesh made again of even triangles at a chosen detail, ready to be shaped by hand.
#pragma once
#include "Engine/Model.hpp"

#include <string>
#include <vector>

namespace bce {

// What a closed mesh encloses (wherever its winding number is above 0, so overlapping pieces and folds come out as one
// solid, and voids stay), made again as one closed mesh of triangles about `detail` apart: sampled on a grid that fine,
// its surface put where the grid's lines cross the mesh's, then evened out. The mesh needn't share its points (any
// shape's mesh will do). False with `why` when nothing's inside, or when it would take more than about `most` triangles.
bool remesh(const std::vector<V3> &pts, const std::vector<uint32_t> &tris, double detail, std::vector<V3> &outPts, std::vector<uint32_t> &outTris,
            std::string &why, size_t most = 1500000);

// A body being sculpted: its mesh (points shared, closed) shaped by strokes of a brush, each stroke undone and done again
// as one. Brushes (as BK_BRUSH_…): grab (the points under it at the start carried along by the drag), draw (raised along
// the surface's average way out there; carved when inverted), inflate (each point along its own normal), smooth (each
// point toward its neighbours' middle), flatten (onto the plane through the points under it), pinch (drawn in toward the
// brush's middle along the surface), crease (carved and pinched: a sharp groove; a ridge when inverted). The brush
// weighs points by how near its middle they are (none at its radius); `mirror` does the same across x = 0 as well.
// Every query and stroke comes out the same whatever order things are visited in.
class Sculptor {
 public:
  Sculptor(std::vector<V3> pts, std::vector<uint32_t> tris);

  // Where a ray first meets the surface (its point, and the surface's normal there): false if it misses.
  bool ray(V3 origin, V3 dir, V3 &at, V3 &normal) const;
  // A stroke: begun at a point (for grab, where the drag starts), dabs along its way (for grab, where the drag has got
  // to), ended. Pressure scales the strength of each dab (1 for a mouse).
  enum Brush { Grab, Draw, Inflate, Smooth, Flatten, Pinch, Crease };
  void begin(int brush, V3 at, double radius, double strength, bool mirror, bool invert);
  void dab(V3 at, double pressure);
  void end();
  bool undo();
  bool redo();
  // The points moved or turned since this was last asked, in order (and forgotten).
  std::vector<uint32_t> takeChanged();

  const std::vector<V3> &points() const { return p; }
  const std::vector<V3> &normals() const { return n; }
  const std::vector<uint32_t> &triangles() const { return tri; }

 private:
  struct TreeNode {
    V3 lo, hi;
    int left = -1, right = -1, first = 0, count = 0;
  };
  struct Step {
    std::vector<uint32_t> idx;
    std::vector<V3> before, after;
  };

  std::vector<V3> p, n;
  std::vector<uint32_t> tri;
  std::vector<uint32_t> vtStart, vtList, nbStart, nbList;  // per point: its triangles; its neighbours
  std::vector<TreeNode> nodes;  // a box tree over the triangles (a node's children always come after it)
  std::vector<uint32_t> order, leafOf;
  std::vector<int> parentOf;
  std::vector<uint8_t> dirtyNode;
  mutable std::vector<uint32_t> seen;
  mutable uint32_t seenStamp = 0;

  // The stroke under way.
  bool stroking = false;
  int brush = 0;
  double radius = 1, strength = 0.5;
  bool mirror = false, invert = false;
  V3 start{0, 0, 0}, last{0, 0, 0};
  bool dabbed = false;
  std::vector<uint32_t> grabbed;      // grab: the points under it at the start (or under its mirror), in order,
  std::vector<double> grabWeight[2];  // how much each follows the drag, and the drag mirrored,
  std::vector<V3> grabFrom;           // and where each was
  std::vector<uint32_t> touchedAt;    // per point: the stroke that last recorded it
  uint32_t strokeId = 0;
  Step step;
  std::vector<Step> undone, done;
  size_t kept = 0;  // points held by `done` and `undone`

  std::vector<uint8_t> changedFlag;
  std::vector<uint32_t> changed;
  std::vector<uint32_t> dirtyList;
  // (Scratch, kept between dabs.)
  std::vector<uint32_t> idx[2], sum, around;
  std::vector<double> wt[2];
  std::vector<V3> off[2], offSum;

  int build(int first, int count, const std::vector<V3> &mid, int parent);
  void boxOf(uint32_t t, V3 &lo, V3 &hi) const;
  V3 normalAt(uint32_t i) const;
  // The points within `r` of c, in order, and each one's weight (1 at the middle, falling smoothly to 0 at r).
  void within(V3 c, double r, std::vector<uint32_t> &out, std::vector<double> &weight) const;
  // How a dab at c would move the points under it (from where they are now).
  void offsets(V3 c, double pressure, std::vector<uint32_t> &which, std::vector<double> &weight, std::vector<V3> &by) const;
  void dabAt(V3 c, double pressure);
  void record(uint32_t i);
  // After points moved: their triangles' boxes, their own and their neighbours' normals, and what's changed.
  void moved(const std::vector<uint32_t> &which);
  void restore(const Step &s, bool back);
  void keep(Step s);
};

}  // namespace bce
