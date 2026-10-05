// Sculpted bodies (Sculpt.cpp): a closed mesh made again of even triangles at a chosen detail, ready to be shaped by hand.
#pragma once
#include "Engine/Model.hpp"

#include <cmath>
#include <functional>
#include <queue>
#include <string>
#include <vector>

namespace bce {

// What a closed mesh encloses (wherever its winding number is above 0, so overlapping pieces and folds come out as one
// solid, and voids stay), made again as one closed mesh of triangles about `detail` apart: sampled on a grid that fine,
// its surface put where the grid's lines cross the mesh's, then evened out. The mesh needn't share its points (any
// shape's mesh will do). False with `why` when nothing's inside, or when it would take more than about `most` triangles.
bool remesh(const std::vector<V3> &pts, const std::vector<uint32_t> &tris, double detail, std::vector<V3> &outPts, std::vector<uint32_t> &outTris,
            std::string &why, size_t most = 1500000);

// Whether a mesh passes through itself: two of its triangles that share no corner crossing (touching doesn't count).
bool selfCrossing(const std::vector<V3> &pts, const std::vector<uint32_t> &tris);

// The void that hollows a closed mesh with walls `t` thick: every point inside it further than t from its surface, as a
// closed mesh facing into the void (a part of the body thinner than twice the walls stays solid). Found on a grid half
// the walls apart, so its surface is as true as that. False with `why` when the walls would fill the body.
bool hollowByGrid(const std::vector<V3> &pts, const std::vector<uint32_t> &tris, double t, std::vector<V3> &outPts,
                  std::vector<uint32_t> &outTris, std::string &why);

// The surface where field(p) crosses 0 (inside where it's below 0), as one closed mesh of triangles facing out: marching
// cubes on a grid of n points along each axis, h apart, centred on `mid` (the same either side of it to the bit; its
// outermost points must all be outside). Only blocks of 4 steps a side that block(lo, hi) can't prove all inside (−1) or
// all outside (+1) are worked out point by point. Where the surface crosses a grid edge is by a straight line between
// its ends' values; the points are then evened out `passes` times. False with `why` when nothing's inside or it would
// take more than about `most` triangles. With k above 1, the cells fine(lo, hi) asks for (among those not proved inside
// or outside) are worked out k times finer each way; the cells beside them meet their finer surface without a gap.
// With `uncrossed`, wherever evening out made the surface pass through itself, those points are put back (all of them at
// last), and it's told whether the surface ends up not passing through itself.
bool isoSurface(V3 mid, double h, const int n[3], const std::function<double(V3)> &field, const std::function<int(V3, V3)> &block, int passes,
                std::vector<V3> &outPts, std::vector<uint32_t> &outTris, std::string &why, size_t most = 1500000, int k = 1,
                const std::function<bool(V3, V3)> &fine = nullptr, bool *uncrossed = nullptr);

// A body being sculpted: its mesh (points shared, closed) shaped by strokes of a brush, each stroke undone and done again
// as one. Brushes (as BK_BRUSH_…): grab (the points under it at the start carried along by the drag), draw (raised along
// the surface's average way out there; carved when inverted), inflate (each point along its own normal), smooth (each
// point toward its neighbours' middle), flatten (onto the plane through the points under it), pinch (drawn in toward the
// brush's middle along the surface), crease (carved and pinched: a sharp groove; a ridge when inverted), detail (the
// triangles only, made the detail size: the surface stays where it is), clay (raised up to a plane a little above the
// points under it, never past it), layer (raised to one height for the whole stroke, however often it passes),
// blob (pushed out from a point under the brush: round blobs), scrape (what stands above the plane through the points
// under it cut down to it; inverted, fill: what lies below it filled up), smudge (carried along the stroke's way),
// snake hook (pulled along with the pointer, the points under it taken again at every step: horns and tentacles),
// twist (turned round the brush's axis). The brush weighs points by how near its middle they are (none at its rim):
// its tip (Tip) sets how (a core at full strength, the fade soft or crisp, the footprint an oval at an angle to the
// stroke's way) and leans its push. `mirror` (bits: x, y, z) does the same across those planes through the body's
// origin as well, in every combination of them.
//
// With a detail size set, every brush also makes the triangles it passes over that size: sides much longer than it asks
// are halved, sides much shorter merged away where that keeps the surface sound, and sides between thin triangles turned
// to make them rounder (grab does this when its drag ends, holding on to the points it took until then). So the mesh's triangles and points come and go: they're kept in
// slots, a slot freed when its triangle or point goes and used again later.
//
// Every query and stroke comes out the same whatever order things are visited in.
// A brush's tip: hardness, the part of its radius at full strength (0…1); rigidity, how crisp its fade from there to the
// rim (0: soft and rounded, 1: a straight slope); oval, how wide its footprint is across as along (0.05…1), and angle,
// how far that's turned from the stroke's way (degrees); tilt, how far its push leans from straight out toward the
// stroke's way (degrees, ±85). As it is (all 0, oval 1), a brush weighs points as it always has.
struct BrushTip {
  double hardness = 0, rigidity = 0, oval = 1, angle = 0, tilt = 0;
  bool plain() const { return hardness == 0 && rigidity == 0 && oval == 1; }
};

class Sculptor {
 public:
  Sculptor(std::vector<V3> pts, std::vector<uint32_t> tris);
  // Whether the mesh was one it can hold: closed, each side run once each way (a point where two fans of triangles meet
  // is made two points).
  bool ok() const { return good; }

  // Where a ray first meets the surface (its point, and the surface's normal there): false if it misses.
  bool ray(V3 origin, V3 dir, V3 &at, V3 &normal) const;
  // A stroke: begun at a point (for grab, where the drag starts), dabs along its way (for grab, where the drag has got
  // to), ended. Pressure scales the strength of each dab and size its radius (both 1 for a mouse; grab takes neither).
  enum Brush { Grab, Draw, Inflate, Smooth, Flatten, Pinch, Crease, Detail, Clay, Layer, Blob, Scrape, Smudge, SnakeHook, Twist };
  using Tip = BrushTip;
  // `across`: the way the stroke is taken to go before it has moved (the view's right, say), for the oval and the tilt.
  void begin(int brush, V3 at, double radius, double strength, int mirror, bool invert, const Tip &tip = Tip(), V3 across = {0, 0, 0});
  // `tilt` (degrees), when a number, leans this dab's push instead of the tip's (a pen held at a slant).
  void dab(V3 at, double pressure, double size = 1, double tilt = NAN);
  void end();
  bool undo();
  bool redo();
  // The size every brush makes the triangles under it (mm; 0: leaves them as they are). Taken by strokes begun after.
  void setDetail(double size) { detail = size > 0 && std::isfinite(size) ? size : 0; }
  // The points moved, turned, made or taken away since this was last asked, in order (and forgotten); the same for
  // the triangles' slots.
  std::vector<uint32_t> takeChanged();
  std::vector<uint32_t> takeChangedTriangles();

  // Every slot, in use or not (a free triangle slot's corners mean nothing).
  const std::vector<V3> &points() const { return p; }
  const std::vector<V3> &normals() const { return n; }
  const std::vector<uint32_t> &triangles() const { return tri; }
  bool triangleAlive(uint32_t t) const { return triAlive[t] != 0; }
  size_t liveTriangles() const { return live; }
  // The mesh as it is: the points in use and the triangles, in slot order, the points renumbered.
  void compact(std::vector<V3> &pts, std::vector<uint32_t> &tris) const;
  // What's wrong with its links ("" when nothing): each side met by one running back, every point's fan closed, the box
  // tree holding every triangle in use and no other, every number finite.
  std::string check() const;

 private:
  struct TreeNode {
    V3 lo, hi;
    int left = -1, right = -1;
    std::vector<uint32_t> items;  // a leaf's triangles
  };
  // A slot as a stroke found it and left it.
  struct PointSlot {
    V3 at;
    uint32_t corner;
    uint8_t alive;
    bool operator==(const PointSlot &o) const { return at == o.at && corner == o.corner && alive == o.alive; }
  };
  struct TriSlot {
    uint32_t v[3], o[3];
    uint8_t alive;
    bool operator==(const TriSlot &x) const {
      return alive == x.alive && v[0] == x.v[0] && v[1] == x.v[1] && v[2] == x.v[2] && o[0] == x.o[0] && o[1] == x.o[1] && o[2] == x.o[2];
    }
  };
  struct Step {
    std::vector<uint32_t> pIdx, tIdx;
    std::vector<PointSlot> pBefore, pAfter;
    std::vector<TriSlot> tBefore, tAfter;
    size_t size() const { return pIdx.size() + tIdx.size(); }
  };

  bool good = false;
  // The mesh as a corner table: corner c (3 per triangle) is at point tri[c]; opp[c] is the corner across the side
  // facing c (that side running the other way in its triangle); corner[v] is one of v's corners.
  std::vector<V3> p, n;
  std::vector<uint32_t> tri, opp, corner;
  std::vector<uint8_t> triAlive, ptAlive;
  // Free slots, the lowest used first.
  std::priority_queue<uint32_t, std::vector<uint32_t>, std::greater<uint32_t>> freeTris, freePts;
  size_t live = 0;
  double detail = 0;

  std::vector<TreeNode> nodes;  // a box tree over the triangles (a node's children always come after it)
  std::vector<int> leafOf, parentOf;
  std::vector<uint8_t> dirtyNode;
  size_t grownLeaves = 0, liveAtBuild = 0;
  mutable std::vector<uint32_t> seen;
  mutable uint32_t seenStamp = 0;
  std::vector<uint32_t> region;
  uint32_t regionStamp = 0;

  // The stroke under way.
  bool stroking = false;
  int brush = 0;
  double radius = 1, strength = 0.5, strokeDetail = 0;
  int mirror = 0;
  bool invert = false;
  Tip tip;
  double tiltNow = 0;     // this dab's lean (degrees)
  V3 way{0, 0, 0};        // the stroke's way (unit; `across` until it has moved)
  double moving = 0;      // how far this dab is from the one before (smudge)
  V3 hook{0, 0, 0};       // snake hook: how far this step pulls
  V3 start{0, 0, 0}, last{0, 0, 0};
  bool dabbed = false;
  std::vector<uint32_t> grabbed;      // grab: the points under it at the start (or under a mirror of it), in order,
  std::vector<double> grabWeight[8];  // how much each follows the drag, and each mirror of it,
  std::vector<V3> grabFrom;           // and where each was
  // Layer: per point, the stroke it was first raised in, from where, along which way, and how high it's got.
  std::vector<uint32_t> layerOf;
  std::vector<V3> layerFrom, layerAlong;
  std::vector<double> layerHeight;
  std::vector<uint32_t> pointTouched, triTouched;  // per slot: the stroke that last recorded it
  uint32_t strokeId = 0;
  Step step;
  std::vector<Step> undone, done;
  size_t kept = 0;  // slots held by `done` and `undone`

  std::vector<uint8_t> changedFlag, changedTriFlag;
  std::vector<uint32_t> changed, changedTris;
  std::vector<uint32_t> dirtyList;
  // (Scratch, kept between dabs.)
  std::vector<uint32_t> idx[8], sum, merged, around, ring, work, front, fresh;
  std::vector<double> wt[8];
  std::vector<V3> off[8], offSum, offMerged;
  mutable std::vector<uint32_t> fan;
  mutable std::vector<V3> faceN;
  mutable std::vector<uint32_t> faceAt;
  mutable uint32_t faceStamp = 0;

  bool link();
  uint32_t swing(uint32_t c) const;
  uint32_t sideOf(uint32_t a, uint32_t b) const;
  void neighbours(uint32_t v, std::vector<uint32_t> &out) const;
  void rebuildTree();
  int build(std::vector<uint32_t> &order, int first, int count, const std::vector<V3> &mid, int parent);
  void boxOf(uint32_t t, V3 &lo, V3 &hi) const;
  void addToLeaf(uint32_t t, int leaf);
  void removeFromLeaf(uint32_t t);
  void dirty(int node);
  V3 normalAt(uint32_t v, bool cache = false) const;
  // The points within `r` of c, in order, and each one's weight (1 at the middle, falling smoothly to 0 at r).
  void within(V3 c, double r, std::vector<uint32_t> &out, std::vector<double> &weight) const;
  // The corners of the triangles passing within `r` of c, their points, in order.
  void touching(V3 c, double r, std::vector<uint32_t> &out) const;
  // The mirrors in use (k: bits x, y, z flipped), in order; where they put a point.
  int copies(int out[8]) const;
  static V3 flip(V3 v, int k) { return {k & 1 ? -v.x : v.x, k & 2 ? -v.y : v.y, k & 4 ? -v.z : v.z}; }
  // The points under a dab at c, and each one's weight, as the tip has it (the stroke's way there: dir).
  void weigh(V3 c, double radius, V3 dir, std::vector<uint32_t> &which, std::vector<double> &weight, V3 &out) const;
  static V3 wayAlong(V3 dir, V3 out);
  void growLayer();
  void anchorLayer();
  // How a dab at c would move the points under it (from where they are now), its mirror k of the stroke.
  void offsets(V3 c, double pressure, double radius, int k, std::vector<uint32_t> &which, std::vector<double> &weight, std::vector<V3> &by) const;
  void dabAt(V3 c, double pressure, double radius);
  // The triangles made the detail size round `seeds`' points (the sides for which inside(a, b) holds), at most `most`
  // changes; `mark`: a point made on a side between two points of `region` joins it. The points changed are left in
  // `fresh`; their normals made again unless `later` (the caller moves them with the rest).
  template <class Inside>
  void retopo(const std::vector<uint32_t> &seeds, Inside inside, bool mark, int most, bool later);
  bool split(uint32_t e, uint32_t &m);
  bool collapse(uint32_t e);
  bool flip(uint32_t e);
  int valence(uint32_t v) const;
  void refreshBoxes();
  uint32_t newPoint(V3 at);
  uint32_t newTriangle();
  void killTriangle(uint32_t t);
  void touchTriangle(uint32_t t);
  void recordPoint(uint32_t v);
  void recordTriangle(uint32_t t);
  // After points moved: their triangles' boxes, their own and their neighbours' normals, and what's changed.
  void moved(const std::vector<uint32_t> &which);
  void restore(const Step &s, bool back);
  void keep(Step s);
};

}  // namespace bce
