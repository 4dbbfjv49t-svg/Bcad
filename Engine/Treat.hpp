// Bcad's geometry engine: a mesh's edges as the corners they are (creases), what a pick means on a mesh, the cut across an
// edge, roundings, inward roundings and bevels along edges, and hollowing.
#pragma once
#include "Engine/Model.hpp"

#include <string>
#include <vector>

namespace bce {

// An edge between two faces (or a run of edges meeting smoothly), seen at each of its points. Face A is the one whose
// outward normal at the run's middle points most up (ties: most +y, then +x), as OpenCascade's kernel names them; the
// "into" directions run from the edge square across it into each face.
struct Crease {
  std::vector<V3> pts;
  std::vector<int> fa, fb;          // the faces either side at each point (a run may pass from face to face)
  std::vector<V3> na, nb, ia, ib;   // outward normals and into-directions of faces A and B at each point
  std::vector<int> edges;           // the mesh's edges it runs along, in order
  bool closed = false;
  double angle = 90;                // the material angle between the faces at the middle, degrees (above 180: concave)
  double length = 0;

  // The tangent at point i (along the run).
  V3 tangent(size_t i) const;
  // The material angle at point i, degrees.
  double angleAt(size_t i) const;
};

// Edges a pick stands for on a mesh (as OpenCascade's kernel resolves them): an edge pick the edge through its point, a
// corner pick the edge at the corner leaving its face, a face pick its face's edges, a body pick every edge between faces
// that don't meet smoothly. Runs meeting smoothly are taken whole. `missing` counts picks that match nothing.
std::vector<Crease> creasesOf(const Solid &s, const int *kinds, const double *picks, int count, int *missing);

// The face a face pick (normal, a point on it) stands for (as OpenCascade's kernel finds it), or -1.
int faceAt(const Solid &s, const double *pick);

// The crease one pick stands for, described at the pick's point (an edge pick) or the middle of the longest edge; false
// when there is none.
bool creaseAt(const Solid &s, int kind, const double *pick, Crease &out, size_t &at);

// A point of its own on the run `along` its length (its faces' directions taken between its neighbours'), so a cut there
// has the run's frame there; its index.
size_t pointAt(Crease &c, double along);

// The cut across a crease at point i: the outlines of the solid in the plane square to the edge, in the end-on frame
// (origin on the edge, face A leaving along -x, y its outward normal), outlines counter-clockwise, holes clockwise.
// (With `faceOf`, each loop's sides' faces: side j from point j to the next.)
std::vector<std::vector<std::pair<double, double>>> sliceAcross(const Solid &s, const Crease &c, size_t i, std::vector<std::vector<int>> *faceOf = nullptr);

// A treatment along picked edges: a rounding (outward), an inward rounding (cove) or a bevel (legs along faces A and B,
// its edges with the faces rounded by `corner` when above zero).
struct Treatment {
  enum Kind { Round, Cove, Bevel } kind = Round;
  double radius = 0, legA = 0, legB = 0, corner = 0;
  std::vector<int> kinds;
  std::vector<double> picks;
};

// How a treatment came out: `fits` false when it doesn't (`why` says so; `most` the largest radius that does, 0 if
// none); picks that matched nothing.
struct TreatFit {
  bool fits = true;
  std::string why;
  double most = 0;
  int missing = 0;
};

// The solid treated along the picked edges, at a chord error of d (its tools made to the same detail). With `onto`, the
// tools made for s's edges are taken from (or added to) that solid instead (a hollow's inside, cut where s's edges are).
Solid treated(const Solid &s, const Treatment &t, double d, TreatFit &fit, const Solid *onto = nullptr);

// Hollowing: walls of `thickness` (faces picked as openings left out, others with walls of their own), and the shape
// without its roundings to hollow instead when the shape itself won't (only what lies inside the shape kept).
struct Hollowing {
  double thickness = 2;
  std::vector<double> open, walls, wallThickness;
  bool viaSharp = false;
  Shape sharp;
};

// The shape (as placed) hollowed, at a chord error of d; false when the walls don't fit (`missing` the picks that match
// no face).
bool hollowed(const Shape &s, const Hollowing &h, double d, Solid &out, int *missing = nullptr);

}  // namespace bce
