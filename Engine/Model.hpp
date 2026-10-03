// Bcad's geometry engine: shapes described exactly — primitives, and merges and splits of shapes — and their meshes with
// the faces, edges, corners and circles the app works with.
#pragma once
#include "Engine/Math.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace bce {

// A piece of a profile in a half-plane (r ≥ 0, z): a straight segment, or an arc of a circle (centre, radius, angles
// a0 → a1; a1 < a0 runs clockwise). Turned round the z axis it sweeps a face; a segment on the axis sweeps none.
struct Elem {
  double r0, z0, r1, z1;
  bool arc = false;
  double cr = 0, cz = 0, rad = 0, a0 = 0, a1 = 0;

  static Elem line(double r0, double z0, double r1, double z1) { return {r0, z0, r1, z1}; }
  static Elem arcOf(double cr, double cz, double rad, double a0, double a1) {
    Elem e{cr + rad * std::cos(a0), cz + rad * std::sin(a0), cr + rad * std::cos(a1), cz + rad * std::sin(a1), true, cr, cz, rad, a0, a1};
    return e;
  }
  bool onAxis() const { return !arc && r0 == 0 && r1 == 0; }
  bool flat() const { return !arc && z0 == z1; }
  bool same(const Elem &o) const {
    return arc == o.arc && r0 == o.r0 && z0 == o.z0 && r1 == o.r1 && z1 == o.z1 && cr == o.cr && cz == o.cz && rad == o.rad;
  }
  // The point at t ∈ [0, 1] (the ends exactly at the ends).
  void at(double t, double &r, double &z) const;
  // Outward normal at t, for a profile running counter-clockwise round its region.
  void normalAt(double t, double &nr, double &nz) const;
  // The nearest point to (r, z), as t.
  double nearest(double r, double z) const;
  // How many pieces it's cut into for a chord error of at most `deflection`.
  int pieces(double deflection) const;
};

// A face as the engine knows it exactly. Turned: a profile piece turned round the z axis of `place` (which maps that frame
// to the solid's coordinates; the form holds while `exact`, i.e. the placements kept shapes similar). Flat faces also know
// their plane (n · x = d), and are exact as meshed. Curved: no simple form, only its mesh.
struct FaceGeom {
  enum Kind { Curved, Flat, Turned } kind = Curved;
  Elem elem{};
  Affine place;
  bool exact = true;
  bool flat = false;
  V3 pn;
  double pd = 0;
};

// An edge exactly: a circle round the z axis of `place` (radius r at height z), a profile piece in that frame's half-plane
// y = 0, a straight line (exact as meshed), or a curve known only by its points.
struct EdgeGeom {
  enum Kind { Curve, Line, Circle, Profile } kind = Curve;
  double r = 0, z = 0;
  Elem elem{};
  Affine place;
  bool exact = true;
};

// A mesh with what the app needs to pick and measure: a face per triangle, and per face its normal and centroid; edges
// as polylines with the faces either side (-1 for none); corners; circles (centre, axis, radius).
struct Solid {
  std::vector<V3> p, n;
  std::vector<uint32_t> tri;
  std::vector<uint32_t> triFace;
  struct Face {
    V3 normal, centroid;
    FaceGeom geom;
    // The volume between this face's mesh and the exact surface (the mesh lies inside a convex surface): what the mesh's
    // own volume misses here.
    double deficit = 0;
    // Made by a rounding or an inward rounding: it meets the faces beside it smoothly, however its mesh bends there.
    bool blend = false;
    // A tool's face meant to lie outside what it takes away (inside what it adds): any of it left is a sliver where meshes
    // meet near tangent, to be taken into the face beside it.
    bool aux = false;
  };
  std::vector<Face> faces;
  struct Edge {
    std::vector<V3> pts;
    int f0 = -1, f1 = -1;
    EdgeGeom geom;
  };
  std::vector<Edge> edges;
  std::vector<V3> corners;
  struct Circle {
    V3 centre, axis;
    double radius;
  };
  std::vector<Circle> circles;
  // Per triangle, how far the exact surface lies beyond it (outward; less than zero where it curves in), as a quadratic
  // over the triangle: its values at the three corners, then at the midpoints of sides 0–1, 1–2, 2–0. Over any part of
  // the triangle it gives the volume the mesh misses there, so whatever a merge or split keeps of a triangle keeps
  // exactly its share. Empty where every face is flat.
  std::vector<double> gap;
  // The grid a merge last put its points on (0: none, or moved off it since): a merge of this keeps to that grid, so its
  // points stay where they are and a part made afresh lands on them.
  double grid = 0;

  uint32_t vertex(V3 pos, V3 nrm) {
    p.push_back(pos), n.push_back(nrm);
    return (uint32_t)p.size() - 1;
  }
  void triangle(uint32_t a, uint32_t b, uint32_t c, int face) {
    tri.push_back(a), tri.push_back(b), tri.push_back(c), triFace.push_back((uint32_t)face);
  }
  // Each face's area centroid from its triangles.
  void centroids();
  // The mesh's own volume (closed and facing outwards).
  double meshVolume() const;
  // Each curved face's deficit shared out over its triangles by how the surface bends along each side (from the corner
  // normals: a side of length l whose ends' normals turn by angle a misses about l·a/8 at its middle).
  void slivers();
  // The volume missed over triangle t.
  double sliver(size_t t) const;
  // Everything carried by a placement: the mesh, normals, edges, corners, the exact forms; circles while it keeps them
  // circles.
  void transform(const Affine &a);
};

// A triangle's gap (6 values as in Solid::gap) at barycentric (l0, l1, l2).
inline double gapAt(const double *g, double l0, double l1, double l2) {
  return g[0] * l0 * (2 * l0 - 1) + g[1] * l1 * (2 * l1 - 1) + g[2] * l2 * (2 * l2 - 1) + 4 * (g[3] * l0 * l1 + g[4] * l1 * l2 + g[5] * l2 * l0);
}
// The gap over a piece of triangle (a, b, c) with corners q[0..2] (in its plane): the same quadratic, put by the piece's
// corners and midpoints.
inline void gapOfPiece(const double *g, V3 a, V3 b, V3 c, const V3 q[3], double *out) {
  V3 nn = cross(b - a, c - a);
  double whole = dot(nn, nn);
  double l[3][3];
  for (int k = 0; k < 3; k++) {
    double l0 = whole > 0 ? dot(cross(b - q[k], c - q[k]), nn) / whole : 1.0 / 3, l1 = whole > 0 ? dot(cross(c - q[k], a - q[k]), nn) / whole : 1.0 / 3;
    l[k][0] = l0, l[k][1] = l1, l[k][2] = 1 - l0 - l1;
  }
  for (int k = 0; k < 3; k++) {
    int j = (k + 1) % 3;
    out[k] = gapAt(g, l[k][0], l[k][1], l[k][2]);
    out[3 + k] = gapAt(g, (l[k][0] + l[j][0]) / 2, (l[k][1] + l[j][1]) / 2, (l[k][2] + l[j][2]) / 2);
  }
}

// A shape the way Bcad makes it: flat-sided, turned round the z axis, or a tube swept along an oval in the xy plane.
struct Model {
  enum Kind { Poly, Turned, Swept } kind = Poly;
  // Flat-sided: corners, and faces as corner loops counter-clockwise seen from outside.
  std::vector<V3> verts;
  std::vector<std::vector<int>> loops;
  // Turned: a closed profile, counter-clockwise round its region.
  std::vector<Elem> profile;
  // Turned: the steps round the axis when set (to meet another mesh point for point), else from the deflection; and
  // heights where its mesh has rings on slopes and arcs (where a part on the same axis ends).
  int around = 0;
  std::vector<double> levels;
  // Swept: the oval's semi-axes along (cos phi, sin phi) and across it, and the tube's outline round the oval's line
  // (r along the oval's outward normal, z up), counter-clockwise.
  double a = 0, b = 0, phi = 0;
  std::vector<Elem> section;
  double volume = 0;

  // The largest d · x over the shape, and (if asked) a point where it's reached.
  double support(V3 d, V3 *at = nullptr) const;
  // The mesh, every chord within `deflection` of the exact shape.
  void build(Solid &out, double deflection) const;
  // Turned: the steps its mesh takes round the axis at that deflection.
  int columns(double deflection) const;
};

struct Node;
struct Treatment;
struct Hollowing;

// A shape as the C API holds it: what it is and where it's placed (shared, so copies cost nothing).
struct Shape {
  std::shared_ptr<const Node> node;
  Affine place;
};

// What a shape is: a primitive, a merge (union, subtract, intersect as BK_UNION …) of two placed shapes, the side of a
// plane (point p, normal n; side 0 is where n points) of a placed shape, or a placed shape with its edges treated
// (rounded, bevelled) or hollowed (Treat.hpp). Its meshes are made when asked, at the detail asked, and the last few kept.
struct Node {
  enum Kind { Prim, Bool, Split, Treat, Hollow } kind = Prim;
  std::shared_ptr<const Model> model;
  std::shared_ptr<const Treatment> treat;
  std::shared_ptr<const Hollowing> hollow;
  int op = 0;
  Shape a, b;
  V3 p, n;
  int side = 0;
  mutable std::vector<std::pair<double, std::shared_ptr<const Solid>>> made;
  mutable std::vector<std::pair<double, int>> counted;  // its pieces at a detail, once counted
  // A treatment's or hollow's result as made when asked for (what was shown): what it stands for at any other detail
  // where it can't be made again.
  std::shared_ptr<const Solid> shown;
  // (The kept meshes, should two threads ask at once.)
  mutable std::mutex lock;
};

// The model for a primitive (kinds and sizes as in BcadKernel.h) and the placement that centres it; empty with `why` set
// when the sizes don't make one.
bool primitive(int kind, const double *p, Shape &out, std::string &why);

// Models made as tools (roundings, hollows): a turned outline (closed, counter-clockwise round its region), and a
// flat-sided solid from its corners and faces (corner loops, turned outward when they come inward); volumes worked out.
std::shared_ptr<Model> turnedModel(std::vector<Elem> profile);
// ∮ r²/2 dz along a profile piece: its share of the area moment about the axis (times the angle turned: the volume).
double profileMoment(const Elem &e);
// A tube along an oval (semi-axes a, b, turned phi): its section (r along the oval's outward normal, z up), volume worked
// out.
std::shared_ptr<Model> sweptModel(double a, double b, double phi, std::vector<Elem> section);
std::shared_ptr<Model> polyModel(std::vector<V3> verts, std::vector<std::vector<int>> loops);
// A shape of a model, or of two shapes merged (op as BK_UNION …).
Shape shapeOf(std::shared_ptr<const Model> m, const Affine &place = Affine());
Shape merged(int op, const Shape &a, const Shape &b);

// A node's mesh in its own coordinates at a chord error of `deflection`.
std::shared_ptr<const Solid> evaluate(const Node &n, double deflection);

// The mesh of a placed shape at a chord error of `deflection` (in placed millimetres).
void mesh(const Shape &s, double deflection, Solid &out);

// How far a shape reaches along d (placed coordinates), where, and whether that's exact (otherwise it's from a mesh).
struct Reach {
  double value;
  V3 point;
  bool exact;
};
// prove: where only a test against the other part's mesh could tell (a subtract's or intersect's farthest point inside
// the other part or not), make that test; otherwise the reach counts as inexact.
Reach support(const Shape &s, V3 d, bool prove = true);

// The bounding box and volume of a placed shape: exact for primitives and wherever it can be told exactly; otherwise from
// `meshed` (the shape's mesh, if at hand) or a mesh made for it.
void bounds(const Shape &s, V3 &lo, V3 &hi, const Solid *meshed = nullptr);
double volume(const Shape &s, const Solid *meshed = nullptr);
// The box of a shape turned or stretched any way, quickly (asked as it turns): exact wherever that's quick to tell (a
// primitive's always), otherwise from the points of its display mesh. True when all of it is exact.
bool placedBounds(const Shape &s, V3 &lo, V3 &hi);
// Separate pieces of a placed shape (as shown), counted once per detail.
int pieceCount(const Shape &s);

// Whether a point is inside a closed mesh (by its winding number).
bool inside(const Solid &s, V3 q);

// Merges and splits of meshes (Boolean.cpp, Split.cpp), and what's made of their results afterwards (Csg.cpp): faces on
// one surface joined, edges, corners and circles found again.
// merge: crossing points this close (a part of the shapes' size) are one.
Solid combine(const Solid &a, const Solid &b, int op, double merge = 1e-11, bool keepGrid = false);
// What the mesh booleans on this thread ran into since it was last cleared (for tests and measuring): merges made, cuts
// a triangle's triangulation couldn't keep or lost an end of, points it made where two cuts crossed, cuts led round an
// outline corner, points put on a side that wasn't found, results left open, regions judged inside or out by a winding
// number too near a half to be sure, and regions whose pieces beside cuts disagreed (a cut not made let them run on).
struct CombineReport {
  long calls = 0, keepsFailed = 0, segsDropped = 0, crossingsMade = 0, detours = 0, edgeFallbacks = 0, open = 0, unsure = 0, torn = 0;
};
extern thread_local CombineReport combineReport;
Solid cut(const Solid &s, V3 p, V3 n, int side);
// Whether two faces lie on one surface (one plane facing one way; one turned profile piece in one place).
bool sameForm(const FaceGeom &a, const FaceGeom &b);
void finish(Solid &s, double deflection);
// Closed shells of a mesh that enclose material (an inner void isn't one).
int pieces(const Solid &s);

}  // namespace bce
