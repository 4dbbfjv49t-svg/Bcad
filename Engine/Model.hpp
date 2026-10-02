// Bcad's geometry engine: shapes described exactly, and their meshes with the faces, edges, corners and circles the app
// works with.
#pragma once
#include "Engine/Math.hpp"

#include <cstdint>
#include <memory>
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
  // The point at t ∈ [0, 1] (the ends exactly at the ends).
  void at(double t, double &r, double &z) const;
  // Outward normal at t, for a profile running counter-clockwise round its region.
  void normalAt(double t, double &nr, double &nz) const;
  // The nearest point to (r, z), as t.
  double nearest(double r, double z) const;
  // How many pieces it's cut into for a chord error of at most `deflection`.
  int pieces(double deflection) const;
};

// A face as the engine knows it exactly, for measuring to it: a profile piece turned round the z axis, or none (its mesh
// is exact, as for a flat polygon, or it has no simple form).
struct FaceGeom {
  enum Kind { MeshOnly, Turned } kind = MeshOnly;
  Elem elem{};
};

// An edge exactly: a circle round the z axis (radius r at height z), a profile piece in the half-plane y = 0, or a
// polyline as it is.
struct EdgeGeom {
  enum Kind { Polyline, Circle, Profile } kind = Polyline;
  double r = 0, z = 0;
  Elem elem{};
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
  // Whether the exact forms above (FaceGeom, EdgeGeom) still describe the faces and edges: true while the placement keeps
  // shapes similar; their coordinates are then `local` ones, `place` maps them to the mesh's.
  bool exact = true;
  Affine place;

  uint32_t vertex(V3 pos, V3 nrm) {
    p.push_back(pos), n.push_back(nrm);
    return (uint32_t)p.size() - 1;
  }
  void triangle(uint32_t a, uint32_t b, uint32_t c, int face) {
    tri.push_back(a), tri.push_back(b), tri.push_back(c), triFace.push_back((uint32_t)face);
  }
  // Each face's area centroid from its triangles.
  void centroids();
  // Everything carried by a placement: the mesh, normals, edges, corners; circles while it keeps them circles.
  void transform(const Affine &a);
};

// A shape the way Bcad makes it: flat-sided, turned round the z axis, or a tube swept along an oval in the xy plane.
struct Model {
  enum Kind { Poly, Turned, Swept } kind = Poly;
  // Flat-sided: corners, and faces as corner loops counter-clockwise seen from outside.
  std::vector<V3> verts;
  std::vector<std::vector<int>> loops;
  // Turned: a closed profile, counter-clockwise round its region.
  std::vector<Elem> profile;
  // Swept: the oval's semi-axes along (cos phi, sin phi) and across it, and the tube's outline round the oval's line
  // (r along the oval's outward normal, z up), counter-clockwise.
  double a = 0, b = 0, phi = 0;
  std::vector<Elem> section;
  double volume = 0;

  // The largest d · x over the shape.
  double support(V3 d) const;
  // The mesh, every chord within `deflection` of the exact shape.
  void build(Solid &out, double deflection) const;
};

// A shape as the C API holds it: a model and where it's placed (shared, so copies cost nothing).
struct Shape {
  std::shared_ptr<const Model> model;
  Affine place;
};

// The model for a primitive (kinds and sizes as in BcadKernel.h) and the placement that centres it; empty with `why` set
// when the sizes don't make one.
bool primitive(int kind, const double *p, Shape &out, std::string &why);

// The mesh of a placed shape at a chord error of `deflection` (in placed millimetres).
void mesh(const Shape &s, double deflection, Solid &out);

// The exact bounding box and volume of a placed shape.
void bounds(const Shape &s, V3 &lo, V3 &hi);
double volume(const Shape &s);

}  // namespace bce
