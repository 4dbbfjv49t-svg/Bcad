// Bcad's own geometry engine behind the same C API as BcadKernel.cpp (BcadKernel.h): shapes, placements, meshes and
// measuring so far. What it can't do yet answers with nothing and says so in bk_last_error.
#include "BcadKernel.h"

#include "Engine/Distance.hpp"
#include "Engine/Fasteners.hpp"
#include "Engine/Model.hpp"

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace bce;

struct BKShape {
  Shape shape;
};

static thread_local std::string lastError;

static const char *notYet = "not in Bcad's engine yet";

const char *bk_last_error(void) { return lastError.c_str(); }
// No OpenCascade in this engine.
const char *bk_occt_version(void) { return ""; }

static bool finite(const double *v, int n) {
  for (int i = 0; i < n; i++)
    if (!std::isfinite(v[i])) return false;
  return true;
}

BKShape *bk_primitive(int kind, const double *p) {
  Shape s;
  std::string why;
  if (!p || !primitive(kind, p, s, why)) {
    lastError = p ? why : "shape: no sizes";
    return nullptr;
  }
  return new BKShape{s};
}

BKShape *bk_transform(const BKShape *s, const double *m) {
  if (!s) return nullptr;
  if (!m || !finite(m, 12)) {
    lastError = "transform: placement must be numbers";
    return nullptr;
  }
  Affine a = Affine::from(m);
  if (std::fabs(a.det()) <= 1e-12) {
    lastError = "transform: placement flattens the shape";
    return nullptr;
  }
  return new BKShape{{s->shape.node, s->shape.place.then(a)}};
}

int bk_piece_count(const BKShape *s) { return s ? pieceCount(s->shape) : 0; }

int bk_bounds(const BKShape *s, const double *m, double *out) {
  if (!s || !out) return -1;
  if (!m || !finite(m, 12)) {
    lastError = "bounds: placement must be numbers";
    return -1;
  }
  V3 lo, hi;
  bool exact = placedBounds({s->shape.node, s->shape.place.then(Affine::from(m))}, lo, hi);
  double v[6] = {lo.x, lo.y, lo.z, hi.x, hi.y, hi.z};
  memcpy(out, v, sizeof v);
  return exact ? 1 : 0;
}
BKShape *bk_copy(const BKShape *s) { return s ? new BKShape{s->shape} : nullptr; }
void bk_free(BKShape *s) { delete s; }

// MARK: - meshes

template <typename T> static T *mallocCopy(const std::vector<T> &v) {
  T *out = (T *)malloc(sizeof(T) * std::max<size_t>(1, v.size()));
  if (!v.empty()) memcpy(out, v.data(), sizeof(T) * v.size());
  return out;
}

BKMesh *bk_mesh(const BKShape *s, double deflection) {
  if (!s) return nullptr;
  BKMesh *m = new BKMesh();
  Solid solid;
  mesh(s->shape, std::isfinite(deflection) ? std::max(deflection, 0.001) : 0.05, solid);
  std::vector<float> pos, nrm, ep, cs;
  pos.reserve(solid.p.size() * 3), nrm.reserve(solid.n.size() * 3);
  for (size_t i = 0; i < solid.p.size(); i++) {
    V3 a = solid.p[i], b = solid.n[i];
    pos.insert(pos.end(), {(float)a.x, (float)a.y, (float)a.z});
    nrm.insert(nrm.end(), {(float)b.x, (float)b.y, (float)b.z});
  }
  std::vector<double> info;
  for (const auto &f : solid.faces) info.insert(info.end(), {f.normal.x, f.normal.y, f.normal.z, f.centroid.x, f.centroid.y, f.centroid.z});
  std::vector<uint32_t> es{0};
  std::vector<int32_t> ef;
  for (const auto &e : solid.edges) {
    for (V3 q : e.pts) ep.insert(ep.end(), {(float)q.x, (float)q.y, (float)q.z});
    es.push_back((uint32_t)(ep.size() / 3));
    ef.insert(ef.end(), {e.f0, e.f1});
  }
  std::vector<double> circles;
  for (const auto &c : solid.circles) circles.insert(circles.end(), {c.centre.x, c.centre.y, c.centre.z, c.axis.x, c.axis.y, c.axis.z, c.radius});
  for (V3 q : solid.corners) cs.insert(cs.end(), {(float)q.x, (float)q.y, (float)q.z});
  m->vertexCount = (int)solid.p.size();
  m->triangleCount = (int)(solid.tri.size() / 3);
  m->positions = mallocCopy(pos);
  m->normals = mallocCopy(nrm);
  m->indices = mallocCopy(solid.tri);
  m->triangleFace = mallocCopy(solid.triFace);
  m->faceCount = (int)solid.faces.size();
  m->faceInfo = mallocCopy(info);
  m->edgeCount = (int)solid.edges.size();
  m->edgePointCount = (int)(ep.size() / 3);
  m->edgePoints = mallocCopy(ep);
  m->edgeStart = mallocCopy(es);
  m->edgeFaces = mallocCopy(ef);
  m->circleCount = (int)solid.circles.size();
  m->circles = mallocCopy(circles);
  m->cornerCount = (int)solid.corners.size();
  m->corners = mallocCopy(cs);
  V3 lo, hi;
  bounds(s->shape, lo, hi, &solid);
  if (solid.p.empty()) lo = hi = V3{0, 0, 0};
  m->bbox[0] = lo.x, m->bbox[1] = lo.y, m->bbox[2] = lo.z, m->bbox[3] = hi.x, m->bbox[4] = hi.y, m->bbox[5] = hi.z;
  m->volume = volume(s->shape, &solid);
  m->valid = 1;
  return m;
}

void bk_mesh_free(BKMesh *m) {
  if (!m) return;
  free(m->positions);
  free(m->normals);
  free(m->indices);
  free(m->triangleFace);
  free(m->faceInfo);
  free(m->edgePoints);
  free(m->edgeStart);
  free(m->edgeFaces);
  free(m->circles);
  free(m->corners);
  delete m;
}

// MARK: - measuring

static bool endOf(const BKShape *s, const double *m, int kind, int index, const double *point, End &e) {
  if (kind == BK_END_POINT) {
    if (!point || !finite(point, 3)) {
      lastError = "distance: a point must be numbers";
      return false;
    }
    e.kind = End::Point, e.point = {point[0], point[1], point[2]};
    return true;
  }
  if (!s || !m || !finite(m, 12) || (kind != BK_END_EDGE && kind != BK_END_FACE)) {
    lastError = "distance: no shape to measure";
    return false;
  }
  e.kind = kind == BK_END_EDGE ? End::Edge : End::Face;
  e.shape = &s->shape, e.place = Affine::from(m), e.index = index;
  return true;
}

double bk_distance(const BKShape *a, const double *ma, int kindA, int indexA, const double *pointA, const BKShape *b,
                   const double *mb, int kindB, int indexB, const double *pointB, double *out) {
  End ea, eb;
  if (!endOf(a, ma, kindA, indexA, pointA, ea) || !endOf(b, mb, kindB, indexB, pointB, eb)) return -1;
  double d;
  V3 pa, pb;
  std::string why;
  if (!distance(ea, eb, d, pa, pb, why)) {
    lastError = why;
    return -1;
  }
  if (out) {
    double v[6] = {pa.x, pa.y, pa.z, pb.x, pb.y, pb.z};
    memcpy(out, v, sizeof v);
  }
  return d;
}

// MARK: - not yet

static BKShape *later(const char *what) {
  lastError = std::string(what) + ": " + notYet;
  return nullptr;
}

BKShape *bk_fastener(const BKFastener *f, double clearance) {
  (void)clearance;
  if (f)
    if (const char *why = fastenerMisfit(*f)) return lastError = std::string("bolt: ") + why, nullptr;
  return later("bolt");
}

// MARK: - merging and splitting

BKShape *bk_boolean(int op, const BKShape *a, const BKShape *b) {
  if (!a || !b) return a ? bk_copy(a) : (b && op == BK_UNION ? bk_copy(b) : nullptr);
  if (op != BK_UNION && op != BK_SUBTRACT && op != BK_INTERSECT) {
    lastError = "combine: unknown operation";
    return nullptr;
  }
  // The parts keep their own placements; the result sits where they are.
  auto node = std::make_shared<Node>();
  node->kind = Node::Bool, node->op = op, node->a = a->shape, node->b = b->shape;
  return new BKShape{{node, Affine()}};
}

BKShape *bk_split(const BKShape *s, const double *p, const double *n, int side) {
  if (!s) return nullptr;
  if (!p || !n || !finite(p, 3) || !finite(n, 3)) {
    lastError = "split: plane must be numbers";
    return nullptr;
  }
  V3 nv{n[0], n[1], n[2]};
  if (norm(nv) < 1e-12) {
    lastError = "split: plane must be numbers";
    return nullptr;
  }
  auto node = std::make_shared<Node>();
  node->kind = Node::Split, node->a = s->shape, node->p = {p[0], p[1], p[2]}, node->n = unit(nv), node->side = side == 0 ? 0 : 1;
  return new BKShape{{node, Affine()}};
}

BKShape *bk_fillet(const BKShape *, const int *, const double *, int, double, double *maxRadius, int *missing) {
  if (maxRadius) *maxRadius = 0;
  if (missing) *missing = 0;
  return later("round");
}

BKShape *bk_chamfer(const BKShape *, const int *, const double *, int, double, double, double, int *missing) {
  if (missing) *missing = 0;
  return later("bevel");
}

BKShape *bk_cove(const BKShape *, const int *, const double *, int, double, double *maxRadius, int *missing) {
  if (maxRadius) *maxRadius = 0;
  if (missing) *missing = 0;
  return later("cove");
}

BKShape *bk_hollow(const BKShape *, const BKShape *const *, int, const double *, int, const double *, const double *, int, double, int *missing) {
  if (missing) *missing = 0;
  return later("hollow");
}

BKSection *bk_section(const BKShape *, int, const double *, double) {
  lastError = std::string("section: ") + notYet;
  return nullptr;
}

void bk_section_free(BKSection *section) {
  if (!section) return;
  free(section->points);
  free(section->loopStart);
  delete section;
}

int bk_export_step(const BKShape *const *, int, const char *) {
  lastError = std::string("STEP: ") + notYet;
  return 0;
}
