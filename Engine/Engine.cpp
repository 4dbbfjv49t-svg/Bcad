// Bcad's geometry engine behind its C API (BcadKernel.h): shapes, bolts and nuts, placements, meshes, boxes, measuring,
// merging, splitting, rounding, hollowing and STEP files.
#include "BcadKernel.h"

#include "Engine/Bolts.hpp"
#include "Engine/Distance.hpp"
#include "Engine/Fasteners.hpp"
#include "Engine/Model.hpp"
#include "Engine/Step.hpp"
#include "Engine/Treat.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace bce;

struct BKShape {
  Shape shape;
};

static thread_local std::string lastError;

const char *bk_last_error(void) { return lastError.c_str(); }

const Shape &bce::heldShape(const BKShape *s) { return s->shape; }

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
  // A turn given to a few digits (its columns a hair off square, or off one length) made exactly the turn it stands
  // for, so circles stay circles: moved no more than those digits are off.
  if (!a.similarity()) {
    V3 c[3] = {a.column(0), a.column(1), a.column(2)};
    double s = trig::cbrt(std::fabs(a.det()));
    bool near = true;
    for (int i = 0; i < 3; i++) {
      near = near && std::fabs(norm(c[i]) - s) <= 1e-5 * s;
      for (int j = i + 1; j < 3; j++) near = near && std::fabs(dot(c[i], c[j])) <= 1e-5 * s * s;
    }
    if (near) {
      // The nearest turn (polar decomposition by averaging with the inverse transpose), times the scale.
      Affine r = a;
      for (int k = 0; k < 8; k++) {
        Affine inv = r.inverse();
        for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++) r.m[4 * i + j] = (r.m[4 * i + j] / s + inv.m[4 * j + i] * s) / 2 * s;
      }
      for (int i = 0; i < 3; i++) r.m[4 * i + 3] = a.m[4 * i + 3];
      if (r.similarity()) a = r;
    }
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
  // Closed: every side of a triangle met by one running the other way (otherwise the app says it isn't a solid).
  m->valid = solid.tri.empty() || shut(solid) ? 1 : 0;
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

// MARK: - bolts and nuts

BKShape *bk_fastener(const BKFastener *f, double clearance) {
  std::string what = f && isNut(f->kind) ? "nut: " : "bolt: ";
  if (!f || !std::isfinite(clearance)) return lastError = what + "sizes must be numbers", nullptr;
  if (const char *why = fastenerMisfit(*f)) return lastError = what + why, nullptr;
  Shape out;
  std::string why;
  if (!fastener(*f, clearance, out, why)) return lastError = what + why, nullptr;
  return new BKShape{out};
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

// A treatment of the picked edges, made now at the detail shown (so whether it fits is known at once, and that mesh kept).
static BKShape *treat(const BKShape *s, const int *kinds, const double *picks, int count, Treatment t, double *maxRadius, int *missing,
                      const char *what) {
  if (maxRadius) *maxRadius = t.radius;
  if (missing) *missing = 0;
  if (!s) return nullptr;
  if (count < 0 || (count > 0 && (!kinds || !picks)) || !finite(picks, count * 6) || !std::isfinite(t.radius) || !std::isfinite(t.legA) ||
      !std::isfinite(t.legB) || !std::isfinite(t.corner)) {
    lastError = std::string(what) + ": must be numbers";
    return nullptr;
  }
  t.kinds.assign(kinds, kinds + count);
  t.picks.assign(picks, picks + count * 6);
  const double d = 0.05;
  Solid a;
  mesh(s->shape, d, a);
  TreatFit fit;
  Solid made = treated(a, t, d, fit);
  if (missing) *missing = fit.missing;
  if (!fit.fits) {
    if (maxRadius) *maxRadius = fit.most;
    lastError = fit.why;
    return nullptr;
  }
  auto node = std::make_shared<Node>();
  node->kind = Node::Treat, node->a = s->shape, node->treat = std::make_shared<Treatment>(t);
  node->shown = std::make_shared<Solid>(std::move(made));
  node->made.push_back({d, node->shown});
  return new BKShape{{node, Affine()}};
}

BKShape *bk_fillet(const BKShape *s, const int *kinds, const double *picks, int count, double radius, double *maxRadius, int *missing) {
  Treatment t;
  t.kind = Treatment::Round, t.radius = radius;
  return treat(s, kinds, picks, count, t, maxRadius, missing, "rounding");
}

BKShape *bk_chamfer(const BKShape *s, const int *kinds, const double *picks, int count, double legA, double legB, double cornerRadius, int *missing) {
  Treatment t;
  t.kind = Treatment::Bevel, t.legA = legA, t.legB = legB, t.corner = cornerRadius;
  return treat(s, kinds, picks, count, t, nullptr, missing, "bevel");
}

BKShape *bk_cove(const BKShape *s, const int *kinds, const double *picks, int count, double radius, double *maxRadius, int *missing) {
  Treatment t;
  t.kind = Treatment::Cove, t.radius = radius;
  return treat(s, kinds, picks, count, t, maxRadius, missing, "cove");
}

BKShape *bk_hollow(const BKShape *s, const BKShape *const *sharp, int sharpCount, const double *open, int openCount, const double *walls,
                   const double *wallThickness, int wallCount, double thickness, int *missing) {
  if (missing) *missing = 0;
  if (!s) return nullptr;
  if (openCount < 0 || wallCount < 0 || sharpCount < 0 || (openCount > 0 && !open) || (wallCount > 0 && (!walls || !wallThickness)) ||
      !finite(open, openCount * 6) || !finite(walls, wallCount * 6) || !finite(wallThickness, wallCount) || !std::isfinite(thickness)) {
    lastError = "hollow: walls must be numbers";
    return nullptr;
  }
  Hollowing h;
  h.thickness = thickness;
  if (openCount > 0) h.open.assign(open, open + openCount * 6);
  if (wallCount > 0) h.walls.assign(walls, walls + wallCount * 6), h.wallThickness.assign(wallThickness, wallThickness + wallCount);
  // Made now at the detail shown (so whether the walls fit is known at once, and that mesh kept): the shape itself, or
  // failing that each of the shapes without its roundings, hollowed and kept to what lies inside the shape.
  const double d = 0.05;
  Solid made;
  int miss = 0;
  bool ok = hollowed(s->shape, h, d, made, &miss);
  for (int k = 0; k < sharpCount && !ok; k++) {
    if (!sharp || !sharp[k]) continue;
    Hollowing via = h;
    via.viaSharp = true, via.sharp = sharp[k]->shape;
    if ((ok = hollowed(s->shape, via, d, made))) h = via;
  }
  if (missing) *missing = miss;
  if (!ok) {
    lastError = "hollow: the walls don't fit this shape";
    return nullptr;
  }
  auto node = std::make_shared<Node>();
  node->kind = Node::Hollow, node->a = s->shape, node->hollow = std::make_shared<Hollowing>(h);
  node->shown = std::make_shared<Solid>(std::move(made));
  node->made.push_back({d, node->shown});
  return new BKShape{{node, Affine()}};
}

int bk_pick_edges(const BKShape *s, const int *kinds, const double *picks, int count, double *out, int max) {
  if (!s || count < 0 || (count > 0 && (!kinds || !picks)) || !finite(picks, count * 6)) return 0;
  Solid m;
  mesh(s->shape, 0.05, m);
  int missing = 0;
  std::vector<Crease> creases = creasesOf(m, kinds, picks, count, &missing);
  int n = 0;
  for (auto &c : creases) {
    if (c.pts.size() < 2) continue;
    // At its middle by length (a point of its own there).
    size_t i = pointAt(c, c.length / 2);
    V3 p = c.pts[i], t = c.tangent(i);
    if (out && n < max) {
      double v[6] = {p.x, p.y, p.z, t.x, t.y, t.z};
      memcpy(out + 6 * n, v, sizeof v);
    }
    n++;
  }
  return n;
}

BKSection *bk_section(const BKShape *s, int kind, const double *pick, double radius) {
  if (!s) return nullptr;
  const double none[6] = {0, 0, 0, 0, 0, 0};
  const double *q = pick ? pick : none;
  if (!finite(q, 6) || !std::isfinite(radius)) {
    lastError = "section: pick must be numbers";
    return nullptr;
  }
  Solid solid;
  mesh(s->shape, std::max(radius > 0 ? radius / 400 : 0.05, 1e-4), solid);
  Crease c;
  size_t at = 0;
  if (!creaseAt(solid, kind, q, c, at)) {
    lastError = "section: no edge between two faces here";
    return nullptr;
  }
  auto loops = sliceAcross(solid, c, at);
  if (loops.empty()) {
    lastError = "section: the cut across this edge is empty";
    return nullptr;
  }
  std::vector<double> points;
  std::vector<int> starts{0};
  for (const auto &loop : loops) {
    for (auto [x, y] : loop) points.insert(points.end(), {x, y});
    starts.push_back((int)(points.size() / 2));
  }
  BKSection *out = new BKSection();
  out->loopCount = (int)loops.size();
  out->pointCount = (int)(points.size() / 2);
  out->points = mallocCopy(points);
  out->loopStart = mallocCopy(starts);
  out->angle = c.angleAt(at);
  auto describe = [&](int f, double *info) {
    V3 n = solid.faces[f].normal, m = solid.faces[f].centroid;
    double v[6] = {n.x, n.y, n.z, m.x, m.y, m.z};
    memcpy(info, v, sizeof v);
  };
  describe(c.fa[at], out->faceA);
  describe(c.fb[at], out->faceB);
  V3 x = -c.ia[at], y = unit(c.na[at] - x * dot(c.na[at], x)), z = cross(x, y), p = c.pts[at];
  double pt[3] = {p.x, p.y, p.z}, dir[3] = {z.x, z.y, z.z};
  memcpy(out->point, pt, sizeof pt);
  memcpy(out->direction, dir, sizeof dir);
  return out;
}

void bk_section_free(BKSection *section) {
  if (!section) return;
  free(section->points);
  free(section->loopStart);
  delete section;
}

int bk_export_step(const BKShape *const *shapes, const char *const *names, int count, const char *path) {
  if (!shapes || count < 0 || !path) return lastError = "STEP: nothing to write", 0;
  std::vector<Shape> list;
  std::vector<std::string> named;
  for (int i = 0; i < count; i++) {
    if (!shapes[i]) return lastError = "STEP: nothing to write", 0;
    list.push_back(shapes[i]->shape);
    named.push_back(names && names[i] ? names[i] : "");
  }
  std::string file = path, text, why;
  if (size_t slash = file.find_last_of('/'); slash != std::string::npos) file.erase(0, slash + 1);
  if (!stepText(list, named, file, text, why)) return lastError = "STEP: " + why, 0;
  FILE *f = fopen(path, "wb");
  bool ok = f && fwrite(text.data(), 1, text.size(), f) == text.size();
  if (f) ok = fclose(f) == 0 && ok;
  if (!ok) return lastError = "STEP: couldn't write the file", 0;
  return 1;
}
