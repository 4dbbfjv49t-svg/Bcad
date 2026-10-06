// Bcad's geometry engine behind its C API (BcadKernel.h): shapes, bolts and nuts, placements, meshes, boxes, measuring,
// merging, splitting, rounding, hollowing, and meshes and STEP files to print and send on.
#include "BcadKernel.h"

#include "Engine/Bolts.hpp"
#include "Engine/Distance.hpp"
#include "Engine/Fasteners.hpp"
#include "Engine/Figure.hpp"
#include "Engine/Model.hpp"
#include "Engine/Print.hpp"
#include "Engine/Sculpt.hpp"
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

BKShape *bk_mesh_shape(const float *positions, int vertexCount, const uint32_t *indices, int triangleCount) {
  if (!positions || !indices || vertexCount < 0 || triangleCount < 0) return lastError = "mesh: no points or triangles", nullptr;
  std::vector<V3> pts((size_t)vertexCount);
  for (size_t i = 0; i < pts.size(); i++) pts[i] = {positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]};
  std::vector<uint32_t> tris(indices, indices + 3 * (size_t)triangleCount);
  std::string why;
  auto m = meshModel(pts, tris, why);
  if (!m) return lastError = "mesh: " + why, nullptr;
  return new BKShape{shapeOf(m)};
}

void bk_figure_defaults(double sex, double *out) {
  if (out) figureDefaults(sex, out);
}

void bk_figure_range(int field, double *out) {
  if (out) figureRange(field, out[0], out[1]);
}

void bk_figure_pose(int pose, double *params) {
  if (params) figurePose(pose, params);
}

int bk_figure_pose_of(const double *params, int count) {
  FigureSpec s;
  std::string why;
  return figureSpec(params, count, s, why) ? figurePoseOf(s.v) : -1;
}

BKShape *bk_figure(const double *params, int count, int draft) {
  FigureSpec spec;
  Shape s;
  std::string why;
  if (!figureSpec(params, count, spec, why) || !figure(spec, draft != 0, s, why)) return lastError = why, nullptr;
  return new BKShape{s};
}

int bk_figure_extent(const double *params, int count, double *out) {
  FigureSpec spec;
  std::string why;
  if (!out || !figureSpec(params, count, spec, why)) return 0;
  V3 size, anchor;
  figureBox(spec, size, anchor);
  for (int a = 0; a < 3; a++) out[a] = size[a], out[3 + a] = anchor[a];
  return 1;
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

BKSculptMesh *bk_remesh(const BKShape *s, const double *m, double detail) {
  if (!s) return nullptr;
  if (!m || !finite(m, 12) || !std::isfinite(detail)) return lastError = "remesh: sizes must be numbers", nullptr;
  Affine a = Affine::from(m);
  if (std::fabs(a.det()) <= 1e-12) return lastError = "remesh: placement flattens the shape", nullptr;
  // The shape's mesh well within the detail (a curve's chords lie inside it, and would come out a little small).
  Solid in;
  mesh({s->shape.node, s->shape.place.then(a)}, std::min(0.05, std::max(0.002, detail / 50)), in);
  std::vector<V3> pts;
  std::vector<uint32_t> tris;
  std::string why;
  if (!remesh(in.p, in.tri, detail, pts, tris, why)) return lastError = "remesh: " + why, nullptr;
  BKSculptMesh *out = new BKSculptMesh();
  out->vertexCount = (int)pts.size(), out->triangleCount = (int)tris.size() / 3;
  std::vector<float> pos(3 * pts.size());
  for (size_t i = 0; i < pts.size(); i++) pos[3 * i] = (float)pts[i].x, pos[3 * i + 1] = (float)pts[i].y, pos[3 * i + 2] = (float)pts[i].z;
  out->positions = mallocCopy(pos);
  out->indices = mallocCopy(tris);
  return out;
}

BKSculptMesh *bk_mesh_body(const BKShape *s, const double *m) {
  if (!s) return nullptr;
  if (!m || !finite(m, 12)) return lastError = "mesh body: sizes must be numbers", nullptr;
  const Node *n = s->shape.node.get();
  if (!n || n->kind != Node::Prim || !n->model || n->model->kind != Model::Mesh || !n->model->mesh) return lastError = "mesh body: not one as it is", nullptr;
  Affine a = s->shape.place.then(Affine::from(m));
  double det = a.det();
  if (!(std::fabs(det) > 1e-12)) return lastError = "mesh body: placement flattens the shape", nullptr;
  const Solid &body = *n->model->mesh;
  BKSculptMesh *out = new BKSculptMesh();
  out->vertexCount = (int)body.p.size(), out->triangleCount = (int)body.tri.size() / 3;
  std::vector<float> pos(3 * body.p.size());
  for (size_t i = 0; i < body.p.size(); i++) {
    V3 q = a.point(body.p[i]);
    pos[3 * i] = (float)q.x, pos[3 * i + 1] = (float)q.y, pos[3 * i + 2] = (float)q.z;
  }
  std::vector<uint32_t> tris = body.tri;
  // (Mirrored: each triangle turned round, to face out still.)
  if (det < 0)
    for (size_t t = 0; t + 2 < tris.size(); t += 3) std::swap(tris[t + 1], tris[t + 2]);
  out->positions = mallocCopy(pos);
  out->indices = mallocCopy(tris);
  return out;
}

void bk_sculpt_mesh_free(BKSculptMesh *m) {
  if (!m) return;
  free(m->positions), free(m->indices);
  delete m;
}

struct BKSculpt {
  Sculptor sculptor;
  // As floats, per slot (a free triangle slot's corners all 0); what the last sync found changed.
  std::vector<float> positions, normals;
  std::vector<uint32_t> indices, changed, changedTris;
  BKSculpt(std::vector<V3> p, std::vector<uint32_t> t) : sculptor(std::move(p), std::move(t)) {
    if (!sculptor.ok()) return;
    for (uint32_t i = 0; i < sculptor.points().size(); i++) put(i);
    for (uint32_t i = 0; i < sculptor.triangles().size() / 3; i++) putTriangle(i);
  }
  void put(uint32_t i) {
    if (positions.size() < 3 * sculptor.points().size()) positions.resize(3 * sculptor.points().size()), normals.resize(positions.size());
    V3 a = sculptor.points()[i], b = sculptor.normals()[i];
    float *q = &positions[3 * i], *m = &normals[3 * i];
    q[0] = (float)a.x, q[1] = (float)a.y, q[2] = (float)a.z;
    m[0] = (float)b.x, m[1] = (float)b.y, m[2] = (float)b.z;
  }
  void putTriangle(uint32_t t) {
    if (indices.size() < sculptor.triangles().size()) indices.resize(sculptor.triangles().size());
    for (int k = 0; k < 3; k++) indices[3 * t + k] = sculptor.triangleAlive(t) ? sculptor.triangles()[3 * t + k] : 0;
  }
};

BKSculpt *bk_sculpt_new(const float *positions, int vertexCount, const uint32_t *indices, int triangleCount) {
  if (!positions || !indices || vertexCount <= 0 || triangleCount <= 0) return lastError = "sculpt: no points or triangles", nullptr;
  std::vector<V3> p((size_t)vertexCount);
  for (size_t i = 0; i < p.size(); i++) {
    p[i] = {positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]};
    if (!std::isfinite(p[i].x) || !std::isfinite(p[i].y) || !std::isfinite(p[i].z)) return lastError = "sculpt: points must be numbers", nullptr;
  }
  std::vector<uint32_t> t(indices, indices + 3 * (size_t)triangleCount);
  for (uint32_t v : t)
    if (v >= (uint32_t)vertexCount) return lastError = "sculpt: a triangle's corner is missing", nullptr;
  BKSculpt *s = new BKSculpt(std::move(p), std::move(t));
  if (!s->sculptor.ok()) {
    delete s;
    return lastError = "sculpt: the mesh isn't closed", nullptr;
  }
  return s;
}

void bk_sculpt_free(BKSculpt *s) { delete s; }

int bk_sculpt_ray(const BKSculpt *s, const double *origin, const double *direction, double *at, double *normal) {
  if (!s || !origin || !direction || !finite(origin, 3) || !finite(direction, 3)) return 0;
  V3 a, n;
  if (!s->sculptor.ray({origin[0], origin[1], origin[2]}, {direction[0], direction[1], direction[2]}, a, n)) return 0;
  if (at) at[0] = a.x, at[1] = a.y, at[2] = a.z;
  if (normal) normal[0] = n.x, normal[1] = n.y, normal[2] = n.z;
  return 1;
}

void bk_sculpt_set_detail(BKSculpt *s, double size) {
  if (s) s->sculptor.setDetail(std::isfinite(size) ? size : 0);
}

void bk_sculpt_begin(BKSculpt *s, int brush, const double *at, double radius, double strength, int mirror, int invert) {
  if (!s || !at || !finite(at, 3) || !std::isfinite(radius) || !std::isfinite(strength)) return;
  s->sculptor.begin(brush, {at[0], at[1], at[2]}, radius, strength, mirror != 0 ? BK_MIRROR_X : 0, invert != 0);
}

void bk_sculpt_begin_brush(BKSculpt *s, const BKBrush *b, const double *at) {
  if (!s || !b || !at || !finite(at, 3) || !std::isfinite(b->radius) || !std::isfinite(b->strength)) return;
  BrushTip tip;
  tip.hardness = b->hardness, tip.rigidity = b->rigidity, tip.oval = b->oval, tip.angle = b->angle, tip.tilt = b->tilt;
  s->sculptor.begin(b->brush, {at[0], at[1], at[2]}, b->radius, b->strength, b->mirror, b->invert != 0, tip, {b->across[0], b->across[1], b->across[2]});
}

void bk_sculpt_dab(BKSculpt *s, const double *at, double pressure, double size) {
  if (!s || !at || !finite(at, 3) || !std::isfinite(pressure) || !std::isfinite(size)) return;
  s->sculptor.dab({at[0], at[1], at[2]}, pressure, size);
}

void bk_sculpt_dab_tilted(BKSculpt *s, const double *at, double pressure, double size, double tilt) {
  if (!s || !at || !finite(at, 3) || !std::isfinite(pressure) || !std::isfinite(size)) return;
  s->sculptor.dab({at[0], at[1], at[2]}, pressure, size, tilt);
}

void bk_sculpt_end(BKSculpt *s) {
  if (s) s->sculptor.end();
}
int bk_sculpt_undo(BKSculpt *s) { return s && s->sculptor.undo() ? 1 : 0; }
int bk_sculpt_redo(BKSculpt *s) { return s && s->sculptor.redo() ? 1 : 0; }

int bk_sculpt_sync(BKSculpt *s) {
  if (!s) return 0;
  s->changed = s->sculptor.takeChanged();
  for (uint32_t i : s->changed) s->put(i);
  s->changedTris = s->sculptor.takeChangedTriangles();
  for (uint32_t t : s->changedTris) s->putTriangle(t);
  return (int)s->changed.size();
}

const uint32_t *bk_sculpt_changed(const BKSculpt *s) { return s ? s->changed.data() : nullptr; }
int bk_sculpt_changed_triangle_count(const BKSculpt *s) { return s ? (int)s->changedTris.size() : 0; }
const uint32_t *bk_sculpt_changed_triangles(const BKSculpt *s) { return s ? s->changedTris.data() : nullptr; }
int bk_sculpt_vertex_count(const BKSculpt *s) { return s ? (int)s->positions.size() / 3 : 0; }
int bk_sculpt_triangle_count(const BKSculpt *s) { return s ? (int)s->indices.size() / 3 : 0; }
int bk_sculpt_live_triangle_count(const BKSculpt *s) { return s ? (int)s->sculptor.liveTriangles() : 0; }
const float *bk_sculpt_positions(const BKSculpt *s) { return s ? s->positions.data() : nullptr; }
const float *bk_sculpt_normals(const BKSculpt *s) { return s ? s->normals.data() : nullptr; }
const uint32_t *bk_sculpt_indices(const BKSculpt *s) { return s ? s->indices.data() : nullptr; }

const char *bk_sculpt_check(const BKSculpt *s) {
  static thread_local std::string why;
  why = s ? s->sculptor.check() : "no sculpt";
  return why.c_str();
}

BKSculptMesh *bk_sculpt_mesh(const BKSculpt *s) {
  if (!s) return nullptr;
  std::vector<V3> pts;
  std::vector<uint32_t> tris;
  s->sculptor.compact(pts, tris);
  BKSculptMesh *out = new BKSculptMesh();
  out->vertexCount = (int)pts.size(), out->triangleCount = (int)tris.size() / 3;
  std::vector<float> pos(3 * pts.size());
  for (size_t i = 0; i < pts.size(); i++) pos[3 * i] = (float)pts[i].x, pos[3 * i + 1] = (float)pts[i].y, pos[3 * i + 2] = (float)pts[i].z;
  out->positions = mallocCopy(pos);
  out->indices = mallocCopy(tris);
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
  // Without openings, walls at least half as thick as the body is at its narrowest leave nothing inside: refused at once
  // (working them out would take minutes).
  if (openCount == 0) {
    double thinnest = thickness;
    for (int k = 0; k < wallCount; k++) thinnest = std::min(thinnest, wallThickness[k]);
    V3 lo, hi;
    placedBounds(s->shape, lo, hi);
    double narrowest = std::min({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z});
    if (!(thinnest < narrowest / 2)) {
      lastError = "hollow: the walls don't fit this shape";
      return nullptr;
    }
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
  // A big mesh body (a sculpt, a figure: over 100,000 triangles) straight on the grid: walls offset from each of its
  // triangles took many seconds (a fine sphere's 500k, eleven), and the grid's void prints the same.
  const Node &n = *s->shape.node;
  if (n.kind == Node::Prim && n.model->kind == Model::Mesh && n.model->mesh && n.model->mesh->tri.size() > 300000 && h.open.empty() && h.walls.empty())
    h.grid = true;
  bool ok = hollowed(s->shape, h, d, made, &miss);
  for (int k = 0; k < sharpCount && !ok; k++) {
    if (!sharp || !sharp[k]) continue;
    Hollowing via = h;
    via.viaSharp = true, via.sharp = sharp[k]->shape;
    if ((ok = hollowed(s->shape, via, d, made))) h = via;
  }
  // Walls that can't be offset all round (a body thinner than twice them in places, as a sculpted one often is): the void
  // found on a grid instead, where no face is to be opened or have a wall of its own.
  if (!ok && h.open.empty() && h.walls.empty()) {
    Hollowing via = h;
    via.grid = true;
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
  // Meshed finely enough to show the rounding's arc, but not finer than a fifty-thousandth of the body (a small rounding
  // on a body metres across once took tens of millions of triangles).
  double d = std::max(radius > 0 ? radius / 400 : 0.05, 1e-4);
  V3 lo, hi;
  placedBounds(s->shape, lo, hi);
  double size = std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z});
  if (std::isfinite(size) && size > 0) d = std::max(d, size * 2e-5);
  Solid solid;
  mesh(s->shape, d, solid);
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
  if (!shapes || count < 0 || !path) return lastError = "file: nothing to write", 0;
  std::vector<Shape> list;
  std::vector<std::string> named;
  for (int i = 0; i < count; i++) {
    if (!shapes[i]) return lastError = "file: nothing to write", 0;
    list.push_back(shapes[i]->shape);
    named.push_back(names && names[i] ? names[i] : "");
  }
  std::string file = path, text, why;
  size_t which = 0;
  if (size_t slash = file.find_last_of('/'); slash != std::string::npos) file.erase(0, slash + 1);
  if (!stepText(list, named, file, text, why, &which)) return lastError = "file " + std::to_string(which) + ": " + why, 0;
  FILE *f = fopen(path, "wb");
  bool ok = f && fwrite(text.data(), 1, text.size(), f) == text.size();
  if (f) ok = fclose(f) == 0 && ok;
  if (!ok) return lastError = "file: couldn't write it", 0;
  return 1;
}

BKPrintMesh *bk_print_mesh(const BKShape *s) {
  if (!s) return nullptr;
  PrintMesh pm;
  std::string why;
  bool sound = printMesh(s->shape, pm, why);
  if (!sound) lastError = "file 0: " + why;
  BKPrintMesh *m = new BKPrintMesh();
  m->vertexCount = (int)(pm.pos.size() / 3);
  m->triangleCount = (int)(pm.tri.size() / 3);
  m->positions = mallocCopy(pm.pos);
  m->indices = mallocCopy(pm.tri);
  m->volume = pm.volume;
  m->crowded = pm.crowded;
  m->slivers = pm.slivers;
  m->valid = sound ? 1 : 0;
  return m;
}

void bk_print_mesh_free(BKPrintMesh *m) {
  if (!m) return;
  free(m->positions);
  free(m->indices);
  delete m;
}
