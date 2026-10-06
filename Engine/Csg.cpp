// Shapes as a tree: a primitive's mesh made from its model, a merge's or a split's from its parts' meshes (made at the same
// detail, and kept for when they're asked again); what's found again on a merged or split mesh (edges a hair long taken
// out, faces on one surface joined, edges, corners, circles); and how far a shape reaches and how much it holds — exactly
// where that can be told.
#include "Engine/Model.hpp"
#include "Engine/Treat.hpp"
#include "Engine/Weld.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <unordered_map>

namespace bce {

// MARK: - evaluating

namespace {

// A turned primitive among a merge's parts: its axis line (a pointing the way fixed for that line, whichever way the part
// was turned), the deflection it's meshed at, and its steps round the axis.
struct Lathe {
  bool ok = false;
  V3 o, a;
  double d = 0;
  int count = 0;
};

Lathe latheOf(const Shape &s, double d) {
  Lathe l;
  if (!s.node || s.node->kind != Node::Prim || !s.node->model || s.node->model->kind != Model::Turned || !s.place.similarity()) return l;
  l.d = d / s.place.stretch();
  l.count = s.node->model->columns(l.d);
  l.o = s.place.point({0, 0, 0});
  V3 a = unit(s.place.vector({0, 0, 1}));
  double ax = std::fabs(a.x), ay = std::fabs(a.y), az = std::fabs(a.z);
  double big = ax >= ay && ax >= az ? a.x : ay >= az ? a.y : a.z;
  l.a = big < 0 ? -a : a;
  l.ok = true;
  return l;
}

bool oneLine(const Lathe &p, const Lathe &q) {
  double size = 1 + std::max({std::fabs(p.o.x), std::fabs(p.o.y), std::fabs(p.o.z)});
  return p.ok && q.ok && norm(cross(p.a, q.a)) <= 1e-9 && norm(cross(q.o - p.o, p.a)) <= 1e-9 * size;
}

// A frame made from the axis line `line` alone (the part's axis along it one way or the other, its first step along a
// direction fixed for the line, its origin on the line): turning a turned shape about its own axis, or mirroring it in a
// plane through that axis, leaves it as it is, so this places the same shape however its part was turned — and two
// parts' surfaces on one line mesh point for point, and where they meet face to face they cancel exactly.
Affine frameOn(const Shape &s, const Lathe &l, const Lathe &line) {
  V3 z = s.place.vector({0, 0, 1});
  double scale = norm(z);
  V3 a = dot(z, line.a) > 0 ? line.a : -line.a;
  // (Along x as near as square to the line: an upright part not turned keeps its own frame.)
  V3 ref = std::fabs(line.a.x) < 0.9 ? V3{1, 0, 0} : V3{0, 1, 0};
  V3 x = unit(ref - line.a * dot(ref, line.a)), y = cross(a, x);
  // (A hair off the line where the parts' placements round differently.)
  V3 o = line.o + line.a * dot(l.o - line.o, line.a);
  Affine f;
  f.m[0] = x.x * scale, f.m[1] = y.x * scale, f.m[2] = a.x * scale, f.m[3] = o.x;
  f.m[4] = x.y * scale, f.m[5] = y.y * scale, f.m[6] = a.y * scale, f.m[7] = o.y;
  f.m[8] = x.z * scale, f.m[9] = y.z * scale, f.m[10] = a.z * scale, f.m[11] = o.z;
  return f;
}

// Where a turned part's rims lie along the line (its profile's corners off the axis), as heights in the frame `to`.
std::vector<double> rimsIn(const Shape &s, const Affine &from, const Affine &to) {
  std::vector<double> out;
  Affine back = to.inverse();
  for (const Elem &e : s.node->model->profile)
    if (e.r0 > 0) out.push_back(back.point(from.point({0, 0, e.z0})).z);
  return out;
}

// Its mesh, `count` steps round, rings at `levels` too, in the frame f.
void lathe(const Shape &s, const Lathe &l, const Affine &f, int count, const std::vector<double> &levels, Solid &out) {
  Model m = *s.node->model;
  m.around = count;
  m.levels = levels;
  out = Solid();
  m.build(out, l.d);
  out.centroids();
  out.slivers();
  out.transform(f);
}

}  // namespace

std::shared_ptr<const Solid> evaluate(const Node &node, double d) {
  // A mesh body is its mesh, at any detail: given as it is, no copy kept.
  if (node.kind == Node::Prim && node.model->kind == Model::Mesh) return node.model->mesh;
  {
    std::lock_guard<std::mutex> hold(node.lock);
    for (const auto &m : node.made)
      if (std::fabs(m.first - d) <= 1e-12 * d) return m.second;
  }
  auto out = std::make_shared<Solid>();
  if (node.kind == Node::Prim) {
    node.model->build(*out, d);
    out->centroids();
    out->slivers();
  } else if (node.kind == Node::Treat) {
    Solid a;
    mesh(node.a, d, a);
    TreatFit fit;
    *out = treated(a, *node.treat, d, fit);
    // Made once already (it fitted); at another detail failing after all, as it was made then (not the shape untreated:
    // what's saved must be what was shown).
    if (!fit.fits && node.shown) *out = *node.shown;
  } else if (node.kind == Node::Hollow) {
    // Likewise; with nothing made before, the shape as it is.
    if (!hollowed(node.a, *node.hollow, d, *out)) {
      if (node.shown) *out = *node.shown;
      else mesh(node.a, d, *out);
    }
  } else if (node.kind == Node::Bool) {
    Solid a, b;
    // Turned parts on one axis line meshed alike: the finer count for both, each with rings where the other ends.
    Lathe la = latheOf(node.a, d), lb = latheOf(node.b, d);
    bool one = oneLine(la, lb);
    Affine fa, fb;
    std::vector<double> atA, atB;
    if (la.ok) fa = frameOn(node.a, la, la);
    if (lb.ok) fb = frameOn(node.b, lb, one ? la : lb);
    if (one) {
      la.count = lb.count = std::max(la.count, lb.count);
      atA = rimsIn(node.b, fb, fa), atB = rimsIn(node.a, fa, fb);
    }
    if (la.ok) lathe(node.a, la, fa, la.count, atA, a);
    else mesh(node.a, d, a);
    if (lb.ok) lathe(node.b, lb, fb, lb.count, atB, b);
    else mesh(node.b, d, b);
    // A mesh body as it is was checked not to cross itself when it was made: its triangles needn't be looked at against
    // each other again (should moving or rounding have made two cross, the merge finds it and looks at every pair).
    auto apart = [](const Shape &s, Solid &m) {
      if (s.node->kind == Node::Prim && s.node->model->kind == Model::Mesh) m.sound.assign(m.tri.size() / 3, 1), m.grid = -1;
    };
    apart(node.a, a), apart(node.b, b);
    *out = combine(a, b, node.op, 1e-11, true);
    finish(*out, d);
  } else {
    Solid a;
    mesh(node.a, d, a);
    *out = cut(a, node.p, node.n, node.side);
    finish(*out, d);
  }
  std::lock_guard<std::mutex> hold(node.lock);
  node.made.push_back({d, out});
  if (node.made.size() > 4) node.made.erase(node.made.begin());
  return out;
}

void mesh(const Shape &s, double deflection, Solid &out) {
  double grow = s.place.stretch();
  out = *evaluate(*s.node, grow > 0 ? deflection / grow : deflection);
  out.transform(s.place);
}

// MARK: - inside

namespace {

double winding(const Solid &s, V3 q) {
  double sum = 0;
  for (size_t k = 0; k < s.tri.size(); k += 3) {
    V3 a = s.p[s.tri[k]] - q, b = s.p[s.tri[k + 1]] - q, c = s.p[s.tri[k + 2]] - q;
    double la = norm(a), lb = norm(b), lc = norm(c);
    double det = dot(a, cross(b, c)), div = la * lb * lc + dot(a, b) * lc + dot(a, c) * lb + dot(b, c) * la;
    sum += 2 * trig::atan2(det, div);
  }
  return sum / (4 * M_PI);
}

}  // namespace

bool inside(const Solid &s, V3 q) { return winding(s, q) > 0.5; }

// MARK: - pieces

int pieces(const Solid &s) {
  Welded w = weld(s);
  size_t nt = w.count();
  std::vector<uint32_t> parent(nt);
  std::iota(parent.begin(), parent.end(), 0);
  auto find = [&](uint32_t x) {
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
  };
  // Triangles sharing a point are one piece: shapes touching anywhere (along a face, an edge or at a single point) hold
  // together as one.
  std::vector<uint32_t> firstAt(w.pts.size(), UINT32_MAX);
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) {
      uint32_t &f = firstAt[w.tri[3 * t + k]];
      if (f == UINT32_MAX) f = (uint32_t)t;
      else parent[find((uint32_t)t)] = find(f);
    }
  // Each shell's volume about the mesh's middle, against its size (not how far it sits from the origin, which would make a
  // small part far off count as nothing).
  std::vector<double> vol(nt, 0);
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (const auto &p : w.pts) lo = vmin(lo, p), hi = vmax(hi, p);
  V3 mid = (lo + hi) * 0.5, ext = hi - lo;
  double size = std::max({ext.x, ext.y, ext.z, 0.0});
  for (size_t t = 0; t < nt; t++) {
    V3 a = w.pts[w.tri[3 * t]] - mid, b = w.pts[w.tri[3 * t + 1]] - mid, c = w.pts[w.tri[3 * t + 2]] - mid;
    vol[find((uint32_t)t)] += dot(a, cross(b, c)) / 6;
  }
  int n = 0;
  for (size_t t = 0; t < nt; t++)
    if (find((uint32_t)t) == t && vol[t] > 1e-9 * (1 + size * size * size)) n++;
  return n;
}

// MARK: - finishing a merged or split mesh

namespace {

constexpr double pi = M_PI;

// A turned face's surface in the solid's frame, whatever part of it the face is and however its part was placed: a
// cylinder (axis through o along a, radius r), a cone (apex o, a the way it widens, k its widening per length), a ball
// (centre o, radius r) or a ring (centre o, axis a, radius r round the axis, k the tube's).
struct Form {
  enum Kind { None, Cylinder, Cone, Ball, Ring } kind = None;
  V3 o, a;
  double r = 0, k = 0;
};

Form formOf(const FaceGeom &g) {
  Form f;
  double s;
  if (g.kind != FaceGeom::Turned || !g.exact || g.flat || !g.place.similarity(&s) || !(s > 0)) return f;
  const Elem &e = g.elem;
  V3 axis = unit(g.place.vector({0, 0, 1}));
  if (!e.arc) {
    if (e.onAxis() || e.z0 == e.z1) return f;
    double dr = e.r1 - e.r0, dz = e.z1 - e.z0;
    if (std::fabs(dr) <= 1e-12 * (std::fabs(e.r0) + std::fabs(e.r1) + std::fabs(dz))) {
      f.kind = Form::Cylinder, f.o = g.place.point({0, 0, 0}), f.a = axis, f.r = s * 0.5 * (e.r0 + e.r1);
    } else {
      double k = dr / dz;
      f.kind = Form::Cone, f.o = g.place.point({0, 0, e.z0 - e.r0 / k}), f.a = k > 0 ? axis : -axis, f.k = std::fabs(k);
    }
  } else {
    f.o = g.place.point({0, 0, e.cz}), f.a = axis, f.r = s * e.rad;
    if (std::fabs(e.cr) <= 1e-12 * e.rad) {
      f.kind = Form::Ball;
    } else {
      f.kind = Form::Ring, f.k = f.r, f.r = s * e.cr;
    }
  }
  return f;
}

bool sameSurface(const FaceGeom &a, const FaceGeom &b) {
  if (a.flat && b.flat) return norm(a.pn - b.pn) < 1e-9 && std::fabs(a.pd - b.pd) < 1e-9 * (1 + std::fabs(a.pd));
  Form p = formOf(a), q = formOf(b);
  if (p.kind == Form::None || p.kind != q.kind) return false;
  double size = 1 + std::max({std::fabs(p.o.x), std::fabs(p.o.y), std::fabs(p.o.z), p.r, p.k});
  double tol = 1e-9 * size;
  auto near = [&](double x, double y) { return std::fabs(x - y) <= tol; };
  bool along = norm(cross(p.a, q.a)) <= 1e-9;
  switch (p.kind) {
  case Form::Cylinder: return along && near(p.r, q.r) && norm(cross(q.o - p.o, p.a)) <= tol;
  case Form::Cone: return along && dot(p.a, q.a) > 0 && std::fabs(p.k - q.k) <= 1e-9 * (1 + p.k) && norm(q.o - p.o) <= tol;
  case Form::Ball: return near(p.r, q.r) && norm(q.o - p.o) <= tol;
  case Form::Ring: return along && near(p.r, q.r) && near(p.k, q.k) && norm(q.o - p.o) <= tol;
  default: return false;
  }
}

// The face `g` widened to take in `o` on the same surface: its profile piece run on, in its own frame, over every point
// of o's (so whatever is found on the joined face — its rims, its normals — lies within it).
void spanBoth(FaceGeom &g, const FaceGeom &o) {
  if (g.kind != FaceGeom::Turned || o.kind != FaceGeom::Turned || g.flat || o.flat) return;
  Affine back = g.place.inverse();
  Elem &e = g.elem;
  std::vector<V3> at;
  for (int i = 0; i <= 8; i++) {
    double r, z;
    o.elem.at(i / 8.0, r, z);
    at.push_back(back.point(o.place.point({r, 0, z})));
  }
  if (!e.arc) {
    double lo = std::min(e.z0, e.z1), hi = std::max(e.z0, e.z1);
    for (V3 p : at) lo = std::min(lo, p.z), hi = std::max(hi, p.z);
    double k = (e.r1 - e.r0) / (e.z1 - e.z0);
    double z0 = e.z0 < e.z1 ? lo : hi, z1 = e.z0 < e.z1 ? hi : lo;
    e = Elem::line(std::max(0.0, e.r0 + (z0 - e.z0) * k), z0, std::max(0.0, e.r0 + (z1 - e.z0) * k), z1);
  } else {
    // Angles measured from the middle of g's arc, the way it runs.
    double dir = e.a1 >= e.a0 ? 1 : -1, mid = 0.5 * (e.a0 + e.a1), lo = (e.a0 - mid) * dir, hi = (e.a1 - mid) * dir;
    for (V3 p : at) {
      double t = trig::atan2(p.z - e.cz, trig::hypot(p.x, p.y) - e.cr) - mid;
      t = std::remainder(t * dir, 2 * pi);
      lo = std::min(lo, t), hi = std::max(hi, t);
    }
    if (hi - lo >= 2 * pi - 1e-9) lo = -pi, hi = pi;
    e = Elem::arcOf(e.cr, e.cz, e.rad, mid + dir * lo, mid + dir * hi);
  }
}

// A frame with z along `axis` at `origin`.
Affine frameAt(V3 origin, V3 axis) {
  V3 z = unit(axis), x = unit(std::fabs(z.x) < 0.9 ? cross(z, V3{1, 0, 0}) : cross(z, V3{0, 1, 0})), y = cross(z, x);
  Affine f;
  f.m[0] = x.x, f.m[1] = y.x, f.m[2] = z.x, f.m[3] = origin.x;
  f.m[4] = x.y, f.m[5] = y.y, f.m[6] = z.y, f.m[7] = origin.y;
  f.m[8] = x.z, f.m[9] = y.z, f.m[10] = z.z, f.m[11] = origin.z;
  return f;
}

// A line of points that is (part of) a circle, known exactly from the face it lies on: a turned face cut at one height
// along its axis (radius from the face's profile there), or a sphere cut by any plane.
struct Ring {
  EdgeGeom geom;
  V3 centre, axis;
  double radius, turn;  // turn: how far round the line goes (radians)
};

bool circleOn(const FaceGeom &g, const std::vector<V3> &pts, double deflection, Ring &out) {
  if (g.kind != FaceGeom::Turned || !g.exact || g.flat || pts.size() < 3) return false;
  double scale = 1;
  g.place.similarity(&scale);
  Affine back = g.place.inverse();
  std::vector<V3> q(pts.size());
  double size = 0, z = 0;
  for (size_t i = 0; i < pts.size(); i++) {
    q[i] = back.point(pts[i]);
    size = std::max({size, std::fabs(q[i].x), std::fabs(q[i].y), std::fabs(q[i].z)});
    z += q[i].z;
  }
  z /= q.size();
  double tol = 1e-9 * (1 + size);
  bool level = true;
  for (V3 v : q) level = level && std::fabs(v.z - z) <= tol;
  const Elem &el = g.elem;
  // In the face's own frame: the circle's middle and axis, the line's points about it, and the radius.
  V3 mid, ax{0, 0, 1};
  double r;
  // How far in from the circle the mesh's points may lie, per unit of the surface's chord error: the surface leans across
  // the circle's plane, a point that far under it lies farther in along the plane (1 / the radial part of its normal).
  double lean = 1;
  if (level) {
    mid = {0, 0, z};
    if (!el.arc) {
      if (el.z0 == el.z1) return false;
      double t = (z - el.z0) / (el.z1 - el.z0);
      if (t < -1e-9 || t > 1 + 1e-9) return false;
      r = el.r0 + std::clamp(t, 0.0, 1.0) * (el.r1 - el.r0);
      lean = trig::hypot(el.r1 - el.r0, el.z1 - el.z0) / std::fabs(el.z1 - el.z0);
    } else {
      double rho = 0;
      for (V3 v : q) rho += trig::hypot(v.x, v.y);
      rho /= q.size();
      double a = trig::asin(std::clamp((z - el.cz) / el.rad, -1.0, 1.0));
      double r1 = el.cr + el.rad * trig::cos(a), r2 = el.cr - el.rad * trig::cos(a);
      r = std::fabs(r1 - rho) <= std::fabs(r2 - rho) ? r1 : r2;
      lean = 1 / std::max(trig::cos(a), 1e-9);
    }
  } else {
    // A sphere: any flat line on it is a circle.
    if (!el.arc || el.cr != 0) return false;
    V3 c, n;
    for (V3 v : q) c += v;
    c = c / (double)q.size();
    for (size_t i = 0; i + 1 < q.size(); i++) n += cross(q[i] - c, q[i + 1] - c);
    if (norm(n) <= 0) return false;
    ax = unit(n);
    for (V3 v : q)
      if (std::fabs(dot(v - c, ax)) > tol) return false;
    V3 ball{0, 0, el.cz};
    double h = dot(ball - c, ax);
    mid = ball - ax * h;
    if (el.rad * el.rad - h * h <= 0) return false;
    r = std::sqrt(el.rad * el.rad - h * h);
    lean = el.rad / r;
  }
  // Not so steep that points a chord's error under the surface could be anywhere.
  lean = std::min(lean, 8.0);
  // The mesh's points lie on chords inside the circle, by no more than the chord error.
  double turn = 0;
  V3 e1 = unit(std::fabs(ax.x) < 0.9 ? cross(ax, V3{1, 0, 0}) : cross(ax, V3{0, 1, 0})), e2 = cross(ax, e1);
  for (size_t i = 0; i < q.size(); i++) {
    double rho = norm((q[i] - mid) - ax * dot(q[i] - mid, ax));
    if (r <= tol || rho > r + tol || rho < r - 2 * lean * deflection / scale - tol) return false;
    if (i == 0) continue;
    V3 u = q[i - 1] - mid, v = q[i] - mid;
    double d = trig::atan2(dot(v, e2), dot(v, e1)) - trig::atan2(dot(u, e2), dot(u, e1));
    turn += d > M_PI ? d - 2 * M_PI : d < -M_PI ? d + 2 * M_PI : d;
  }
  out.turn = turn;
  out.centre = g.place.point(mid), out.axis = unit(g.place.vector(ax)), out.radius = r * scale;
  out.geom.kind = EdgeGeom::Circle;
  if (level) {
    out.geom.place = g.place, out.geom.r = r, out.geom.z = z;
  } else {
    out.geom.place = frameAt(out.centre, out.axis), out.geom.r = out.radius, out.geom.z = 0;
  }
  return true;
}

}  // namespace

// Edges shorter than `eps` (a cut a hair from a corner) and triangles thinner (a point a hair off the line between two
// others, where a cut grazes a side) done away with, so the mesh stays clean when it's rounded or handed on in floats.
// Every step keeps each side met by one running the other way, so a closed mesh stays closed whatever it looks like, and
// none is ever refused: points nearer than twice `eps` across an edge are made one (the one on more faces kept: a corner
// before a point inside a face), triangles left with two corners in one place go (their other two sides cancel), as do
// two triangles on the same three points facing opposite ways; a corner nearer than `eps` to its triangle's long side
// is put on that side, every triangle along the side split there and the thin one gone.
void clean(Welded &w, double eps, const std::vector<char> *only) {
  bool gaps = !w.gap.empty();
  // (Triangles changed are no longer known not to cross others.)
  bool marks = w.sound.size() == w.count();
  // The triangles that may need it: those given (the rest known clean), and any changed on the way.
  std::vector<char> fresh = only && only->size() == w.count() ? *only : std::vector<char>(w.count(), 1);
  // Points are made one a little farther apart than a triangle counts as thin, so a thin one's corner off its long side is
  // never so near either end that putting it on the side leaves another thin one the other way round.
  double near = 2 * eps;
  // Triangles out (by mark), the rest kept in order with all they carry.
  auto compact = [&](const std::vector<char> &dead) {
    size_t n = 0, all = w.count();
    for (size_t t = 0; t < all; t++) {
      if (dead[t]) continue;
      for (int k = 0; k < 3; k++) w.tri[3 * n + k] = w.tri[3 * t + k], w.nrm[3 * n + k] = w.nrm[3 * t + k];
      if (gaps)
        for (int k = 0; k < 6; k++) w.gap[6 * n + k] = w.gap[6 * t + k];
      if (marks) w.sound[n] = w.sound[t];
      fresh[n] = fresh[t];
      w.face[n++] = w.face[t];
    }
    w.tri.resize(3 * n), w.nrm.resize(3 * n), w.face.resize(n), fresh.resize(n);
    if (gaps) w.gap.resize(6 * n);
    if (marks) w.sound.resize(n);
  };
  // (Rounds while that leaves fewer thin triangles; a few on a tiny patch can go round in a circle, and are left.)
  size_t fewest = SIZE_MAX;
  int stale = 0;
  for (int round = 0; round < 64 && stale < 3; round++) {
    bool changed = false;
    size_t nt = w.count();
    // Points made one, nearest first.
    {
      struct Short {
        double l2;
        uint32_t a, b;
      };
      std::vector<Short> shorts;
      for (size_t t = 0; t < nt; t++) {
        if (!fresh[t]) continue;
        for (int k = 0; k < 3; k++) {
          uint32_t a = w.tri[3 * t + k], b = w.tri[3 * t + (k + 1) % 3];
          double l2 = norm2(w.pts[a] - w.pts[b]);
          if (l2 < near * near) shorts.push_back({l2, std::min(a, b), std::max(a, b)});
        }
      }
      if (!shorts.empty()) {
        std::sort(shorts.begin(), shorts.end(), [](const Short &x, const Short &y) {
          return x.l2 != y.l2 ? x.l2 < y.l2 : x.a != y.a ? x.a < y.a : x.b < y.b;
        });
        shorts.erase(std::unique(shorts.begin(), shorts.end(), [](const Short &x, const Short &y) { return x.a == y.a && x.b == y.b; }), shorts.end());
        // Faces at each point of a short edge.
        std::vector<int> slot(w.pts.size(), -1);
        std::vector<std::vector<uint32_t>> faces;
        for (const Short &e : shorts)
          for (uint32_t v : {e.a, e.b})
            if (slot[v] < 0) slot[v] = (int)faces.size(), faces.emplace_back();
        for (size_t t = 0; t < nt; t++)
          for (int k = 0; k < 3; k++) {
            int at = slot[w.tri[3 * t + k]];
            if (at < 0) continue;
            auto &f = faces[at];
            if (std::find(f.begin(), f.end(), w.face[t]) == f.end()) f.push_back(w.face[t]);
          }
        std::vector<uint32_t> to(w.pts.size());
        std::iota(to.begin(), to.end(), 0);
        auto now = [&](uint32_t v) {
          while (to[v] != v) v = to[v] = to[to[v]];
          return v;
        };
        for (const Short &e : shorts) {
          uint32_t a = now(e.a), b = now(e.b);
          if (a == b || norm2(w.pts[a] - w.pts[b]) >= near * near) continue;
          auto &fa = faces[slot[a]], &fb = faces[slot[b]];
          uint32_t keep = a, gone = b;
          if (fb.size() > fa.size() || (fb.size() == fa.size() && b < a)) std::swap(keep, gone);
          to[gone] = keep;
          auto &fk = faces[slot[keep]];
          for (uint32_t f : faces[slot[gone]])
            if (std::find(fk.begin(), fk.end(), f) == fk.end()) fk.push_back(f);
          changed = true;
        }
        for (size_t t = 0; t < nt; t++)
          for (int k = 0; k < 3; k++) {
            uint32_t &v = w.tri[3 * t + k];
            if (now(v) != v) {
              v = now(v);
              fresh[t] = 1;
              if (marks) w.sound[t] = 0;
            }
          }
      }
    }
    // Triangles with two corners in one place, and pairs on the same three points facing opposite ways, out.
    {
      std::vector<char> dead(nt, 0);
      bool any = false;
      std::vector<std::pair<std::array<uint32_t, 3>, uint32_t>> keyed;
      for (size_t t = 0; t < nt; t++) {
        if (!fresh[t]) continue;
        const uint32_t *v = &w.tri[3 * t];
        if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) {
          dead[t] = 1, any = changed = true;
          continue;
        }
        std::array<uint32_t, 3> k{v[0], v[1], v[2]};
        std::sort(k.begin(), k.end());
        keyed.push_back({k, (uint32_t)t});
      }
      std::sort(keyed.begin(), keyed.end());
      // The way round a triangle runs: its corners from the least, as an even or odd turn of the sorted three.
      auto turn = [&](uint32_t t) {
        const uint32_t *v = &w.tri[3 * t];
        int m = v[0] < v[1] && v[0] < v[2] ? 0 : v[1] < v[2] ? 1 : 2;
        return v[(m + 1) % 3] < v[(m + 2) % 3];
      };
      for (size_t i = 0, j; i < keyed.size(); i = j) {
        for (j = i + 1; j < keyed.size() && keyed[j].first == keyed[i].first;) j++;
        if (j - i < 2) continue;
        std::vector<uint32_t> up, down;
        for (size_t k = i; k < j; k++) (turn(keyed[k].second) ? up : down).push_back(keyed[k].second);
        for (size_t k = 0; k < std::min(up.size(), down.size()); k++) dead[up[k]] = dead[down[k]] = 1, any = changed = true;
      }
      if (any) compact(dead);
      nt = w.count();
    }
    // Thin triangles: the corner off the long side put on it, longest side first.
    {
      struct Thin {
        double l2;  // its long side's length, squared
        uint32_t t;
      };
      // A triangle's long side (its corners k, k + 1) and length squared.
      auto longSide = [&](uint32_t t, int &k) {
        double best = -1;
        k = 0;
        for (int j = 0; j < 3; j++) {
          double l2 = norm2(w.pts[w.tri[3 * t + (j + 1) % 3]] - w.pts[w.tri[3 * t + j]]);
          if (l2 > best) best = l2, k = j;
        }
        return best;
      };
      std::vector<Thin> thin;
      for (size_t t = 0; t < nt; t++) {
        if (!fresh[t]) continue;
        int k;
        double best = longSide((uint32_t)t, k);
        V3 A = w.pts[w.tri[3 * t + k]], B = w.pts[w.tri[3 * t + (k + 1) % 3]], C = w.pts[w.tri[3 * t + (k + 2) % 3]];
        double h = best > 0 ? norm(cross(B - A, C - A)) / std::sqrt(best) : 0;
        if (h < eps) thin.push_back({best, (uint32_t)t});
      }
      if (thin.size() < fewest) fewest = thin.size(), stale = 0;
      else stale++;
      if (!thin.empty()) {
        // Longest side first: a thin triangle's pieces after a split are all shorter, so where thin ones lie side by side
        // (points almost on one line) they're done from the outside in and the run ends.
        std::sort(thin.begin(), thin.end(), [](const Thin &x, const Thin &y) { return x.l2 != y.l2 ? x.l2 > y.l2 : x.t < y.t; });
        // The triangles along each thin one's long side.
        std::vector<uint64_t> need;
        for (const Thin &th : thin) {
          int k;
          longSide(th.t, k);
          uint32_t a = w.tri[3 * th.t + k], b = w.tri[3 * th.t + (k + 1) % 3];
          need.push_back((uint64_t)std::min(a, b) << 32 | std::max(a, b));
        }
        std::sort(need.begin(), need.end());
        need.erase(std::unique(need.begin(), need.end()), need.end());
        std::vector<std::pair<uint64_t, uint32_t>> sides;
        for (size_t t = 0; t < nt; t++)
          for (int k = 0; k < 3; k++) {
            uint32_t a = w.tri[3 * t + k], b = w.tri[3 * t + (k + 1) % 3];
            uint64_t key = (uint64_t)std::min(a, b) << 32 | std::max(a, b);
            if (std::binary_search(need.begin(), need.end(), key)) sides.push_back({key, (uint32_t)t});
          }
        std::sort(sides.begin(), sides.end());
        std::vector<uint32_t> on;
        std::vector<char> touched(nt, 0), dead(nt, 0);
        for (const Thin &th : thin) {
          uint32_t t = th.t;
          if (touched[t]) continue;
          int k;
          double best = longSide(t, k);
          uint32_t a = w.tri[3 * t + k], b = w.tri[3 * t + (k + 1) % 3], c = w.tri[3 * t + (k + 2) % 3];
          V3 A = w.pts[a], B = w.pts[b], C = w.pts[c];
          double s = dot(C - A, B - A) / best;
          if (!(s > 0 && s < 1)) continue;
          uint64_t key = (uint64_t)std::min(a, b) << 32 | std::max(a, b);
          on.clear();
          for (auto it = std::lower_bound(sides.begin(), sides.end(), std::make_pair(key, 0u)); it != sides.end() && it->first == key; ++it) on.push_back(it->second);
          bool free = true;
          for (uint32_t u : on) free = free && !touched[u];
          if (!free) continue;
          for (uint32_t u : on) {
            touched[u] = 1, dead[u] = 1;
            int m = 0;
            while (m < 3 && !((w.tri[3 * u + m] == a || w.tri[3 * u + m] == b) && (w.tri[3 * u + (m + 1) % 3] == a || w.tri[3 * u + (m + 1) % 3] == b))) m++;
            if (m == 3) {
              dead[u] = 0;
              continue;
            }
            uint32_t x = w.tri[3 * u + m], y = w.tri[3 * u + (m + 1) % 3], d = w.tri[3 * u + (m + 2) % 3];
            if (d == c) continue;
            V3 nx = w.nrm[3 * u + m], ny = w.nrm[3 * u + (m + 1) % 3], nd = w.nrm[3 * u + (m + 2) % 3];
            double sx = x == a ? s : 1 - s;  // c's place along x → y
            V3 nc = unit(nx * (1 - sx) + ny * sx);
            V3 q[3] = {w.pts[w.tri[3 * u]], w.pts[w.tri[3 * u + 1]], w.pts[w.tri[3 * u + 2]]};
            double g[6] = {0, 0, 0, 0, 0, 0};
            if (gaps) std::copy(&w.gap[6 * u], &w.gap[6 * u] + 6, g);
            uint32_t f = w.face[u];
            auto put = [&](uint32_t p0, uint32_t p1, uint32_t p2, V3 n0, V3 n1, V3 n2) {
              w.tri.insert(w.tri.end(), {p0, p1, p2}), w.nrm.insert(w.nrm.end(), {n0, n1, n2}), w.face.push_back(f);
              if (marks) w.sound.push_back(0);
              fresh.push_back(1);
              if (gaps) {
                V3 piece[3] = {w.pts[p0], w.pts[p1], w.pts[p2]};
                w.gap.resize(w.gap.size() + 6);
                gapOfPiece(g, q[0], q[1], q[2], piece, &w.gap[w.gap.size() - 6]);
              }
              touched.push_back(1), dead.push_back(0);
            };
            put(x, c, d, nx, nc, nd);
            put(c, y, d, nc, ny, nd);
          }
          changed = true;
        }
        compact(dead);
      }
    }
    if (!changed) break;
  }
}

bool balanced(const Welded &w) {
  std::vector<uint64_t> fwd, back;
  fwd.reserve(w.tri.size()), back.reserve(w.tri.size());
  for (size_t t = 0; t < w.tri.size(); t += 3)
    for (int k = 0; k < 3; k++) {
      uint32_t a = w.tri[t + k], b = w.tri[t + (k + 1) % 3];
      fwd.push_back((uint64_t)a << 32 | b), back.push_back((uint64_t)b << 32 | a);
    }
  std::sort(fwd.begin(), fwd.end()), std::sort(back.begin(), back.end());
  return fwd == back;
}

bool sameForm(const FaceGeom &a, const FaceGeom &b) { return sameSurface(a, b); }

void finish(Solid &s, double deflection) {
  Welded w = weld(s);
  {
    double size = 0;
    for (V3 q : w.pts) size = std::max({size, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
    clean(w, std::max(1e-4, 2e-7 * size));
  }
  size_t nt = w.count();
  // Edges between triangles: which triangles, so faces meeting on one surface can be joined and edges found.
  // (Sorted pairs grouped, rather than a map of lists: one allocation, not one per edge.)
  struct Span {
    const uint32_t *p;
    size_t n;
    size_t size() const { return n; }
    uint32_t operator[](size_t i) const { return p[i]; }
  };
  std::vector<std::pair<uint64_t, uint32_t>> sides;
  sides.reserve(3 * nt);
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) {
      uint32_t a = w.tri[3 * t + k], b = w.tri[3 * t + (k + 1) % 3];
      sides.push_back({(uint64_t)std::min(a, b) << 32 | std::max(a, b), (uint32_t)t});
    }
  std::sort(sides.begin(), sides.end());
  std::vector<uint32_t> sideTris(sides.size());
  for (size_t i = 0; i < sides.size(); i++) sideTris[i] = sides[i].second;
  std::vector<std::pair<uint64_t, Span>> edgeTris;
  edgeTris.reserve(sides.size() / 2 + 1);
  for (size_t i = 0, j; i < sides.size(); i = j) {
    for (j = i + 1; j < sides.size() && sides[j].first == sides[i].first;) j++;
    edgeTris.push_back({sides[i].first, Span{&sideTris[i], j - i}});
  }
  // A face left in separate patches (a side cut through, the two ends of a ring's cut) is a face per patch, its deficit
  // shared by area.
  {
    std::vector<int> patch(nt);
    std::iota(patch.begin(), patch.end(), 0);
    auto top = [&](int x) {
      while (patch[x] != x) x = patch[x] = patch[patch[x]];
      return x;
    };
    for (auto &[key, ts] : edgeTris)
      for (size_t i = 1; i < ts.size(); i++)
        if (w.face[ts[0]] == w.face[ts[i]]) patch[top((int)ts[i])] = top((int)ts[0]);
    std::vector<double> area(nt), faceArea(s.faces.size(), 0);
    for (size_t t = 0; t < nt; t++) {
      area[t] = norm(cross(w.pts[w.tri[3 * t + 1]] - w.pts[w.tri[3 * t]], w.pts[w.tri[3 * t + 2]] - w.pts[w.tri[3 * t]]));
      faceArea[w.face[t]] += area[t];
    }
    // The first patch of a face keeps its number; any more get new ones.
    std::unordered_map<int, uint32_t> number;
    std::vector<char> taken(s.faces.size(), 0);
    std::vector<uint32_t> source(s.faces.size());
    std::iota(source.begin(), source.end(), 0);
    std::vector<double> patchArea(s.faces.size(), 0);
    std::vector<Solid::Face> faces = s.faces;
    for (size_t t = 0; t < nt; t++) {
      auto it = number.find(top((int)t));
      if (it == number.end()) {
        uint32_t f = w.face[t], id = f;
        if (taken[f]) id = (uint32_t)faces.size(), faces.push_back(s.faces[f]), source.push_back(f), patchArea.push_back(0);
        taken[f] = 1;
        it = number.emplace(top((int)t), id).first;
      }
      patchArea[it->second] += area[t];
      w.face[t] = it->second;
    }
    for (size_t f = 0; f < faces.size(); f++) {
      double whole = faceArea[source[f]];
      faces[f].deficit = whole > 0 ? s.faces[source[f]].deficit * patchArea[f] / whole : 0;
    }
    s.faces = faces;
  }
  size_t nf = s.faces.size();
  std::vector<int> parent(nf);
  std::iota(parent.begin(), parent.end(), 0);
  auto find = [&](int x) {
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
  };
  for (auto &[key, ts] : edgeTris)
    for (size_t i = 1; i < ts.size(); i++) {
      int fa = find((int)w.face[ts[0]]), fb = find((int)w.face[ts[i]]);
      if (fa == fb || !sameSurface(s.faces[fa].geom, s.faces[fb].geom)) continue;
      parent[fb] = fa;
      Solid::Face &f = s.faces[fa], &o = s.faces[fb];
      spanBoth(f.geom, o.geom);
      f.blend = f.blend || o.blend, f.aux = f.aux && o.aux;
    }
  // Faces renumbered: only those with triangles, joined ones as one.
  std::vector<int> used(nf, 0), number(nf, -1);
  for (size_t t = 0; t < nt; t++) used[find((int)w.face[t])] = 1;
  std::vector<Solid::Face> faces;
  for (size_t f = 0; f < nf; f++)
    if (used[f] && find((int)f) == (int)f) number[f] = (int)faces.size(), faces.push_back(s.faces[f]), faces.back().deficit = 0;
  for (size_t f = 0; f < nf; f++)
    if (number[find((int)f)] >= 0) faces[number[find((int)f)]].deficit += s.faces[f].deficit;
  for (size_t t = 0; t < nt; t++) w.face[t] = (uint32_t)number[find((int)w.face[t])];
  s.faces = faces;

  // Edges: where two faces meet, chained into lines. (Sorted lists throughout, not maps: no allocation per point.)
  struct Link {
    int fa, fb;
    uint32_t a, b;
  };
  std::vector<Link> links;
  for (auto &[key, ts] : edgeTris) {
    if (ts.size() != 2) continue;
    int fa = (int)w.face[ts[0]], fb = (int)w.face[ts[1]];
    if (fa == fb) continue;
    links.push_back({std::min(fa, fb), std::max(fa, fb), (uint32_t)(key >> 32), (uint32_t)(key & 0xffffffffu)});
  }
  std::sort(links.begin(), links.end(), [](const Link &x, const Link &y) {
    return x.fa != y.fa ? x.fa < y.fa : x.fb != y.fb ? x.fb < y.fb : x.a != y.a ? x.a < y.a : x.b < y.b;
  });
  s.edges.clear(), s.circles.clear(), s.corners.clear();
  std::vector<std::array<uint32_t, 3>> ends;  // (point, the other end, piece) both ways round, by point
  std::vector<char> walked;
  for (size_t i = 0, j; i < links.size(); i = j) {
    for (j = i + 1; j < links.size() && links[j].fa == links[i].fa && links[j].fb == links[i].fb;) j++;
    size_t n = j - i;
    ends.clear();
    for (size_t k = 0; k < n; k++) {
      const Link &l = links[i + k];
      ends.push_back({l.a, l.b, (uint32_t)k}), ends.push_back({l.b, l.a, (uint32_t)k});
    }
    std::sort(ends.begin(), ends.end());
    walked.assign(n, 0);
    auto at = [&](uint32_t pt) {
      auto lo = std::lower_bound(ends.begin(), ends.end(), std::array<uint32_t, 3>{pt, 0, 0});
      auto hi = lo;
      while (hi != ends.end() && (*hi)[0] == pt) ++hi;
      return std::make_pair(lo, hi);
    };
    // Lines from their ends first (points not met by exactly two pieces), then the closed loops left.
    std::vector<uint32_t> starts;
    for (int pass = 0; pass < 2; pass++)
      for (size_t k = 0, m; k < ends.size(); k = m) {
        for (m = k + 1; m < ends.size() && ends[m][0] == ends[k][0];) m++;
        if ((m - k != 2) == (pass == 0)) starts.push_back(ends[k][0]);
      }
    for (uint32_t st : starts) {
      auto [lo, hi] = at(st);
      for (auto it = lo; it != hi; ++it) {
        uint32_t piece = (*it)[2];
        if (walked[piece]) continue;
        Solid::Edge e;
        e.f0 = links[i].fa, e.f1 = links[i].fb;
        e.pts.push_back(w.pts[st]);
        uint32_t cur = (*it)[1];
        walked[piece] = 1;
        e.pts.push_back(w.pts[cur]);
        while (cur != st) {
          auto [clo, chi] = at(cur);
          if (chi - clo != 2) break;
          const auto &next = (*clo)[2] == piece ? *(clo + 1) : *clo;
          if (walked[next[2]]) break;
          piece = next[2], walked[piece] = 1, cur = next[1];
          e.pts.push_back(w.pts[cur]);
        }
        // A line on a turned face at one height along its axis is a circle round that axis (a hole's rim, a cut across a
        // cylinder): exact again from the face's profile. Half a turn or more is listed as a circle.
        for (int f : {e.f0, e.f1}) {
          Ring ring;
          if (!circleOn(faces[f].geom, e.pts, deflection, ring)) continue;
          e.geom = ring.geom;
          if (std::fabs(ring.turn) >= M_PI - 1e-6) s.circles.push_back({ring.centre, ring.axis, ring.radius});
          break;
        }
        if (e.geom.kind != EdgeGeom::Circle && e.pts.size() == 2) e.geom.kind = EdgeGeom::Line;
        s.edges.push_back(e);
      }
    }
  }
  // Corners: points where three or more faces meet.
  {
    std::vector<uint64_t> at;
    at.reserve(3 * nt);
    for (size_t t = 0; t < nt; t++)
      for (int k = 0; k < 3; k++) at.push_back((uint64_t)w.tri[3 * t + k] << 32 | w.face[t]);
    std::sort(at.begin(), at.end());
    at.erase(std::unique(at.begin(), at.end()), at.end());
    for (size_t k = 0, m; k < at.size(); k = m) {
      for (m = k + 1; m < at.size() && at[m] >> 32 == at[k] >> 32;) m++;
      if (m - k >= 3) s.corners.push_back(w.pts[at[k] >> 32]);
    }
  }
  unweld(w, s);
  s.centroids();
  // Each face's deficit: its triangles' slivers.
  if (!s.gap.empty()) {
    for (auto &f : s.faces) f.deficit = 0;
    for (size_t t = 0; t < s.triFace.size(); t++) s.faces[s.triFace[t]].deficit += s.sliver(t);
  }
  // Each face's normal: its plane's, or its own at the triangle nearest its middle.
  std::vector<double> best(s.faces.size(), INFINITY);
  for (size_t t = 0; t < s.triFace.size(); t++) {
    auto &f = s.faces[s.triFace[t]];
    if (f.geom.flat) {
      f.normal = f.geom.pn;
      continue;
    }
    V3 c = (s.p[s.tri[3 * t]] + s.p[s.tri[3 * t + 1]] + s.p[s.tri[3 * t + 2]]) / 3;
    double d = norm2(c - f.centroid);
    if (d < best[s.triFace[t]]) best[s.triFace[t]] = d, f.normal = unit(s.n[s.tri[3 * t]] + s.n[s.tri[3 * t + 1]] + s.n[s.tri[3 * t + 2]]);
  }
  (void)deflection;
}

// MARK: - reach, box and volume

namespace {

// How far a node reaches along d (its own coordinates), exactly where that can be told; otherwise a bound, not exact
// (the box then comes from the mesh at hand).
Reach reach(const Node &node, V3 d, bool prove) {
  if (node.kind == Node::Prim) {
    V3 at;
    double v = node.model->support(d, &at);
    return {v, at, node.model->exactAlong(d)};
  }
  // Whether a point is inside a part as shown: its kept mesh, the point taken back into the part's own frame (no copy).
  auto within = [&](const Shape &s, V3 q) {
    double grow = s.place.stretch();
    return inside(*evaluate(*s.node, grow > 0 ? 0.05 / grow : 0.05), s.place.inverse().point(q));
  };
  Reach a = support(node.a, d, prove);
  if (node.kind == Node::Treat) {
    // Edges taken off (or filled in): no farther than the shape, not exactly as far.
    a.exact = false;
    return a;
  }
  if (node.kind == Node::Hollow) return a;  // the outside kept (an opening's rim as far as its face was)
  if (node.kind == Node::Split) {
    // Exact while the farthest point stays on the kept side (or on the plane, the edge of it).
    a.exact = a.exact && planeSide(a.point, node.p, node.n) * (node.side == 0 ? 1 : -1) >= 0;
    return a;
  }
  Reach b = support(node.b, d, prove);
  if (node.op == BK_UNION) {
    Reach r = a.value >= b.value ? a : b;
    r.exact = a.exact && b.exact;
    return r;
  }
  if (node.op == BK_SUBTRACT) {
    a.exact = a.exact && prove && !within(node.b, a.point);
    return a;
  }
  if (prove && a.exact && within(node.b, a.point)) return a;
  if (prove && b.exact && within(node.a, b.point)) return b;
  Reach r = a.value <= b.value ? a : b;
  r.exact = false;
  return r;
}

}  // namespace

Reach support(const Shape &s, V3 d, bool prove) {
  V3 local{dot(s.place.column(0), d), dot(s.place.column(1), d), dot(s.place.column(2), d)};
  Reach r = reach(*s.node, local, prove);
  V3 t{s.place.m[3], s.place.m[7], s.place.m[11]};
  return {r.value + dot(t, d), s.place.point(r.point), r.exact};
}

bool placedBounds(const Shape &s, V3 &lo, V3 &hi) {
  bool exact[6], all = true;
  for (int i = 0; i < 3; i++) {
    V3 e{i == 0 ? 1.0 : 0, i == 1 ? 1.0 : 0, i == 2 ? 1.0 : 0};
    Reach up = support(s, e, false), down = support(s, -e, false);
    hi[i] = up.value, lo[i] = -down.value;
    exact[i] = down.exact, exact[3 + i] = up.exact;
    all = all && up.exact && down.exact;
  }
  if (all) return true;
  // The rest from the points of the shape's display mesh (kept from when it was shown), placed: short of the shape by no
  // more than its chord error.
  double grow = s.place.stretch();
  auto shown = evaluate(*s.node, grow > 0 ? 0.05 / grow : 0.05);
  V3 a{INFINITY, INFINITY, INFINITY}, b{-INFINITY, -INFINITY, -INFINITY};
  for (V3 p : shown->p) {
    V3 q = s.place.point(p);
    a = vmin(a, q), b = vmax(b, q);
  }
  if (shown->p.empty()) a = b = V3{s.place.m[3], s.place.m[7], s.place.m[11]};
  for (int i = 0; i < 3; i++) {
    if (!exact[i]) lo[i] = a[i];
    if (!exact[3 + i]) hi[i] = b[i];
  }
  return false;
}

int pieceCount(const Shape &s) {
  // (A mesh body may be in pieces.)
  if (s.node->kind == Node::Prim && s.node->model->kind != Model::Mesh) return 1;
  double grow = s.place.stretch(), d = grow > 0 ? 0.05 / grow : 0.05;
  {
    std::lock_guard<std::mutex> hold(s.node->lock);
    for (const auto &c : s.node->counted)
      if (std::fabs(c.first - d) <= 1e-12 * d) return c.second;
  }
  // Pieces don't change with a placement: counted on the node's own mesh, no copy.
  int n = pieces(*evaluate(*s.node, d));
  std::lock_guard<std::mutex> hold(s.node->lock);
  s.node->counted.push_back({d, n});
  if (s.node->counted.size() > 4) s.node->counted.erase(s.node->counted.begin());
  return n;
}

void bounds(const Shape &s, V3 &lo, V3 &hi, const Solid *meshed) {
  for (int i = 0; i < 3; i++) {
    V3 e{i == 0 ? 1.0 : 0, i == 1 ? 1.0 : 0, i == 2 ? 1.0 : 0};
    Reach up = support(s, e), down = support(s, -e);
    hi[i] = up.value, lo[i] = -down.value;
    if (meshed && (!up.exact || !down.exact)) {
      double a = INFINITY, b = -INFINITY;
      for (const auto &p : meshed->p) a = std::min(a, p[i]), b = std::max(b, p[i]);
      if (!up.exact) hi[i] = b;
      if (!down.exact) lo[i] = a;
    }
  }
}

double volume(const Shape &s, const Solid *meshed) {
  if (s.node->kind == Node::Prim) return s.node->model->volume * std::fabs(s.place.det());
  Solid own;
  if (!meshed) mesh(s, 0.01, own), meshed = &own;
  double v = meshed->meshVolume();
  for (const auto &f : meshed->faces) v += f.deficit;
  return v;
}

}  // namespace bce
