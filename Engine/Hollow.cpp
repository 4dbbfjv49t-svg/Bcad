// Bcad's geometry engine: hollowing. The shape less a void inside it, the void made from the shape's own tree: each
// primitive shrunk by the walls (its faces moved in: a flat-sided one's planes, a turned one's or a tube's outline), an
// opening's face moved out through the shape instead and a face with a wall of its own by that; the parts of a merge
// shrunk alike (a face hidden inside the merge moved out, into the next part's void), what's taken away grown, a cut's
// face moved in; roundings narrower by the walls, bevels moved in, inward roundings wider round the same edges.
#include "Engine/Model.hpp"
#include "Engine/Treat.hpp"
#include "Engine/Weld.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <unordered_map>

namespace bce {

namespace {

constexpr double pi = M_PI;

V3 p2(double u, double v) { return {u, v, 0}; }
double cross2(V3 a, V3 b) { return a.x * b.y - a.y * b.x; }

// What each face of the shape becomes: its form in place, and how far it moves in.
struct Rules {
  double t = 2, out = 4, d = 0.05, size = 1;
  std::vector<FaceGeom> open, shown;
  std::vector<std::pair<FaceGeom, double>> own;
};

FaceGeom flipped(FaceGeom g) {
  if (g.flat) g.pn = -g.pn, g.pd = -g.pd;
  return g;
}

// Whether a shape is made of others (a merge, a treatment or a hollow anywhere in it, not only a primitive, cut or not).
bool compound(const Shape &s) {
  const Node &n = *s.node;
  if (n.kind == Node::Split) return compound(n.a);
  return n.kind != Node::Prim;
}

// Whether face f meets another face along some edge (else it's the whole surface of its piece).
bool bounded(const Solid &s, int f) {
  for (const auto &e : s.edges)
    if ((e.f0 == f && e.f1 >= 0 && e.f1 != f) || (e.f1 == f && e.f0 >= 0 && e.f0 != f)) return true;
  return false;
}

// How far a part's face moves in (less than zero: out). Growing (a part taken away), every face moves out by its wall;
// in a merge, common part or cut (`inBool`), a face that doesn't show in the shape moves out.
double moveOf(const Rules &r, const FaceGeom &g0, int sign, bool flip, bool inBool) {
  FaceGeom g = flip ? flipped(g0) : g0;
  // An opening before a wall of its own, should a face be picked as both (as OpenCascade's kernel takes it).
  if (sign > 0)
    for (const auto &o : r.open)
      if (sameForm(o, g)) return -r.out;
  for (const auto &[w, t] : r.own)
    if (sameForm(w, g)) return sign * t;
  if (sign < 0) return -r.t;
  if (inBool) {
    bool shown = false;
    for (const auto &f : r.shown) shown = shown || sameForm(f, g);
    if (!shown) return -r.t;
  }
  return r.t;
}

// MARK: - a flat-sided primitive

// Its planes moved in (in place), and the solid they bound: every three planes' meeting point inside all the others, the
// faces round them. False when nothing is left.
bool flatInset(const Model &m, const Affine &W, const std::vector<double> &move, double d, double size, Solid &out) {
  std::vector<V3> world;
  for (V3 v : m.verts) world.push_back(W.point(v));
  V3 mid;
  for (V3 v : world) mid += v;
  mid = mid / (double)world.size();
  std::vector<V3> n;
  std::vector<double> c;
  for (size_t f = 0; f < m.loops.size(); f++) {
    const auto &loop = m.loops[f];
    V3 nrm;
    for (size_t i = 0; i < loop.size(); i++) {
      V3 a = world[loop[i]], b = world[loop[(i + 1) % loop.size()]];
      nrm += V3{(a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)};
    }
    nrm = unit(nrm);
    if (dot(nrm, world[loop[0]] - mid) < 0) nrm = -nrm;
    n.push_back(nrm), c.push_back(dot(nrm, world[loop[0]]) - move[f]);
  }
  double eps = 1e-9 * (1 + size);
  std::vector<V3> pts;
  size_t k = n.size();
  for (size_t a = 0; a < k; a++)
    for (size_t b = a + 1; b < k; b++)
      for (size_t e = b + 1; e < k; e++) {
        V3 bc = cross(n[b], n[e]);
        double det = dot(n[a], bc);
        if (std::fabs(det) < 1e-12) continue;
        V3 x = (bc * c[a] + cross(n[e], n[a]) * c[b] + cross(n[a], n[b]) * c[e]) / det;
        bool in = true;
        for (size_t l = 0; l < k && in; l++) in = dot(n[l], x) <= c[l] + eps;
        if (!in) continue;
        bool seen = false;
        for (V3 q : pts) seen = seen || norm(q - x) <= eps * 10;
        if (!seen) pts.push_back(x);
      }
  if (pts.size() < 4) return false;
  std::vector<std::vector<int>> loops;
  for (size_t f = 0; f < k; f++) {
    std::vector<int> on;
    V3 centre;
    for (size_t i = 0; i < pts.size(); i++)
      if (std::fabs(dot(n[f], pts[i]) - c[f]) <= eps * 10) on.push_back((int)i), centre += pts[i];
    if (on.size() < 3) continue;
    centre = centre / (double)on.size();
    V3 e1 = unit(pts[on[0]] - centre), e2 = cross(n[f], e1);
    std::sort(on.begin(), on.end(), [&](int a, int b) {
      double ta = trig::atan2(dot(pts[a] - centre, e2), dot(pts[a] - centre, e1)), tb = trig::atan2(dot(pts[b] - centre, e2), dot(pts[b] - centre, e1));
      return ta < tb || (ta == tb && a < b);
    });
    loops.push_back(on);
  }
  if (loops.size() < 4) return false;
  mesh(shapeOf(polyModel(pts, loops)), d, out);
  return out.meshVolume() > 1e-9 * size * size * size;
}

// MARK: - a turned primitive's outline, or a tube's section

// A piece moved in by o (along its inward normal; a full circle shrinks).
Elem movedIn(const Elem &e, double o) {
  double nr, nz;
  e.normalAt(0.5, nr, nz);
  if (!e.arc) return Elem::line(e.r0 - nr * o, e.z0 - nz * o, e.r1 - nr * o, e.z1 - nz * o);
  double mr, mz;
  e.at(0.5, mr, mz);
  bool convex = (mr - e.cr) * nr + (mz - e.cz) * nz > 0;
  double rad = convex ? e.rad - o : e.rad + o;
  Elem out = e;
  out.rad = rad;
  return out;
}

// Where two moved pieces meet, nearest `near`; false when they don't.
bool meetOf(const Elem &a, const Elem &b, V3 near, V3 &out) {
  std::vector<V3> cands;
  auto lineLine = [&](const Elem &p, const Elem &q) {
    V3 d1 = p2(p.r1 - p.r0, p.z1 - p.z0), d2 = p2(q.r1 - q.r0, q.z1 - q.z0), w = p2(q.r0 - p.r0, q.z0 - p.z0);
    double den = d1.x * d2.y - d1.y * d2.x;
    if (std::fabs(den) < 1e-12 * norm(d1) * norm(d2)) {
      // Running on in line: the joint moved with them.
      if (std::fabs(d1.x * w.y - d1.y * w.x) < 1e-9 * (1 + norm(d1) * norm(w))) cands.push_back(p2(p.r1, p.z1));
      return;
    }
    double s = (w.x * d2.y - w.y * d2.x) / den;
    cands.push_back(p2(p.r0 + d1.x * s, p.z0 + d1.y * s));
  };
  auto lineCircle = [&](const Elem &l, const Elem &c) {
    V3 o = p2(l.r0, l.z0), dl = p2(l.r1 - l.r0, l.z1 - l.z0), f = o - p2(c.cr, c.cz);
    double A = dot(dl, dl), B = 2 * dot(f, dl), C = dot(f, f) - c.rad * c.rad, disc = B * B - 4 * A * C;
    if (disc < 0) {
      if (disc > -1e-9 * A * (1 + C)) disc = 0;
      else return;
    }
    for (double sg : {-1.0, 1.0}) cands.push_back(o + dl * ((-B + sg * std::sqrt(disc)) / (2 * A)));
  };
  auto circleCircle = [&](const Elem &p, const Elem &q) {
    V3 c0 = p2(p.cr, p.cz), c1 = p2(q.cr, q.cz);
    double dd = norm(c1 - c0);
    if (dd < 1e-12) return;
    double a = (p.rad * p.rad - q.rad * q.rad + dd * dd) / (2 * dd), h2 = p.rad * p.rad - a * a;
    if (h2 < 0) {
      if (h2 > -1e-9 * p.rad * p.rad) h2 = 0;
      else return;
    }
    V3 u = (c1 - c0) / dd, m = c0 + u * a, v = p2(-u.y, u.x);
    for (double sg : {-1.0, 1.0}) cands.push_back(m + v * (sg * std::sqrt(h2)));
  };
  if (!a.arc && !b.arc) lineLine(a, b);
  else if (!a.arc) lineCircle(a, b);
  else if (!b.arc) lineCircle(b, a);
  else circleCircle(a, b);
  if (cands.empty()) return false;
  out = cands[0];
  for (V3 q : cands)
    if (norm(q - near) < norm(out - near)) out = q;
  return true;
}

// The outline moved in piece by piece (each by its own amount; pieces on the axis stay there); false when it folds over
// itself or a piece runs out.
bool outlineInset(const std::vector<Elem> &prof, const std::vector<double> &move, std::vector<Elem> &out, bool turned = true) {
  size_t n = prof.size();
  out.clear();
  if (n == 1 && prof[0].arc) {
    Elem e = movedIn(prof[0], move[0]);
    if (!(e.rad > 0)) return false;
    out.push_back(Elem::arcOf(e.cr, e.cz, e.rad, e.a0, e.a1));
    return true;
  }
  std::vector<Elem> moved(n);
  for (size_t k = 0; k < n; k++) moved[k] = prof[k].onAxis() ? prof[k] : movedIn(prof[k], move[k]);
  // Each joint: where the moved pieces either side meet, near the joint moved by both.
  std::vector<V3> joint(n);
  for (size_t k = 0; k < n; k++) {
    const Elem &a = moved[k], &b = moved[(k + 1) % n];
    double nr0, nz0, nr1, nz1;
    prof[k].normalAt(1, nr0, nz0), prof[(k + 1) % n].normalAt(0, nr1, nz1);
    double oa = prof[k].onAxis() ? 0 : move[k], ob = prof[(k + 1) % n].onAxis() ? 0 : move[(k + 1) % n];
    V3 near = p2(prof[k].r1 - (nr0 * oa + nr1 * ob) / 2, prof[k].z1 - (nz0 * oa + nz1 * ob) / 2);
    if (!meetOf(a, b, near, joint[k])) return false;
    if (std::fabs(joint[k].x) < 1e-9 * (1 + std::fabs(joint[k].y))) joint[k].x = 0;
    // (A tube's section isn't turned: its r may run below zero.)
    if (turned && joint[k].x < 0) return false;
  }
  for (size_t k = 0; k < n; k++) {
    V3 from = joint[(k + n - 1) % n], to = joint[k];
    const Elem &e = prof[k];
    if (!e.arc) {
      // Still running the same way.
      if (dot(to - from, p2(e.r1 - e.r0, e.z1 - e.z0)) <= 0) return false;
      out.push_back(Elem::line(from.x, from.y, to.x, to.y));
      continue;
    }
    const Elem &m = moved[k];
    if (!(m.rad > 0)) return false;
    double a0 = trig::atan2(from.y - m.cz, from.x - m.cr), a1 = trig::atan2(to.y - m.cz, to.x - m.cr), turn = e.a1 - e.a0;
    double t = std::remainder(a1 - a0, 2 * pi);
    if (turn > 0 && t <= 0) t += 2 * pi;
    if (turn < 0 && t >= 0) t -= 2 * pi;
    if (std::fabs(t) < 1e-9 || std::fabs(t - turn) > pi) return false;
    Elem arc = Elem::arcOf(m.cr, m.cz, m.rad, a0, a0 + t);
    arc.r0 = from.x, arc.z0 = from.y, arc.r1 = to.x, arc.z1 = to.y;
    out.push_back(arc);
  }
  // Still round its region the same way.
  double area = 0;
  for (const auto &e : out) {
    for (int k = 0; k < (e.arc ? 16 : 1); k++) {
      double r0, z0, r1, z1;
      e.at(k / (e.arc ? 16.0 : 1.0), r0, z0), e.at((k + 1) / (e.arc ? 16.0 : 1.0), r1, z1);
      area += r0 * z1 - r1 * z0;
    }
  }
  return area > 0;
}

// The region an outline leaves moved in, however it does: where it's narrower than its walls it closes up (a glass's side
// thinner than two walls stays solid), and what's left may come apart in several. Each piece moved in whole, joined to the
// next where they meet (or by a turn round their corner), then only what lies inside that outline once over is kept.
namespace offset {

struct Seg {
  bool arc = false;
  V3 a, b;                           // ends
  double cr = 0, cz = 0, rad = 0, a0 = 0, a1 = 0;  // an arc: centre, radius, angles from a to b
  V3 at(double t) const {
    if (!arc) return a + (b - a) * t;
    if (t <= 0) return a;
    if (t >= 1) return b;
    double g = a0 + (a1 - a0) * t;
    return p2(cr + rad * trig::cos(g), cz + rad * trig::sin(g));
  }
  // Left of the way it runs (inside, for an outline running counter-clockwise).
  V3 left(double t) const {
    if (!arc) return unit(p2(-(b.y - a.y), b.x - a.x));
    V3 p = at(t), c = p2(cr, cz);
    return unit(a1 > a0 ? c - p : p - c);
  }
};

Seg segOf(const Elem &e) {
  Seg s;
  s.arc = e.arc, s.a = p2(e.r0, e.z0), s.b = p2(e.r1, e.z1);
  if (e.arc) s.cr = e.cr, s.cz = e.cz, s.rad = e.rad, s.a0 = e.a0, s.a1 = e.a1;
  return s;
}

Seg lineSeg(V3 a, V3 b) {
  Seg s;
  s.a = a, s.b = b;
  return s;
}

// The angle a piece turns through seen from p (a closed outline's winding is their sum over 2π).
double turn(const Seg &s, V3 p) {
  V3 u = s.a - p, v = s.b - p;
  double th = trig::atan2(cross2(u, v), dot(u, v));
  if (!s.arc) return th;
  // Between the chord and the arc: the arc goes the other way round p.
  V3 c = p2(s.cr, s.cz), mid = s.at(0.5);
  if (norm(p - c) < s.rad && cross2(s.b - s.a, p - s.a) * cross2(s.b - s.a, mid - s.a) > 0) th += (s.a1 > s.a0 ? 2 : -2) * pi;
  return th;
}

double winding(const std::vector<Seg> &loop, V3 p) {
  double w = 0;
  for (const auto &s : loop) w += turn(s, p);
  return w / (2 * pi);
}

// Where two pieces cross, as each one's t (ends of one lying on the other included).
void crossings(const Seg &p, const Seg &q, double eps, std::vector<double> &tp, std::vector<double> &tq) {
  auto onSeg = [&](const Seg &s, V3 x, double &t) {
    if (!s.arc) {
      V3 d = s.b - s.a;
      double l2 = dot(d, d);
      if (l2 <= 0) return false;
      t = dot(x - s.a, d) / l2;
      return t >= -1e-9 && t <= 1 + 1e-9 && std::fabs(cross2(d, x - s.a)) <= eps * std::sqrt(l2);
    }
    if (std::fabs(norm(x - p2(s.cr, s.cz)) - s.rad) > eps) return false;
    double g = trig::atan2(x.y - s.cz, x.x - s.cr), lo = std::min(s.a0, s.a1), hi = std::max(s.a0, s.a1);
    while (g < lo - 1e-9) g += 2 * pi;
    while (g > hi + 1e-9) g -= 2 * pi;
    if (g < lo - 1e-9 || g > hi + 1e-9) return false;
    t = (g - s.a0) / (s.a1 - s.a0);
    return true;
  };
  std::vector<V3> pts;
  if (!p.arc && !q.arc) {
    V3 d1 = p.b - p.a, d2 = q.b - q.a, w = q.a - p.a;
    double den = cross2(d1, d2);
    if (std::fabs(den) > 1e-12 * norm(d1) * norm(d2)) pts.push_back(p.a + d1 * (cross2(w, d2) / den));
  } else if (p.arc != q.arc) {
    const Seg &l = p.arc ? q : p, &c = p.arc ? p : q;
    V3 o = l.a, dl = l.b - l.a, f = o - p2(c.cr, c.cz);
    double A = dot(dl, dl), B = 2 * dot(f, dl), C = dot(f, f) - c.rad * c.rad, disc = B * B - 4 * A * C;
    if (A > 0 && disc >= -1e-12 * A * (1 + std::fabs(C)))
      for (double sg : {-1.0, 1.0}) pts.push_back(o + dl * ((-B + sg * std::sqrt(std::max(disc, 0.0))) / (2 * A)));
  } else {
    V3 c0 = p2(p.cr, p.cz), c1 = p2(q.cr, q.cz);
    double dd = norm(c1 - c0);
    if (dd > 1e-12) {
      double a = (p.rad * p.rad - q.rad * q.rad + dd * dd) / (2 * dd), h2 = p.rad * p.rad - a * a;
      if (h2 >= -1e-12 * p.rad * p.rad) {
        V3 u = (c1 - c0) / dd, m = c0 + u * a, v = p2(-u.y, u.x);
        for (double sg : {-1.0, 1.0}) pts.push_back(m + v * (sg * std::sqrt(std::max(h2, 0.0))));
      }
    }
  }
  // Running along each other (in line, or round one circle): where each one's ends lie on the other.
  for (V3 x : {p.a, p.b, q.a, q.b}) pts.push_back(x);
  for (V3 x : pts) {
    double s, t;
    if (onSeg(p, x, s) && onSeg(q, x, t)) tp.push_back(std::clamp(s, 0.0, 1.0)), tq.push_back(std::clamp(t, 0.0, 1.0));
  }
}

Seg partOf(const Seg &s, double t0, double t1) {
  Seg r = s;
  r.a = s.at(t0), r.b = s.at(t1);
  if (s.arc) r.a0 = s.a0 + (s.a1 - s.a0) * t0, r.a1 = s.a0 + (s.a1 - s.a0) * t1;
  return r;
}

}  // namespace offset

bool outlineRegions(const std::vector<Elem> &prof, const std::vector<double> &move, bool turned, std::vector<std::vector<Elem>> &out) {
  using namespace offset;
  size_t n = prof.size();
  out.clear();
  double size = 0;
  for (const auto &e : prof) size = std::max({size, std::fabs(e.r0), std::fabs(e.z0), std::fabs(e.r1), std::fabs(e.z1)});
  double eps = 1e-9 * (1 + size);
  // Each piece moved in whole; an arc moved past its centre is gone (its neighbours meet where they meet).
  std::vector<Elem> moved(n);
  std::vector<char> kept(n, 1);
  for (size_t k = 0; k < n; k++) {
    moved[k] = turned && prof[k].onAxis() ? prof[k] : movedIn(prof[k], move[k]);
    if (moved[k].arc) {
      if (!(moved[k].rad > eps)) {
        kept[k] = 0;
        continue;
      }
      moved[k] = Elem::arcOf(moved[k].cr, moved[k].cz, moved[k].rad, moved[k].a0, moved[k].a1);
    }
  }
  std::vector<size_t> order;
  for (size_t k = 0; k < n; k++)
    if (kept[k]) order.push_back(k);
  if (order.size() < 2) return false;
  // The outline moved: each piece whole, then on to the next. At a corner both pieces moved in round (turning left,
  // inside), they overlap: by the corner itself, a loop the wrong way round that's dropped below. Otherwise where their
  // lines or circles meet nearest the corner (a reflex corner's sharp turn, or a piece moved out to open), else
  // straight on.
  std::vector<Seg> loop;
  for (size_t i = 0; i < order.size(); i++) {
    size_t k = order[i], j = order[(i + 1) % order.size()];
    Seg s = segOf(moved[k]), next = segOf(moved[j]);
    loop.push_back(s);
    if (norm(s.b - next.a) <= eps) continue;
    V3 corner = p2(prof[k].r1, prof[k].z1), meet;
    double tkr, tkz, tjr, tjz;
    prof[k].normalAt(1, tkr, tkz), prof[j].normalAt(0, tjr, tjz);
    // (Normals turn as the tangents do.)
    bool left = tkr * tjz - tkz * tjr > 1e-12;
    bool inward = move[k] > 0 && move[j] > 0 && !(turned && (prof[k].onAxis() || prof[j].onAxis()));
    if (left && inward && j == (k + 1) % n) {
      loop.push_back(lineSeg(s.b, corner));
      loop.push_back(lineSeg(corner, next.a));
    } else if (meetOf(moved[k], moved[j], corner, meet) && norm(meet - corner) < 4 * (std::fabs(move[k]) + std::fabs(move[j])) + eps) {
      if (norm(s.b - meet) > eps) loop.push_back(lineSeg(s.b, meet));
      if (norm(meet - next.a) > eps) loop.push_back(lineSeg(meet, next.a));
    } else {
      loop.push_back(lineSeg(s.b, next.a));
    }
  }
  // Every piece cut where any other crosses it.
  std::vector<std::vector<double>> cuts(loop.size());
  for (size_t i = 0; i < loop.size(); i++) cuts[i] = {0, 1};
  for (size_t i = 0; i < loop.size(); i++)
    for (size_t j = i + 1; j < loop.size(); j++) crossings(loop[i], loop[j], 1e-7 * (1 + size), cuts[i], cuts[j]);
  // A part is kept where inside lies once over on its left and nothing on its right.
  std::vector<Seg> parts;
  double off = 1e-6 * (1 + size);
  for (size_t i = 0; i < loop.size(); i++) {
    auto &c = cuts[i];
    std::sort(c.begin(), c.end());
    for (size_t k = 0; k + 1 < c.size(); k++) {
      if (c[k + 1] - c[k] < 1e-9) continue;
      Seg part = partOf(loop[i], c[k], c[k + 1]);
      if (norm(part.b - part.a) <= 10 * eps && !part.arc) continue;
      V3 mid = loop[i].at((c[k] + c[k + 1]) / 2), side = loop[i].left((c[k] + c[k + 1]) / 2);
      double in = winding(loop, mid + side * off), outside = winding(loop, mid - side * off);
      if (in > 0.5 && outside < 0.5) parts.push_back(part);
    }
  }
  // Joined end to end into loops.
  double snap = 1e-6 * (1 + size);
  std::vector<char> used(parts.size(), 0);
  for (size_t s0 = 0; s0 < parts.size(); s0++) {
    if (used[s0]) continue;
    std::vector<Seg> chain{parts[s0]};
    used[s0] = 1;
    while (norm(chain.back().b - chain.front().a) > snap) {
      size_t best = parts.size();
      for (size_t k = 0; k < parts.size(); k++)
        if (!used[k] && norm(parts[k].a - chain.back().b) <= snap) {
          best = k;
          break;
        }
      if (best == parts.size()) return false;
      used[best] = 1;
      chain.push_back(parts[best]);
    }
    std::vector<Elem> region;
    double area = 0;
    for (auto &s : chain) {
      Elem e = s.arc ? Elem::arcOf(s.cr, s.cz, s.rad, s.a0, s.a1) : Elem::line(s.a.x, s.a.y, s.b.x, s.b.y);
      e.r0 = s.a.x, e.z0 = s.a.y, e.r1 = s.b.x, e.z1 = s.b.y;
      if (turned) e.r0 = std::max(e.r0, 0.0), e.r1 = std::max(e.r1, 0.0);
      region.push_back(e);
      for (int k = 0; k < (s.arc ? 16 : 1); k++) {
        V3 u = s.at(k / (s.arc ? 16.0 : 1.0)), v = s.at((k + 1) / (s.arc ? 16.0 : 1.0));
        area += cross2(u, v) / 2;
      }
    }
    if (area > 1e-9 * (1 + size * size)) out.push_back(region);
  }
  return !out.empty();
}

// MARK: - the tree

struct Ctx {
  const Rules &rules;
};

bool voidOf(const Shape &s, const Affine &W, int sign, bool flip, bool inBool, Ctx &ctx, Solid &out);

// An oval cylinder's void: its ends moved in by `bottom` and `top`, its side by `side` — the side's inward offset, which
// isn't an oval, as points close enough to it, the area it misses kept as its face's deficit (the oval's area, less the
// side times its length, plus π times its square). C its middle, X and Y its semi-axes, Z half its height (all placed).
bool ovalVoid(V3 C, V3 X, V3 Y, V3 Z, double bottom, double side, double top, double d, Solid &out) {
  double a = norm(X), b = norm(Y), hz = norm(Z);
  V3 ex = X / a, ey = Y / b, ez = Z / hz;
  if (a < b) std::swap(a, b), std::swap(ex, ey);
  // The offset turns back on itself where it's moved past the tightest bend (b²/a); its ends must not meet.
  double z0 = -hz + bottom, z1 = hz - top, tight = b * b / a - side;
  if (!(tight > 0) || !(z1 > z0) || side < 0) return false;
  int n = (int)std::ceil(2 * pi * a / std::sqrt(8 * tight * std::max(d, 1e-6)));
  n = std::clamp(n + (n & 1), 32, 1024);
  if (std::fabs(dot(ex, ey)) > 1e-9 || std::fabs(dot(ex, ez)) > 1e-9 || std::fabs(dot(ey, ez)) > 1e-9) return false;
  // Facing out of the void either way round: x × y along z.
  if (dot(cross(ex, ey), ez) < 0) ey = -ey;
  std::vector<V3> ring(n), nrm(n);
  double polyArea = 0, perimeter = 0;
  for (int j = 0; j < n; j++) {
    double th = 2 * pi * j / n, c = trig::cos(th), sn = trig::sin(th);
    V3 nn = unit(p2(b * c, a * sn));
    ring[j] = p2(a * c, b * sn) - nn * side, nrm[j] = nn;
  }
  for (int j = 0; j < n; j++) polyArea += cross2(ring[j], ring[(j + 1) % n]) / 2;
  // The oval's length round, closely (its arc summed finely).
  for (int k = 0, m = 4096; k < m; k++) {
    double th = 2 * pi * (k + 0.5) / m;
    perimeter += trig::hypot(a * trig::sin(th), b * trig::cos(th)) * 2 * pi / m;
  }
  double exact = pi * a * b - side * perimeter + pi * side * side;
  auto at = [&](V3 q, double z) { return C + ex * q.x + ey * q.y + ez * z; };
  auto dir = [&](V3 q) { return ex * q.x + ey * q.y; };
  out = Solid();
  out.faces.resize(3);
  for (int e = 0; e < 2; e++) {
    V3 nn = e == 0 ? -ez : ez;
    double z = e == 0 ? z0 : z1;
    auto &f = out.faces[e];
    f.geom.kind = FaceGeom::Flat, f.geom.flat = true, f.geom.pn = nn, f.geom.pd = dot(nn, at(p2(0, 0), z));
    uint32_t mid = out.vertex(at(p2(0, 0), z), nn), first = (uint32_t)out.p.size();
    for (int j = 0; j < n; j++) out.vertex(at(ring[j], z), nn);
    for (int j = 0; j < n; j++) {
      uint32_t u = first + j, v = first + (j + 1) % n;
      if (e == 0) out.triangle(mid, v, u, 0);
      else out.triangle(mid, u, v, 1);
    }
  }
  out.faces[2].geom.kind = FaceGeom::Curved;
  uint32_t lo = (uint32_t)out.p.size();
  for (int j = 0; j < n; j++) out.vertex(at(ring[j], z0), dir(nrm[j]));
  uint32_t hi = (uint32_t)out.p.size();
  for (int j = 0; j < n; j++) out.vertex(at(ring[j], z1), dir(nrm[j]));
  for (int j = 0; j < n; j++) {
    uint32_t a0 = lo + j, a1 = lo + (j + 1) % n, b0 = hi + j, b1 = hi + (j + 1) % n;
    out.triangle(a0, a1, b1, 2), out.triangle(a0, b1, b0, 2);
  }
  out.faces[2].deficit = (exact - polyArea) * (z1 - z0);
  out.centroids();
  out.faces[0].normal = -ez, out.faces[1].normal = ez;
  out.slivers();
  return true;
}

// A primitive moved in: each of its faces by its rule.
bool primitiveVoid(const Node &node, const Affine &Wn, int sign, bool flip, bool inBool, Ctx &ctx, Solid &out) {
  const Model &m = *node.model;
  // A bolt or nut has no inset of its own here: hollowed from its faces instead.
  if (m.kind == Model::Radial) return false;
  const Rules &r = ctx.rules;
  // Its faces as meshed in place (forms exactly as the shape's own).
  Solid placed;
  mesh(Shape{std::shared_ptr<const Node>(&node, [](const Node *) {}), Wn}, r.d, placed);
  if (m.kind == Model::Poly) {
    if (placed.faces.size() != m.loops.size()) return false;
    std::vector<double> move;
    for (const auto &f : placed.faces) move.push_back(moveOf(r, f.geom, sign, flip, inBool));
    return flatInset(m, Wn, move, r.d, r.size, out);
  }
  // Turned or a tube: each piece's move in the model's own units (under a stretch, enough all along the piece).
  const std::vector<Elem> &prof = m.kind == Model::Turned ? m.profile : m.section;
  double sx = norm(Wn.column(0)), sy = norm(Wn.column(1)), sz = norm(Wn.column(2)), across = std::min(sx, sy);
  std::vector<double> move(prof.size(), 0);
  size_t face = 0;
  for (size_t k = 0; k < prof.size(); k++) {
    if (m.kind == Model::Turned && prof[k].onAxis()) continue;
    if (face >= placed.faces.size()) return false;
    // A move m along the piece's normal (nr, nz) moves its face m / |(nr / across, nz / sz)| in place: the inverse, at its
    // most along the piece (an arc's normal turns), for a wall at least as thick as asked.
    double world = moveOf(r, placed.faces[face++].geom, sign, flip, inBool), most = 0, nr, nz;
    for (double at : {0.0, 0.5, 1.0}) {
      prof[k].normalAt(at, nr, nz);
      most = std::max(most, trig::hypot(nr / std::max(across, 1e-12), nz / std::max(sz, 1e-12)));
    }
    move[k] = world * most;
  }
  // An oval cylinder (a cylinder stretched unevenly across): its side's offset is no oval, so made as such.
  if (m.kind == Model::Turned && std::fabs(sx - sy) > 1e-9 * std::max(sx, sy) && prof.size() == 4 && !prof[0].arc && !prof[1].arc && !prof[2].arc &&
      prof[0].flat() && prof[2].flat() && prof[1].r0 == prof[1].r1 && prof[1].r0 > 0 && prof[3].onAxis() && prof[0].r0 == 0 && prof[2].r1 == 0) {
    double R = prof[1].r0, zlo = prof[0].z0, zhi = prof[2].z0;
    // The moves as asked in place (the pieces' faces in order: bottom, side, top).
    std::vector<double> world;
    for (size_t k = 0; k < 3; k++) world.push_back(moveOf(r, placed.faces[k].geom, sign, flip, inBool));
    V3 C = Wn.point(p2(0, 0) + V3{0, 0, (zlo + zhi) / 2}), X = Wn.vector(V3{R, 0, 0}), Y = Wn.vector(V3{0, R, 0}), Z = Wn.vector(V3{0, 0, (zhi - zlo) / 2});
    return ovalVoid(C, X, Y, Z, world[0], world[1], world[2], r.d, out);
  }
  bool turned = m.kind == Model::Turned;
  // Stretched unevenly, a curved or slanting piece's offset is no longer the same form moved in: walls would come out
  // thicker along the stretch (or thinner across it). Only pieces square to the stretch stay exact (a cylinder stretched
  // along its axis); otherwise walled from its faces instead.
  if (!Wn.similarity()) {
    V3 c0 = Wn.column(0), c1 = Wn.column(1), c2 = Wn.column(2);
    bool square = std::fabs(dot(c0, c1)) <= 1e-9 * sx * sy && std::fabs(dot(c0, c2)) <= 1e-9 * sx * sz && std::fabs(dot(c1, c2)) <= 1e-9 * sy * sz &&
                  std::fabs(sx - sy) <= 1e-9 * std::max(sx, sy);
    for (const auto &e : prof) square = square && !e.arc && (e.r0 == e.r1 || e.z0 == e.z1);
    if (!square || !turned) return false;
  }
  std::vector<std::vector<Elem>> regions(1);
  if (!outlineInset(prof, move, regions[0], turned)) {
    // An opening moved out so far that a curved piece beside it no longer reaches it (a dome round an open base): only
    // just out instead, as far as needed to break through.
    std::vector<double> near = move;
    bool opening = false;
    for (double &mv : near)
      if (mv < 0) mv = -0.02 * r.size, opening = true;
    // Else narrower somewhere than its walls: what room there is (maybe in pieces), the rest solid.
    if (!(opening && outlineInset(prof, near, regions[0], turned)) && !outlineRegions(prof, move, turned, regions)) return false;
  }
  out = Solid();
  for (const auto &region : regions) {
    std::shared_ptr<Model> model = turned ? turnedModel(region) : sweptModel(m.a, m.b, m.phi, region);
    Solid one;
    mesh(shapeOf(model, Wn), r.d, one);
    if (out.tri.empty()) out = std::move(one);
    else out = combine(out, one, BK_UNION);
  }
  return out.meshVolume() > 1e-9 * r.size * r.size * r.size;
}

// A treated shape's void: its shape's void, its edges treated there too (roundings narrower by the walls round the same
// middles, bevels moved in, inward roundings wider round the same edges).
bool treatedVoid(const Node &node, const Affine &Wn, int sign, bool flip, bool inBool, Ctx &ctx, Solid &out) {
  if (!voidOf(node.a, Wn, sign, flip, inBool, ctx, out)) return false;
  if (sign < 0) return true;
  const Treatment &t = *node.treat;
  const Rules &r = ctx.rules;
  // Its edges, to treat them (a cut one comes without).
  if (out.edges.empty()) finish(out, r.d);
  Solid child;
  mesh(node.a, r.d, child);
  child.transform(Wn);
  // The picks in place.
  std::vector<double> picks = t.picks;
  for (size_t i = 0; i < t.kinds.size(); i++) {
    double *q = &picks[6 * i];
    V3 a{q[0], q[1], q[2]}, b{q[3], q[4], q[5]};
    if (t.kinds[i] == BK_PICK_EDGE) a = Wn.point(a), b = Wn.vector(b);
    else if (t.kinds[i] == BK_PICK_CORNER || t.kinds[i] == BK_PICK_FACE) a = unit(Wn.normal(a)), b = Wn.point(b);
    q[0] = a.x, q[1] = a.y, q[2] = a.z, q[3] = b.x, q[4] = b.y, q[5] = b.z;
  }
  if (t.kind == Treatment::Cove) {
    Treatment wider = t;
    wider.picks = picks, wider.radius = t.radius + r.t;
    TreatFit fit;
    Solid made = treated(child, wider, r.d, fit, &out);
    if (fit.fits) out = made;
    return true;
  }
  int missing = 0;
  std::vector<Crease> creases = creasesOf(child, t.kinds.data(), picks.data(), (int)t.kinds.size(), &missing);
  // Each edge's own: where it lies on the void (moved in by each face's wall), and its rounding or bevel there.
  // Each edge: its radius (or legs) there and its pick; then those alike within rounding treated together.
  std::vector<std::tuple<double, double, std::array<double, 6>>> each;
  for (const auto &whole : creases) {
    if (whole.pts.size() < 2) continue;
    // Taken at its middle by length (a point of its own there; one by count may lie near an end).
    Crease c = whole;
    size_t m = pointAt(c, c.length / 2);
    double oA = moveOf(r, child.faces[c.fa[m]].geom, 1, flip, inBool), oB = moveOf(r, child.faces[c.fb[m]].geom, 1, flip, inBool);
    V3 na = c.na[m], nb = c.nb[m];
    double kk = dot(na, nb), det = 1 - kk * kk;
    if (std::fabs(det) < 1e-9) continue;
    double alpha = (-oA + oB * kk) / det, beta = (-oB + oA * kk) / det;
    V3 at = c.pts[m] + na * alpha + nb * beta, dir = c.tangent(m);
    double keyA = 0, keyB = 0;
    if (t.kind == Treatment::Round) {
      double rad = t.radius - std::max(oA, oB);
      if (rad < 0.005) continue;
      keyA = rad;
    } else {
      // The bevel's line moved in by the walls too, in the cut across the edge.
      V3 U = c.ia[m], N = unit(c.na[m] - U * dot(c.na[m], U));
      V3 db = unit(p2(dot(c.ib[m], U), dot(c.ib[m], N))), nB = unit(p2(dot(c.nb[m], U), dot(c.nb[m], N)));
      V3 PA = p2(t.legA, 0), PB = db * t.legB, along = unit(PB - PA), nP = p2(-along.y, along.x);
      if (dot(nP, PA * -1.0) < 0) nP = -nP;  // out of the material, towards the corner
      auto meet2 = [](V3 n1, double c1, V3 n2, double c2, V3 &x) {
        double dd = n1.x * n2.y - n1.y * n2.x;
        if (std::fabs(dd) < 1e-12) return false;
        x = p2((c1 * n2.y - c2 * n1.y) / dd, (n1.x * c2 - n2.x * c1) / dd);
        return true;
      };
      V3 nA = p2(0, 1), e2, pa, pb;
      double cA = -oA, cB = -oB, cP = dot(nP, PA) - r.t;
      if (!meet2(nA, cA, nB, cB, e2) || !meet2(nA, cA, nP, cP, pa) || !meet2(nB, cB, nP, cP, pb)) continue;
      double la = norm(pa - e2), lb = norm(pb - e2);
      if (la < 0.005 || lb < 0.005) continue;
      keyA = la, keyB = lb;
    }
    each.push_back({keyA, keyB, {at.x, at.y, at.z, dir.x, dir.y, dir.z}});
  }
  std::sort(each.begin(), each.end());
  std::vector<std::pair<std::pair<double, double>, std::vector<double>>> groups;
  auto alike = [](double x, double y) { return std::fabs(x - y) <= 1e-9 * (1 + std::fabs(x)); };
  for (const auto &[a, b, pick] : each) {
    if (groups.empty() || !alike(groups.back().first.first, a) || !alike(groups.back().first.second, b)) groups.push_back({{a, b}, {}});
    groups.back().second.insert(groups.back().second.end(), pick.begin(), pick.end());
  }
  for (const auto &[key, edgePicks] : groups) {
    Treatment there = t;
    there.picks = edgePicks;
    there.kinds.assign(edgePicks.size() / 6, BK_PICK_EDGE);
    if (t.kind == Treatment::Round) there.radius = key.first;
    else there.legA = key.first, there.legB = key.second, there.corner = std::max(0.0, t.corner - r.t);
    TreatFit fit;
    Solid made = treated(out, there, r.d, fit);
    if (fit.fits) out = made;
  }
  return true;
}

bool voidOf(const Shape &s, const Affine &W, int sign, bool flip, bool inBool, Ctx &ctx, Solid &out) {
  Affine Wn = s.place.then(W);
  const Node &node = *s.node;
  const Rules &r = ctx.rules;
  switch (node.kind) {
  case Node::Prim:
    return primitiveVoid(node, Wn, sign, flip, inBool, ctx, out);
  case Node::Treat:
    return treatedVoid(node, Wn, sign, flip, inBool, ctx, out);
  case Node::Hollow:
    // Hollowed already: the void of its shape (what it holds inside stays held).
    return voidOf(node.a, Wn, sign, flip, inBool, ctx, out);
  case Node::Split: {
    if (!voidOf(node.a, Wn, sign, flip, inBool, ctx, out)) return false;
    // The cut's face (facing out of the kept side) moved in by its rule.
    V3 p = Wn.point(node.p), n = unit(Wn.normal(node.n));
    if (node.side != 0) n = -n;
    FaceGeom g;
    g.kind = FaceGeom::Flat, g.flat = true, g.pn = -n, g.pd = dot(-n, p);
    double o = moveOf(r, g, sign, flip, inBool);
    out = cut(out, p + n * o, n, 0);
    return !out.tri.empty();
  }
  case Node::Bool: {
    // A part's faces that don't show in the shape wall nothing in (moved out instead): merged, what each adds of itself
    // is all the void needs from it; in a common part or one cut away from, such a face walls in only what the faces
    // shown wall in already, and an opening in a face shown beside it must pass it.
    Solid a, b;
    if (!voidOf(node.a, Wn, sign, flip, true, ctx, a)) return false;
    if (node.op == BK_SUBTRACT) {
      // What's taken away grows by the walls (its faces in the result face the other way).
      if (!voidOf(node.b, Wn, -sign, !flip, false, ctx, b)) return false;
    } else if (!voidOf(node.b, Wn, sign, flip, true, ctx, b)) {
      return false;
    }
    out = combine(a, b, node.op);
    finish(out, r.d);
    return !out.tri.empty();
  }
  }
  return false;
}

// MARK: - walls from the finished faces

// The shape's own faces moved in by their walls, as OpenCascade offsets a shape's faces: the walls are what lies within
// them, whatever the shape was made of (a merge has no inside faces left to wall, a rounding's face is a face like any).
// Faces meeting smoothly are one patch, moved in together: a slab, the patch raised a hair out of the shape and moved in
// by its wall, closed round its rim. Where a patch meets another inside a corner (or an opening at more than a right
// angle), the slab runs on past the edge to where the two moved faces meet (a wing), so the void's corner is sharp, as
// the moved faces meet. The shape is kept where a slab or a wing covers it.

// A face's outward normal at p where its form is known (its plane; a turned face's exactly), else the mesh's m; with
// whether the face runs the way its profile's outside does (+1) or the other (-1: a hole's face) — told once per face,
// from its broadest triangle (a corner's own normal can come out turned round where meshes were cut near tangent).
struct Normals {
  const Solid &s;
  std::vector<Affine> back;
  std::vector<int> way;
  explicit Normals(const Solid &solid) : s(solid), back(solid.faces.size()), way(solid.faces.size(), 1) {
    std::vector<double> broadest(s.faces.size(), -1);
    std::vector<size_t> pick(s.faces.size(), 0);
    for (size_t t = 0; t < s.triFace.size(); t++) {
      V3 a = s.p[s.tri[3 * t]], b = s.p[s.tri[3 * t + 1]], c = s.p[s.tri[3 * t + 2]];
      double area = norm(cross(b - a, c - a));
      uint32_t f = s.triFace[t];
      if (area > broadest[f]) broadest[f] = area, pick[f] = t;
    }
    for (size_t f = 0; f < s.faces.size(); f++) {
      const FaceGeom &g = s.faces[f].geom;
      if (g.flat || g.kind != FaceGeom::Turned || !g.exact || broadest[f] <= 0) continue;
      back[f] = g.place.inverse();
      size_t t = pick[f];
      V3 a = s.p[s.tri[3 * t]], b = s.p[s.tri[3 * t + 1]], c = s.p[s.tri[3 * t + 2]];
      way[f] = dot(profileNormal((int)f, (a + b + c) / 3.0), cross(b - a, c - a)) < 0 ? -1 : 1;
    }
  }
  // A turned face's profile's outward normal at p (placed).
  V3 profileNormal(int f, V3 p) const {
    const FaceGeom &g = s.faces[f].geom;
    V3 q = back[f].point(p);
    double r = trig::hypot(q.x, q.y), nr, nz;
    g.elem.normalAt(g.elem.nearest(r, q.z), nr, nz);
    V3 local = r > 1e-12 ? V3{nr * q.x / r, nr * q.y / r, nz} : V3{0, 0, nz >= 0 ? 1.0 : -1.0};
    return g.place.normal(local);
  }
  V3 at(int f, V3 p, V3 m, int *side = nullptr) const {
    const Solid::Face &face = s.faces[f];
    const FaceGeom &g = face.geom;
    if (side) *side = way[f];
    if (g.flat) return g.pn;
    if (g.kind == FaceGeom::Turned && g.exact) {
      V3 w = profileNormal(f, p) * (double)way[f];
      // Only where it agrees with the mesh (a face joined from pieces of other forms keeps one form, wrong over the rest,
      // and may face either way there): elsewhere the mesh's own.
      if (norm(m) > 0 && !(dot(unit(w), unit(m)) > 0.95)) return unit(m);
      if (norm(w) > 0) return w;
    }
    return norm(m) > 0 ? unit(m) : face.normal;
  }
};

// A face's form moved in by o along its own normal (side as Normals::at gives it); a curved face with no form stays so.
FaceGeom movedForm(const FaceGeom &g, double o, int side) {
  FaceGeom out = g;
  if (g.flat) {
    out.pd = g.pd - o;
    return out;
  }
  double s = 1;
  if (g.kind == FaceGeom::Turned && g.exact && g.place.similarity(&s) && s > 0) {
    out.elem = movedIn(g.elem, side * o / s);
    if (!g.elem.arc || out.elem.rad > 0) return out;
  }
  out.kind = FaceGeom::Curved, out.exact = false;
  return out;
}

// A face's form facing the other way (a plane's; a turned face's form has no way of its own).
FaceGeom facingIn(FaceGeom g) {
  if (g.flat) g.pn = -g.pn, g.pd = -g.pd;
  return g;
}

// The offset from a point where two sides meet that stands w1 out from the first (outward b1) and w2 from the second.
V3 mitre(V3 b1, double w1, V3 b2, double w2) {
  double c = dot(b1, b2), den = 1 - c * c, most = 4 * std::max(w1, w2);
  if (den < 1e-6) return b1 * std::max(w1, w2);
  V3 m = b1 * ((w1 - c * w2) / den) + b2 * ((w2 - c * w1) / den);
  double l = norm(m);
  return l > most ? m * (most / l) : m;
}

// Turned the other way round (its triangles, and their gaps with them).
void reversed(Solid &s) {
  for (size_t k = 0; k < s.tri.size(); k += 3) std::swap(s.tri[k + 1], s.tri[k + 2]);
  for (size_t k = 0; k < s.gap.size(); k += 6) std::swap(s.gap[k + 1], s.gap[k + 2]), std::swap(s.gap[k + 3], s.gap[k + 5]);
}

// The slabs (and with `wings`, the wings) of S's faces, each face's `depth` thick (below zero: an opening, no wall), the
// moved face that much (`thin`, a part of it) short of the wall. False when a patch folds over itself moving in (curved
// tighter than its wall) or there's nothing to wall.
bool slabsOf(const Solid &S, const std::vector<double> &depth, double size, bool wings, double thin, double d, std::vector<Solid> &parts) {
  Welded w = weld(S);
  size_t nt = w.count();
  if (nt == 0) return false;
  Normals normals(S);
  std::vector<V3> cn(3 * nt);
  for (size_t t = 0; t < nt; t++) {
    // The point's normal as meshed, facing the way its triangle does (a tool's face taken in may keep its own facing).
    V3 a = w.pts[w.tri[3 * t]], b = w.pts[w.tri[3 * t + 1]], c = w.pts[w.tri[3 * t + 2]], tn = cross(b - a, c - a);
    for (int k = 0; k < 3; k++) {
      V3 m = w.nrm[3 * t + k];
      if (norm(m) == 0) m = tn;
      else if (dot(m, tn) < 0) m = -m;
      cn[3 * t + k] = normals.at(w.face[t], w.pts[w.tri[3 * t + k]], m);
    }
  }
  auto cornerOf = [&](size_t t, uint32_t pt) {
    for (int k = 0; k < 3; k++)
      if (w.tri[3 * t + k] == pt) return k;
    return 0;
  };
  // Each side of each triangle (3t + k: from corner k to the next) by its two points.
  auto key = [](uint32_t a, uint32_t b) { return a < b ? (uint64_t)a << 32 | b : (uint64_t)b << 32 | a; };
  std::unordered_map<uint64_t, std::vector<uint32_t>> sides;
  sides.reserve(3 * nt);
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) sides[key(w.tri[3 * t + k], w.tri[3 * t + (k + 1) % 3])].push_back((uint32_t)(3 * t + k));
  auto across = [&](uint32_t side) -> int64_t {
    uint32_t t = side / 3, k = side % 3;
    const auto &v = sides.find(key(w.tri[3 * t + k], w.tri[3 * t + (k + 1) % 3]))->second;
    if (v.size() != 2) return -1;
    return v[0] == side ? v[1] : v[0];
  };
  // Patches: triangles of faces with the same wall meeting smoothly (normals within a degree at both ends of the side).
  const double smooth = trig::cos(pi / 180);
  std::vector<uint32_t> up(nt);
  for (size_t t = 0; t < nt; t++) up[t] = (uint32_t)t;
  auto find = [&](uint32_t x) {
    while (up[x] != x) x = up[x] = up[up[x]];
    return x;
  };
  for (const auto &[k, v] : sides) {
    if (v.size() != 2) continue;
    uint32_t t1 = v[0] / 3, t2 = v[1] / 3;
    int f1 = (int)w.face[t1], f2 = (int)w.face[t2];
    if (depth[f1] < 0 || depth[f2] < 0 || depth[f1] != depth[f2]) continue;
    uint32_t a = w.tri[v[0]], b = w.tri[3 * t1 + (v[0] % 3 + 1) % 3];
    bool same = f1 == f2 || (dot(cn[3 * t1 + cornerOf(t1, a)], cn[3 * t2 + cornerOf(t2, a)]) > smooth &&
                             dot(cn[3 * t1 + cornerOf(t1, b)], cn[3 * t2 + cornerOf(t2, b)]) > smooth);
    if (same) {
      // Joined to the lower number, so each patch's root is its first triangle whatever order the sides come in.
      uint32_t r1 = find(t1), r2 = find(t2);
      if (r1 != r2) up[std::max(r1, r2)] = std::min(r1, r2);
    }
  }
  std::map<uint32_t, std::vector<uint32_t>> patches;
  std::vector<int64_t> owner(nt, -1);
  for (size_t t = 0; t < nt; t++)
    if (depth[w.face[t]] >= 0) owner[t] = find((uint32_t)t), patches[(uint32_t)owner[t]].push_back((uint32_t)t);
  if (patches.empty()) return false;
  // Each face's way round its profile (for its form moved in).
  std::vector<int> sideOf(S.faces.size(), 1);
  std::vector<char> sideSeen(S.faces.size(), 0);
  for (size_t t = 0; t < nt; t++) {
    int f = (int)w.face[t];
    if (sideSeen[f]) continue;
    sideSeen[f] = 1;
    normals.at(f, w.pts[w.tri[3 * t]], w.nrm[3 * t], &sideOf[f]);
  }
  const double eps = 1e-6 * (1 + size);
  auto add = [&](Solid &part) {
    if (part.tri.empty()) return true;
    if (part.meshVolume() < 0) reversed(part);
    part.centroids();
    parts.push_back(std::move(part));
    return true;
  };
  for (auto &[root, tris] : patches) {
    double D = depth[w.face[tris[0]]], raise = 0.1 * D, rimOut = 0.1 * D;
    // Short of the wall by `thin`, and a curved patch by its chords' sag too (the moved face meshed afresh elsewhere
    // isn't met chord across chord).
    bool curved = false;
    for (uint32_t t : tris) curved = curved || !S.faces[w.face[t]].geom.flat;
    double Db = thin > 0 ? D * (1 - thin) - (curved ? 2 * d : 0) : D;
    if (!(Db > 0)) continue;
    // Its points, each with the patch's normal there.
    std::unordered_map<uint32_t, uint32_t> local;
    std::vector<uint32_t> pts;
    std::vector<V3> nv;
    for (uint32_t t : tris)
      for (int k = 0; k < 3; k++) {
        auto [it, fresh] = local.emplace(w.tri[3 * t + k], (uint32_t)pts.size());
        if (fresh) pts.push_back(w.tri[3 * t + k]), nv.push_back({});
        nv[it->second] += cn[3 * t + k];
      }
    for (V3 &n : nv) n = unit(n);
    // Where two faces meet near tangent their meshes cross a hair off where they touch, their normals there a few
    // degrees apart: a normal between the two, beside each face's own a tiny side away, would pleat the moved surface.
    // Round such points the normals are eased (a side's two ends brought to their mean, in turn) until the moved surface
    // keeps every side the way it ran.
    {
      std::vector<char> near(pts.size(), 0);
      for (uint32_t t : tris)
        for (int k = 0; k < 3; k++) {
          uint32_t i = local[w.tri[3 * t + k]];
          if (dot(cn[3 * t + k], nv[i]) < trig::cos(pi / 360)) near[i] = 1;
        }
      std::vector<std::pair<uint32_t, uint32_t>> links;
      for (uint32_t t : tris)
        for (int k = 0; k < 3; k++) {
          uint32_t i = local[w.tri[3 * t + k]], j = local[w.tri[3 * t + (k + 1) % 3]];
          if (i < j) links.push_back({i, j});
        }
      for (int round = 0; round < 60; round++) {
        bool any = false;
        for (auto [i, j] : links) {
          if (!(near[i] || near[j]) || D * norm(nv[i] - nv[j]) <= 0.9 * norm(w.pts[pts[i]] - w.pts[pts[j]])) continue;
          nv[i] = nv[j] = unit(nv[i] + nv[j]);
          near[i] = near[j] = 1, any = true;
        }
        if (!any) break;
      }
    }
    // Moved in, no side may turn back (a face curved tighter than its wall), nor a triangle of any breadth turn over. Not
    // a side much shorter than the wall, nor a sliver: where two faces meet near tangent their meshes cross a hair off
    // where they touch, their normals there a few degrees apart, which over a tiny side looks like a fold and isn't one.
    for (uint32_t t : tris) {
      V3 P[3], Q[3];
      for (int k = 0; k < 3; k++) {
        uint32_t i = local[w.tri[3 * t + k]];
        P[k] = w.pts[pts[i]], Q[k] = P[k] - nv[i] * D;
      }
      double round = 0;
      for (int k = 0; k < 3; k++) {
        V3 dp = P[(k + 1) % 3] - P[k];
        if (norm(dp) > 0.25 * D && dot(Q[(k + 1) % 3] - Q[k], dp) < -1e-6 * dot(dp, dp)) return false;
        round += norm(dp);
      }
      V3 nP = cross(P[1] - P[0], P[2] - P[0]), nQ = cross(Q[1] - Q[0], Q[2] - Q[0]);
      if (dot(nP, nQ) < -1e-6 * dot(nP, nP) && norm(nP) > 0.02 * round * round && round > 0.75 * D) return false;
    }
    // Its rim: each side of a triangle with no triangle of the patch across, and what lies across it.
    struct Half {
      uint32_t a, b;    // the patch's points (as `local`), running with the patch's triangle
      int face;         // the patch's face along it
      V3 out;           // across the side, away from the patch, in the surface
      double rim = 0;   // how far the slab runs out past it (into the air, at an outside corner)
      double wing = 0;  // how far a wing runs past it (to where the moved faces meet)
      int next = -1;    // the face across (for a wing's run)
    };
    std::vector<Half> rim;
    for (uint32_t t : tris)
      for (int k = 0; k < 3; k++) {
        uint32_t side = 3 * t + k;
        int64_t o = across(side);
        if (o >= 0 && owner[o / 3] == (int64_t)root) continue;
        Half h;
        uint32_t pa = w.tri[3 * t + k], pb = w.tri[3 * t + (k + 1) % 3];
        h.a = local[pa], h.b = local[pb], h.face = (int)w.face[t];
        V3 dir = unit(w.pts[pb] - w.pts[pa]), nA = unit(nv[h.a] + nv[h.b]);
        h.out = unit(cross(dir, nA));
        if (o < 0) {
          h.rim = rimOut;
        } else {
          uint32_t t2 = (uint32_t)(o / 3);
          int f2 = (int)w.face[t2];
          V3 nB = unit(cn[3 * t2 + cornerOf(t2, pa)] + cn[3 * t2 + cornerOf(t2, pb)]);
          bool opened = depth[f2] < 0;
          h.next = f2;
          if (dot(nA, nB) > smooth) {
            // Smooth on into an opening: the wall ends square on the edge; into a thicker wall, a hair on into it.
            h.rim = !opened && depth[f2] > D ? rimOut : 0;
          } else {
            // In the cut across the side: where A moved in (n_A · x = -D) meets B moved in (an opening: B itself), along
            // `out`; past the side (inside a corner) a wing runs there, else the slab runs a little on into the air.
            double cB = opened ? 0 : -depth[f2], den = dot(nB, h.out);
            double u = std::fabs(den) > 1e-9 ? (cB + D * dot(nB, nA)) / den : 0;
            if (u > eps) h.wing = std::min(u, 10 * std::max(D, opened ? D : depth[f2])) + eps;
            else h.rim = rimOut;
          }
        }
        rim.push_back(h);
      }
    // Round each point of the rim: the sides in and out of it.
    std::unordered_map<uint32_t, std::vector<int>> into, outOf;
    for (size_t i = 0; i < rim.size(); i++) into[rim[i].b].push_back((int)i), outOf[rim[i].a].push_back((int)i);
    std::vector<V3> rimAt(pts.size());
    for (const auto &[p, hs] : outOf) {
      auto it = into.find(p);
      if (hs.size() == 1 && it != into.end() && it->second.size() == 1) {
        const Half &hi = rim[it->second[0]], &ho = rim[hs[0]];
        rimAt[p] = mitre(hi.out, hi.rim, ho.out, ho.rim);
      } else {
        V3 sum;
        for (int i : hs) sum += rim[i].out * rim[i].rim;
        rimAt[p] = sum / (double)hs.size();
      }
    }
    // The slab: the patch raised (face 0, outside the shape), moved in (faces as the patch's, moved), its rim's sides.
    Solid slab;
    slab.faces.resize(2);
    slab.faces[0].aux = true;
    slab.faces[1].geom.kind = FaceGeom::Curved;
    std::map<int, int> movedFace;
    for (uint32_t t : tris) {
      int f = (int)w.face[t];
      if (movedFace.count(f)) continue;
      movedFace[f] = (int)slab.faces.size();
      Solid::Face moved;
      // Facing out of the slab: the face's way turned round (a plane's normal; a turned face's form has no way of its
      // own).
      moved.geom = facingIn(movedForm(S.faces[f].geom, D, sideOf[f]));
      moved.blend = S.faces[f].blend;
      slab.faces.push_back(moved);
    }
    std::vector<uint32_t> top(pts.size()), bottom(pts.size());
    for (size_t i = 0; i < pts.size(); i++) {
      V3 p = w.pts[pts[i]];
      // Only the raised side runs on past the rim (the moved face stays the face's, exactly).
      top[i] = slab.vertex(p + nv[i] * raise + rimAt[i], nv[i]);
      bottom[i] = slab.vertex(p - nv[i] * Db, -nv[i]);
    }
    for (uint32_t t : tris) {
      uint32_t i0 = local[w.tri[3 * t]], i1 = local[w.tri[3 * t + 1]], i2 = local[w.tri[3 * t + 2]];
      slab.triangle(top[i0], top[i1], top[i2], 0);
      slab.gap.insert(slab.gap.end(), 6, 0.0);
      slab.triangle(bottom[i0], bottom[i2], bottom[i1], movedFace[(int)w.face[t]]);
      // What the moved face's chords miss: the face's own (exact as its form gives it), facing the other way, each side's
      // sag grown or shrunk as the bend there is (the turn along a side stays, its radius less the wall: from the normals,
      // the sag dn·dp/8 becomes (dn·dn·D - dn·dp)/8).
      const double *gt = w.gap.empty() ? nullptr : &w.gap[6 * t];
      uint32_t c[3] = {i0, i2, i1};
      const int corner[3] = {0, 2, 1}, mid[3] = {5, 4, 3};
      double g[6];
      for (int k = 0; k < 3; k++) {
        uint32_t a = c[k], b = c[(k + 1) % 3];
        V3 dp = w.pts[pts[b]] - w.pts[pts[a]], dn = nv[b] - nv[a];
        double was = dot(dn, dp) / 8, now = (Db * dot(dn, dn) - dot(dn, dp)) / 8;
        g[k] = gt ? -gt[corner[k]] : 0;
        g[3 + k] = gt && std::fabs(was) > 1e-300 ? gt[mid[k]] * now / was : now;
      }
      slab.gap.insert(slab.gap.end(), g, g + 6);
    }
    for (const Half &h : rim) {
      // A side square to a straight edge is flat (where the slab ends square on the rim: the wall's end at an opening
      // met smoothly); others (round a curved edge, or run on into the air) one curved face.
      V3 A = w.pts[pts[h.a]], B = w.pts[pts[h.b]];
      int f = 1;
      if (norm(rimAt[h.a]) == 0 && norm(rimAt[h.b]) == 0 && std::fabs(dot(cross(B - A, nv[h.a]), nv[h.b])) < 1e-12 * (1 + size * size)) {
        Solid::Face side;
        side.geom.kind = FaceGeom::Flat, side.geom.flat = true, side.geom.pn = h.out, side.geom.pd = dot(h.out, A);
        f = (int)slab.faces.size();
        slab.faces.push_back(side);
      }
      slab.triangle(top[h.b], top[h.a], bottom[h.a], f), slab.triangle(top[h.b], bottom[h.a], bottom[h.b], f);
      slab.gap.insert(slab.gap.end(), 12, 0.0);
    }
    if (!add(slab)) return false;
    if (!wings) continue;
    // Wings: runs of rim sides with a wing, each to one face across, as tubes past the edge.
    std::vector<int> nextOf(rim.size(), -1), prevOf(rim.size(), -1);
    for (size_t i = 0; i < rim.size(); i++) {
      auto it = outOf.find(rim[i].b);
      if (it != outOf.end() && it->second.size() == 1 && into[rim[i].b].size() == 1) nextOf[i] = it->second[0], prevOf[it->second[0]] = (int)i;
    }
    std::vector<char> used(rim.size(), 0);
    auto sameRun = [&](int i, int j) { return j >= 0 && rim[j].wing > 0 && rim[j].next == rim[i].next && rim[j].face == rim[i].face; };
    for (size_t s0 = 0; s0 < rim.size(); s0++) {
      if (used[s0] || rim[s0].wing <= 0) continue;
      // Back to the run's start (or once round, if it closes).
      int start = (int)s0;
      while (sameRun(start, prevOf[start]) && prevOf[start] != (int)s0) start = prevOf[start];
      std::vector<int> run;
      for (int i = start; i >= 0 && !used[i] && (run.empty() || sameRun(run[0], i)); i = nextOf[i]) used[i] = 1, run.push_back(i);
      bool closed = nextOf[run.back()] == run[0] && sameRun(run[0], run.back());
      // Its stations: the run's points, each with how far past the edge the wing reaches (mitred with the side beyond
      // at its ends: the next run's wing, or the slab's rim, a hair more so neighbours overlap).
      std::vector<uint32_t> at;
      std::vector<V3> reach, inward;
      size_t m = run.size();
      for (size_t j = 0; j <= m; j++) {
        if (closed && j == m) break;
        const Half *before = j > 0 ? &rim[run[j - 1]] : (closed ? &rim[run[m - 1]] : nullptr);
        const Half *after = j < m ? &rim[run[j]] : nullptr;
        uint32_t p = after ? after->a : before->b;
        V3 r;
        if (before && after) {
          r = mitre(before->out, before->wing, after->out, after->wing);
        } else {
          const Half &own = before ? *before : *after;
          int beyond = before ? nextOf[run[m - 1]] : prevOf[run[0]];
          if (beyond >= 0) {
            const Half &b = rim[beyond];
            r = mitre(own.out, own.wing, b.out, (b.wing > 0 ? b.wing : b.rim) + eps);
          } else {
            r = own.out * own.wing;
          }
        }
        at.push_back(p), reach.push_back(r);
        inward.push_back(before && after ? unit(before->out + after->out) : (before ? before->out : after->out));
      }
      Solid tube;
      tube.faces.resize(2);
      tube.faces[1].geom.kind = FaceGeom::Curved;
      const FaceGeom &g = S.faces[rim[run[0]].face].geom;
      if (g.flat) tube.faces[0].geom = facingIn(movedForm(g, D, 1));
      else tube.faces[0].geom.kind = FaceGeom::Curved;
      // Each station's ring: inside the edge on the patch (raised, moved in), out past it (moved in, raised).
      std::vector<std::array<uint32_t, 4>> ring;
      for (size_t j = 0; j < at.size(); j++) {
        V3 p = w.pts[pts[at[j]]], n = nv[at[j]], base = p - inward[j] * eps, tip = p + reach[j];
        ring.push_back({tube.vertex(base + n * raise, n), tube.vertex(tip + n * raise, n), tube.vertex(tip - n * D, -n), tube.vertex(base - n * D, -n)});
      }
      auto quad = [&](uint32_t a, uint32_t b, uint32_t c, uint32_t d, int f) {
        tube.triangle(a, b, c, f), tube.triangle(a, c, d, f);
        tube.gap.insert(tube.gap.end(), 12, 0.0);
      };
      size_t ns = ring.size();
      for (size_t j = 0; j + (closed ? 0 : 1) < ns; j++) {
        const auto &r0 = ring[j], &r1 = ring[(j + 1) % ns];
        for (int i = 0; i < 4; i++) quad(r0[i], r0[(i + 1) % 4], r1[(i + 1) % 4], r1[i], i == 2 ? 0 : 1);
      }
      if (!closed) {
        quad(ring[0][3], ring[0][2], ring[0][1], ring[0][0], 1);
        quad(ring[ns - 1][0], ring[ns - 1][1], ring[ns - 1][2], ring[ns - 1][3], 1);
      }
      if (!add(tube)) return false;
    }
  }
  return !parts.empty();
}

// Solids merged into one, two by two (each round's pieces small and many, then fewer and larger). False when a merge goes
// wrong (comes out short of a part of it, or past both together: faces met so near flat the merge can't tell them).
bool mergedAll(std::vector<Solid> parts, Solid &out) {
  if (parts.empty()) return false;
  while (parts.size() > 1) {
    std::vector<Solid> next;
    for (size_t i = 0; i + 1 < parts.size(); i += 2) {
      double a = parts[i].meshVolume(), b = parts[i + 1].meshVolume();
      next.push_back(combine(parts[i], parts[i + 1], BK_UNION));
      double v = next.back().meshVolume(), slack = 1e-6 * (a + b);
      if (next.back().tri.empty() || v < std::max(a, b) - slack || v > a + b + slack) return false;
    }
    if (parts.size() % 2) next.push_back(std::move(parts.back()));
    parts = std::move(next);
  }
  out = std::move(parts[0]);
  return true;
}

// S's walls from its faces: what of S its slabs and wings cover.
bool wallsFrom(const Solid &S, const std::vector<double> &depth, double size, Solid &out) {
  std::vector<Solid> parts;
  Solid U;
  if (!slabsOf(S, depth, size, true, 0, 0, parts) || !mergedAll(std::move(parts), U)) return false;
  out = combine(S, U, BK_INTERSECT);
  double v = out.meshVolume();
  return !out.tri.empty() && v <= std::min(S.meshVolume(), U.meshVolume()) * (1 + 1e-6);
}

// A void kept clear of S's faces by their walls: what of it lies within a wall of a face (a hair short of it, so the
// void's own faces, where the walls end, aren't met face to face) taken off. The void from the tree keeps its corners
// sharp and exact; this takes off where a part's face hidden in a merge, moved out, reached into another part's wall.
bool clearOfWalls(const Solid &S, const std::vector<double> &depth, double size, double d, Solid &hole) {
  std::vector<Solid> parts;
  Solid U;
  if (!slabsOf(S, depth, size, false, 1e-4, d, parts)) return false;
  std::vector<Solid> each = parts;
  if (mergedAll(std::move(parts), U)) {
    Solid cleared = combine(hole, U, BK_SUBTRACT);
    double v = cleared.meshVolume();
    if (!cleared.tri.empty() && v <= hole.meshVolume() * (1 + 1e-6) && v > 0) {
      hole = std::move(cleared);
      return true;
    }
  }
  // The walls not merged into one (slabs side by side meeting face to face): taken out of the void one by one, each
  // only as far as it keeps the void whole.
  bool any = false;
  for (const auto &slab : each) {
    double was = hole.meshVolume(), most = slab.meshVolume(), slack = 1e-6 * (was + most);
    for (double merge : {1e-11, 1e-9, 1e-7}) {
      Solid cleared = combine(hole, slab, BK_SUBTRACT, merge);
      double v = cleared.meshVolume();
      if (cleared.tri.empty() || !(v > 0) || v > was + slack || v < was - most - slack || !shut(cleared)) continue;
      hole = std::move(cleared);
      any = true;
      break;
    }
  }
  return any;
}

}  // namespace

static bool hollowedHere(const Shape &s, const Hollowing &h, double d, Solid &out, int *missing);

// Worked on about the shape's own middle (see middleOf), moved back after.
bool hollowed(const Shape &s, const Hollowing &h, double d, Solid &out, int *missing) {
  V3 lo, hi;
  bounds(s, lo, hi);
  V3 c = middleOf(lo, hi);
  if (c.x == 0 && c.y == 0 && c.z == 0) return hollowedHere(s, h, d, out, missing);
  Affine back = Affine::translation(-c);
  Shape here{s.node, s.place.then(back)};
  Hollowing moved = h;
  std::vector<int> faces(moved.open.size() / 6, BK_PICK_FACE), walls(moved.walls.size() / 6, BK_PICK_FACE);
  movePicks(faces, moved.open, -c), movePicks(walls, moved.walls, -c);
  if (moved.viaSharp) moved.sharp = Shape{moved.sharp.node, moved.sharp.place.then(back)};
  if (!hollowedHere(here, moved, d, out, missing)) return false;
  out.transform(Affine::translation(c));
  return true;
}

static bool hollowedHere(const Shape &s, const Hollowing &h, double d, Solid &out, int *missing) {
  if (missing) *missing = 0;
  Solid whole;
  mesh(s, d, whole);
  if (whole.tri.empty()) return false;
  Rules rules;
  rules.t = std::max(h.thickness, 0.01), rules.d = d;
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (V3 q : whole.p) lo = vmin(lo, q), hi = vmax(hi, q);
  rules.size = std::max(1.0, norm(hi - lo));
  double thickest = rules.t;
  for (double w : h.wallThickness) thickest = std::max(thickest, w);
  rules.out = 2 * thickest + 0.05 * rules.size;
  for (const auto &f : whole.faces) rules.shown.push_back(f.geom);
  int lost = 0;
  // Each face's wall (below zero: opened).
  std::vector<double> depth(whole.faces.size(), rules.t);
  for (size_t i = 0; i + 5 < h.open.size(); i += 6) {
    int f = faceAt(whole, &h.open[i]);
    if (f < 0) lost++;
    // A face meeting no other along any edge is the whole surface of its piece (a ball's, a ring's): opened, nothing of
    // the piece would be left, so it stays shut.
    else if (bounded(whole, f)) rules.open.push_back(whole.faces[f].geom), depth[f] = -1;
  }
  for (size_t i = 0; i + 5 < h.walls.size() && i / 6 < h.wallThickness.size(); i += 6) {
    int f = faceAt(whole, &h.walls[i]);
    if (f < 0) lost++;
    else {
      rules.own.push_back({whole.faces[f].geom, std::max(h.wallThickness[i / 6], 0.01)});
      // An opening before a wall of its own, should a face be picked as both (as OpenCascade's kernel takes it).
      if (depth[f] >= 0) depth[f] = std::max(h.wallThickness[i / 6], 0.01);
    }
  }
  if (missing) *missing = lost;
  // The shape without its roundings hollowed instead: only what lies inside the shape kept.
  if (h.viaSharp) {
    Hollowing plain = h;
    plain.viaSharp = false;
    Solid inner;
    if (!hollowed(h.sharp, plain, d, inner)) return false;
    out = combine(whole, inner, BK_INTERSECT);
    finish(out, d);
    double v = out.meshVolume(), all = whole.meshVolume();
    return shut(out) && v > 0 && v < all * 0.999 && pieces(out) == pieces(whole);
  }
  double all = whole.meshVolume();
  int parts = pieces(whole);
  // Walls from the finished faces.
  auto fromFaces = [&]() {
    Solid made;
    if (!wallsFrom(whole, depth, rules.size, made)) return false;
    finish(made, d);
    double v = made.meshVolume();
    if (!shut(made) || !(v > 0) || !(v < all * 0.999) || pieces(made) != parts) return false;
    out = std::move(made);
    return true;
  };
  // A void from the shape's own tree.
  auto fromTree = [&]() {
    Ctx ctx{rules};
    Solid hole, made;
    if (!voidOf(s, Affine(), 1, false, false, ctx, hole)) return false;
    // Made of others, or opened (an opening moved out runs on past a face it meets inside a corner), the void is kept
    // clear of the walls (as is: when that can't be made, as it is).
    bool cleared = (compound(s) || !rules.open.empty()) && clearOfWalls(whole, depth, rules.size, d, hole);
    made = combine(whole, hole, BK_SUBTRACT);
    finish(made, d);
    // A hair's remnant of a face where the void was cleared just short of a wall: into the face beside it.
    if (cleared && foldThin(made, 0.05 * d)) finish(made, d);
    double v = made.meshVolume(), taken = all - v, inside = hole.meshVolume();
    // (A result left open by a merge that went wrong is no result.)
    if (!shut(made) || !(v > 0) || !(v < all * 0.999) || pieces(made) != parts) return false;
    // Shut, the void must lie wholly inside (walls too thick at a rounding would break through).
    if (rules.open.empty() && taken < inside * (1 - 1e-6) - 1e-9 * rules.size * rules.size * rules.size) return false;
    out = std::move(made);
    return true;
  };
  // By its tree first (exactly, its void's corners sharp), then by its faces.
  return fromTree() || fromFaces();
}

}  // namespace bce
