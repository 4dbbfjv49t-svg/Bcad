// Roundings, inward roundings (coves) and bevels along edges, made the way a workshop would: for each run of picked edges a
// tool of the right cross-section is swept along it and taken away (or, at an inside corner, added). Across the edge the
// tool is exact: its arcs start on the faces exactly where the rounding's circle touches them, and it reaches a little out
// past the faces. Along a straight edge the tool is a prism, round a circle a turned outline stepped as the solid's circle
// is, otherwise a sweep of the cross-section from point to point. At an open end the tool runs on past the face there.
// Straight edges between flat faces rounded outward are made as one piece per group of them, a ball rounding each corner
// where three or more meet; a solid rounded on every edge so is made outright.
#include "Engine/Treat.hpp"
#include "Engine/Triangulate.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <numeric>
#include <set>
#include <tuple>
#include <unordered_map>

namespace bce {

namespace {

constexpr double pi = M_PI;

// A point by its exact position.
struct PKey {
  uint64_t x, y, z;
  bool operator==(const PKey &o) const { return x == o.x && y == o.y && z == o.z; }
  bool operator<(const PKey &o) const { return x != o.x ? x < o.x : y != o.y ? y < o.y : z < o.z; }
};
struct PKeyHash {
  size_t operator()(const PKey &k) const { return (size_t)(k.x * 0x9E3779B97F4A7C15ull ^ (k.y + 0x7F4A7C159E3779B9ull) * 31 ^ k.z * 0xBF58476D1CE4E5B9ull); }
};
PKey pkey(V3 p) {
  PKey k;
  double x = p.x + 0.0, y = p.y + 0.0, z = p.z + 0.0;
  std::memcpy(&k.x, &x, 8), std::memcpy(&k.y, &y, 8), std::memcpy(&k.z, &z, 8);
  return k;
}


V3 p2(double u, double v) { return {u, v, 0}; }
double cross2(V3 a, V3 b) { return a.x * b.y - a.y * b.x; }
double area2(const std::vector<V3> &poly) {
  double a = 0;
  for (size_t i = 0; i < poly.size(); i++) a += cross2(poly[i], poly[(i + 1) % poly.size()]);
  return a / 2;
}

// The point on both lines n1 · p = c1 and n2 · p = c2 (false when they run alongside each other).
bool meet(V3 n1, double c1, V3 n2, double c2, V3 &out) {
  double det = n1.x * n2.y - n1.y * n2.x;
  if (std::fabs(det) < 1e-12) return false;
  out = p2((c1 * n2.y - c2 * n1.y) / det, (n1.x * c2 - n2.x * c1) / det);
  return true;
}

// A tool's cross-section in a plane square to the edge, counter-clockwise: from each run's point to the next one's,
// straight or round an arc (the way through `mid`) — or a whole disc (an inward rounding). `fill`: added to the solid
// (an inside corner) rather than taken away.
struct Section2 {
  struct Run {
    V3 p;
    bool arc = false;
    V3 centre, mid;
    double radius = 0;
    bool axis = false;  // lying along the axis it's turned round (cut off there): no face
  };
  std::vector<Run> runs;
  bool circle = false;
  V3 centre;
  double radius = 0;
  bool fill = false;
};

// An arc run's start angle and its turn (signed) round its middle to the next point q, the way through its `mid`.
void arcTurn(const Section2::Run &r, V3 q, double &a0, double &turn) {
  a0 = std::atan2(r.p.y - r.centre.y, r.p.x - r.centre.x);
  double a1 = std::atan2(q.y - r.centre.y, q.x - r.centre.x), am = std::atan2(r.mid.y - r.centre.y, r.mid.x - r.centre.x);
  turn = std::remainder(a1 - a0, 2 * pi);
  double half = std::remainder(am - a0, 2 * pi);
  if (turn > 0 ? !(half > 0 && half < turn) : !(half < 0 && half > turn)) turn += turn > 0 ? -2 * pi : 2 * pi;
}

// Chords an arc takes for a chord error of d (and at least one per 20°).
int chordsFor(double turn, double r, double d) {
  int k = std::max(1, (int)std::ceil(std::fabs(turn) / (2 * std::acos(std::clamp(1 - d / r, -1.0, 1.0))) - 1e-9));
  return std::min(std::max(k, (int)std::ceil(std::fabs(turn) / 0.35 - 1e-9)), 64);
}

// Each run's chords (none for a straight one; for a disc, its one entry).
std::vector<int> chordsOf(const Section2 &s, double d) {
  if (s.circle) return {std::max(8, chordsFor(2 * pi, s.radius, d))};
  std::vector<int> k(s.runs.size(), 0);
  for (size_t i = 0; i < s.runs.size(); i++) {
    if (!s.runs[i].arc) continue;
    double a0, turn;
    arcTurn(s.runs[i], s.runs[(i + 1) % s.runs.size()].p, a0, turn);
    k[i] = chordsFor(turn, s.runs[i].radius, d);
  }
  return k;
}

// The outline as points, arcs as `chords` chords each, and for each point the run its side to the next lies in.
std::vector<V3> pointsOf(const Section2 &s, const std::vector<int> &chords, std::vector<int> *runOf = nullptr) {
  std::vector<V3> out;
  if (runOf) runOf->clear();
  if (s.circle) {
    // Turned by an odd part of a step, so no corner lands on a face through the edge (a side lying in a face's plane).
    for (int j = 0; j < chords[0]; j++) {
      double a = 2 * pi * (j + 0.381966) / chords[0];
      out.push_back(s.centre + p2(std::cos(a), std::sin(a)) * s.radius);
      if (runOf) runOf->push_back(0);
    }
    return out;
  }
  size_t n = s.runs.size();
  for (size_t i = 0; i < n; i++) {
    const auto &r = s.runs[i];
    out.push_back(r.p);
    if (runOf) runOf->push_back((int)i);
    if (!r.arc) continue;
    double a0, turn;
    arcTurn(r, s.runs[(i + 1) % n].p, a0, turn);
    for (int j = 1; j < chords[i]; j++) {
      double a = a0 + turn * j / chords[i];
      out.push_back(r.centre + p2(std::cos(a), std::sin(a)) * r.radius);
      if (runOf) runOf->push_back((int)i);
    }
  }
  return out;
}

void ccw(Section2 &s) {
  if (s.circle || area2(pointsOf(s, chordsOf(s, 1e9))) >= 0) return;
  size_t n = s.runs.size();
  std::vector<Section2::Run> back(n);
  for (size_t j = 0; j < n; j++) {
    back[j] = s.runs[(2 * n - 2 - j) % n];  // the side the other way
    back[j].p = s.runs[n - 1 - j].p;
  }
  s.runs = back;
}

// The material angle at a corner (face A along da, outward normal na; face B along db), radians: from da round through the
// material to db.
double cornerAngle(V3 da, V3 na, V3 db) {
  double u = dot(db, da), v = dot(db, na);
  double phi = std::atan2(-v, u);
  return phi <= 0 ? phi + 2 * pi : phi;
}

// How far a tool reaches out past the faces (each tool a little differently — `which` — so no two lie in one plane where
// they overlap), and how far short of an end with nothing to run onto it stops.
double reachOut(double size, int which = 0) { return std::max(0.3 * size, 1e-3) * (1 + 0.083 * (which % 7)); }
double hair(double r) { return std::max(2e-3 * r, 1e-6); }

// Rounding the corner at e (face A from e along da, outward normal na; face B along db) with radius r: where its circle
// meets face A and face B, its middle, and whether the corner is convex (the circle in the material).
struct Touch {
  V3 a, b, centre;
  bool convex;
};
Touch touchAt(V3 e, V3 da, V3 na, V3 db, double r) {
  V3 dB = unit(p2(dot(db, da), dot(db, na)));
  double phi = cornerAngle(p2(1, 0), p2(0, 1), dB);
  bool convex = phi < pi;
  double half = (convex ? phi : 2 * pi - phi) / 2, t = r / std::tan(half);
  V3 c = unit(p2(1, 0) + dB) * (r / std::sin(half));
  return {e + da * t, e + db * t, e + da * c.x + na * c.y, convex};
}

// A straight run from p.
Section2::Run lineRun(V3 p) {
  Section2::Run run;
  run.p = p;
  return run;
}

// The corner's arc as a run from where it meets face B round to where it meets face A, past the corner.
Section2::Run arcRun(const Touch &k, V3 from, V3 corner, double r) {
  Section2::Run run;
  run.p = from, run.arc = true, run.centre = k.centre, run.radius = r, run.mid = k.centre + unit(corner - k.centre) * r;
  return run;
}

// The outer corner of a tool reaching out by m past face A (normal na) and face B (normal nb) at e.
V3 outerCorner(V3 e, V3 na, V3 nb, double m) {
  V3 O;
  if (!meet(na, dot(na, e) + m, nb, dot(nb, e) + m, O)) O = e + na * m;
  return O;
}

// Rounding the corner at e: at an outside corner the part to take away (outside the circle, reaching out past both
// faces), at an inside one the part to add (reaching into the material).
Section2 roundCorner(V3 e, V3 da, V3 na, V3 db, V3 nb, double r, int which) {
  Touch k = touchAt(e, da, na, db, r);
  double m = reachOut(r, which) * (k.convex ? 1 : -1);
  Section2 out;
  out.fill = !k.convex;
  out.runs = {lineRun(k.a), lineRun(k.a + na * m), lineRun(outerCorner(e, na, nb, m)), lineRun(k.b + nb * m), arcRun(k, k.b, e, r)};
  ccw(out);
  return out;
}

// A face as cut through a ring's axis at E, in the section's frame (x along U, y along N, origin E): straight (through E
// along `dir`, outward `n`) or, for a face turned from an arc, that arc's circle (centre q, radius rad; `away` when its
// outward normal points away from the centre). Points `by` out from it along its outward normal: a line, or a circle
// rad ± by.
struct Cut2 {
  bool circle = false;
  V3 dir, n, q;
  double rad = 0;
  bool away = true;
};

Cut2 cutOf(const FaceGeom &g, V3 E, V3 U, V3 N, V3 dir2, V3 n2) {
  Cut2 c;
  c.dir = dir2, c.n = n2;
  if (g.flat || g.kind != FaceGeom::Turned || !g.exact || !g.elem.arc) return c;
  V3 L = g.place.inverse().point(E);
  double lr = std::hypot(L.x, L.y);
  if (lr < 1e-12) return c;
  V3 Q = g.place.point(V3{L.x / lr * g.elem.cr, L.y / lr * g.elem.cr, g.elem.cz});
  c.circle = true, c.q = p2(dot(Q - E, U), dot(Q - E, N)), c.rad = g.elem.rad * norm(g.place.vector({1, 0, 0}));
  c.away = dot(n2, p2(0, 0) - c.q) > 0;
  return c;
}

// A face as cut square to an edge at E (end-on frame U, N; its way in from the corner and outward normal there, `dir2`,
// `n2`), curved in the cut: the circle it bends round there (exactly, for a ball, or a cylinder cut square to its axis;
// else the one it touches, by how much it bends that way — Euler's sum of its bends round its axis and along its
// profile). `out` (its outward normal there) says which way the material lies. A line where it doesn't bend that way.
Cut2 bendOf(const FaceGeom &g, V3 E, V3 U, V3 N, V3 dir2, V3 n2, V3 out) {
  Cut2 c;
  c.dir = dir2, c.n = n2;
  double scale;
  if (g.flat || g.kind != FaceGeom::Turned || !g.exact || !g.place.similarity(&scale)) return c;
  V3 a = unit(g.place.vector({0, 0, 1})), o = g.place.point({0, 0, 0});
  // On the face itself (E, a mesh point, lies a chord's sag inside a convex one).
  V3 L = g.place.inverse().point(E);
  double lr = std::hypot(L.x, L.y), pr, pz;
  if (lr < 1e-12) return c;
  double t = g.elem.nearest(lr, L.z), nr, nz;
  g.elem.at(t, pr, pz);
  g.elem.normalAt(t, nr, nz);
  V3 on = g.place.point({L.x / lr * pr, L.y / lr * pr, pz});
  // Its normal there (the way `out` faces: a part taken away has its faces turned in).
  V3 n = g.place.normal({nr * L.x / lr, nr * L.y / lr, nz});
  out = dot(n, out) < 0 ? -n : n;
  V3 radial = (on - o) - a * dot(on - o, a);
  double far = norm(radial);
  if (far < 1e-9 * (1 + norm(on - o))) return c;
  V3 ru = radial / far, round = cross(a, ru), way = unit(U * dir2.x + N * dir2.y);
  // Round the axis (a circle about it), and along the profile (straight, or the arc's).
  double kRound = dot(ru, out) / far, kAlong = 0;
  if (g.elem.arc) {
    V3 centre = g.place.point(V3{L.x / lr * g.elem.cr, L.y / lr * g.elem.cr, g.elem.cz});
    kAlong = dot(unit(on - centre), out) / (scale * g.elem.rad);
  }
  double cr = dot(way, round), k = kRound * cr * cr + kAlong * (1 - cr * cr);
  if (std::fabs(k) < 1e-9) return c;
  V3 q3 = on - out / k;
  c.circle = true, c.q = p2(dot(q3 - E, U), dot(q3 - E, N)), c.rad = 1 / std::fabs(k);
  c.away = dot(n2, p2(0, 0) - c.q) > 0;
  return c;
}

// Where a bevel's leg `len` long from the corner (the origin) ends on a face as cut: along a straight one, on a curved
// one's circle (the way it runs into the face).
bool legOn(const Cut2 &F, double len, V3 &out) {
  if (!F.circle) {
    out = F.dir * len;
    return true;
  }
  // |P| = len and |P - q| = rad: on the line both circles share, len²/2 from the origin along q (the origin is on the face's
  // circle: |q| = rad).
  double qq = dot(F.q, F.q);
  if (!(qq > 0) || len >= 2 * std::sqrt(qq)) return false;
  V3 u = F.q / std::sqrt(qq), v = p2(-u.y, u.x);
  double x = len * len / (2 * std::sqrt(qq)), h2 = len * len - x * x;
  if (h2 < 0) return false;
  V3 P = u * x + v * std::sqrt(h2), Q = u * x - v * std::sqrt(h2);
  out = dot(P, F.dir) >= dot(Q, F.dir) ? P : Q;
  return dot(out, F.dir) > 0;
}

// The rounding's circle with faces as cut (`A`, `B`: lines or circles; the corner at the origin): its middle where both
// faces moved into the material by r meet (out of it at an inside corner), where it touches each face and the face's
// outward normal there; false when it can't touch both.
bool rollAt(const Cut2 &A, const Cut2 &B, double r, V3 &C, V3 &TA, V3 &TB, V3 &nA, V3 &nB, bool &convex) {
  V3 e = p2(0, 0), da = A.dir, na = A.n, db = B.dir;
  Touch k = touchAt(e, da, na, db, r);
  convex = k.convex;
  double by = k.convex ? -r : r;
  // Candidates for the middle: on both moved faces.
  std::vector<V3> cand;
  auto lineCircle = [&](const Cut2 &L, const Cut2 &C) {
    double rr = C.rad + (C.away ? by : -by);
    V3 foot = L.n * by, w = foot - C.q;
    double bq = dot(L.dir, w), cq = dot(w, w) - rr * rr, disc = bq * bq - cq;
    if (!(rr > 0) || disc < 0) return;
    for (double sg : {-1.0, 1.0}) cand.push_back(foot + L.dir * (-bq + sg * std::sqrt(disc)));
  };
  if (A.circle && B.circle) {
    double ra = A.rad + (A.away ? by : -by), rb = B.rad + (B.away ? by : -by);
    V3 dq = B.q - A.q;
    double dd = norm(dq);
    if (!(ra > 0 && rb > 0) || dd < 1e-12) return false;
    double x = (dd * dd + ra * ra - rb * rb) / (2 * dd), h2 = ra * ra - x * x;
    if (h2 < 0) return false;
    V3 u = dq / dd, v = p2(-u.y, u.x);
    for (double sg : {-1.0, 1.0}) cand.push_back(A.q + u * x + v * (sg * std::sqrt(h2)));
  } else if (A.circle) {
    lineCircle(B, A);
  } else if (B.circle) {
    lineCircle(A, B);
  } else {
    return false;
  }
  if (cand.empty()) return false;
  C = cand[0];
  for (V3 q : cand)
    if (norm(q - k.centre) < norm(C - k.centre)) C = q;
  // Where it touches each face, and the face's outward normal there.
  auto touch = [&](const Cut2 &F, V3 &T, V3 &n) {
    if (!F.circle) {
      T = C - F.n * by, n = F.n;
      return;
    }
    V3 u = unit(C - F.q);
    T = F.q + u * F.rad, n = F.away ? u : -u;
  };
  touch(A, TA, nA), touch(B, TB, nB);
  return true;
}

// How far along a face as cut a point on it lies from the corner (the origin): along its line, or round its circle.
double alongCut(const Cut2 &F, V3 T) {
  if (!F.circle) return norm(T);
  V3 a = unit(p2(0, 0) - F.q), b = unit(T - F.q);
  return F.rad * std::acos(std::clamp(dot(a, b), -1.0, 1.0));
}

// Rounding the corner at the origin where a face is curved in the cut: the circle as rollAt finds it, the rest as
// roundCorner. False when it can't touch both faces.
bool roundCurved(const Cut2 &A, const Cut2 &B, double r, int which, Section2 &out) {
  V3 e = p2(0, 0), na = A.n, nb = B.n, C, TA, TB, nA, nB;
  bool convex;
  if (!rollAt(A, B, r, C, TA, TB, nA, nB, convex)) return false;
  double m = reachOut(r, which) * (convex ? 1 : -1);
  Touch exact{TA, TB, C, convex};
  out = Section2();
  out.fill = !convex;
  out.runs = {lineRun(TA), lineRun(TA + nA * m), lineRun(outerCorner(e, na, nb, m)), lineRun(TB + nB * m), arcRun(exact, TB, e, r)};
  ccw(out);
  return true;
}

// Bevelling the corner at e: legs la along face A and lb along face B; taken away at an outside corner, added at an inside
// one. With `soft` above zero its two edges with the faces rounded too.
Section2 bevelCorner(V3 e, V3 da, V3 na, V3 db, V3 nb, double la, double lb, double soft, int which, bool &fits, const Cut2 *A = nullptr,
                     const Cut2 *B = nullptr, bool straightOut = false, int convexity = 0) {
  bool convex = convexity ? convexity > 0 : cornerAngle(da, na, db) < pi;
  double m = reachOut(std::max(la, lb), which) * (convex ? 1 : -1);
  Section2 out;
  out.fill = !convex;
  fits = true;
  // Each leg's end on its face, and the face's way on and outward normal there: along a straight face; on a face curved in
  // the cut (`A`, `B`; the corner at the origin), that far from the corner in a straight line, the face's tangent there.
  V3 PA = e + da * la, PB = e + db * lb, ta = da, tna = na, tb = db, tnb = nb;
  auto onCurve = [&](const Cut2 *F, double len, V3 &P, V3 &t, V3 &n) {
    if (!F || !F->circle) return true;
    if (!legOn(*F, len, P)) return false;
    n = unit(P - F->q) * (F->away ? 1.0 : -1.0);
    t = p2(-n.y, n.x);
    if (dot(t, P) < 0) t = -t;
    return true;
  };
  if (!onCurve(A, la, PA, ta, tna) || !onCurve(B, lb, PB, tb, tnb)) return fits = false, out;
  V3 dP = unit(PB - PA), nP = p2(-dP.y, dP.x);
  if (dot(nP, e - PA) < 0) nP = -nP;
  V3 O = outerCorner(e, na, nb, m);
  // The bevel's line run on past each face, out to the tool's sides.
  auto past = [&](V3 from, V3 dir, V3 n) { return from + dir * (m / dot(dir, n)); };
  if (!(soft > 0.005) || !convex) {
    // Where the faces meet nearly flat (a seam between two roundings, at its end) the line lies almost along them, and
    // runs out far before it clears them: straight out from each leg's end instead.
    // (Or asked for, so sections swept along a run all have the same sides.)
    if (!straightOut && std::fabs(dot(unit(PA - PB), na)) > 0.2 && std::fabs(dot(unit(PB - PA), nb)) > 0.2)
      out.runs = {lineRun(past(PA, PA - PB, na)), lineRun(O), lineRun(past(PB, PB - PA, nb))};
    else
      out.runs = {lineRun(PA), lineRun(PA + na * m), lineRun(O), lineRun(PB + nb * m), lineRun(PB)};
  } else {
    Touch kA = touchAt(PA, ta, tna, dP, soft), kB = touchAt(PB, -dP, nP, tb, soft);
    fits = dot(kB.a - kA.b, dP) > 0;
    out.runs = {lineRun(kA.a), lineRun(kA.a + tna * m), lineRun(O), lineRun(kB.b + tnb * m), arcRun(kB, kB.b, PB, soft), lineRun(kB.a),
                arcRun(kA, kA.b, PA, soft)};
  }
  ccw(out);
  return out;
}

// The sections a treatment takes at one point of a crease, in its end-on frame (u into face A, v face A's outward normal);
// `fits` false when a softened bevel's roundings would cross.
std::vector<Section2> sectionsAt(const Crease &c, size_t i, const Treatment &t, int which, bool *fits = nullptr, bool straightOut = false,
                                 int convexity = 0, const Solid *solid = nullptr) {
  V3 U = c.ia[i], N = unit(c.na[i] - U * dot(c.na[i], U));
  V3 da = p2(1, 0), na = p2(0, 1), db = unit(p2(dot(c.ib[i], U), dot(c.ib[i], N))), nb = unit(p2(dot(c.nb[i], U), dot(c.nb[i], N)));
  bool convex = cornerAngle(da, na, db) < pi;
  std::vector<Section2> out;
  switch (t.kind) {
  case Treatment::Round:
    // Faces curved in the cut (given the solid): the circle touching them as they bend, not their tangents.
    if (solid) {
      Cut2 A = bendOf(solid->faces[c.fa[i]].geom, c.pts[i], U, N, da, na, c.na[i]), B = bendOf(solid->faces[c.fb[i]].geom, c.pts[i], U, N, db, nb, c.nb[i]);
      Section2 exact;
      if ((A.circle || B.circle) && roundCurved(A, B, t.radius, which, exact)) {
        out.push_back(exact);
        break;
      }
    }
    out.push_back(roundCorner(p2(0, 0), da, na, db, nb, t.radius, which));
    break;
  case Treatment::Cove:
    if (convex) {
      Section2 s;
      s.circle = true, s.centre = p2(0, 0), s.radius = t.radius;
      out.push_back(s);
    }
    break;
  case Treatment::Bevel: {
    bool ok;
    out.push_back(bevelCorner(p2(0, 0), da, na, db, nb, t.legA, t.legB, t.corner, which, ok, nullptr, nullptr, straightOut, convexity));
    if (fits) *fits = ok;
    break;
  }
  }
  return out;
}

// MARK: - tools

// A cross-section's points from a frame (E, U, N) into the world.
V3 at3(V3 E, V3 U, V3 N, V3 q) { return E + U * q.x + N * q.y; }

// A closed outline's inside as triangles (point numbers), by a triangulation keeping its sides.
std::vector<int> capOf(const std::vector<V3> &pts) {
  double lo[2] = {INFINITY, INFINITY}, hi[2] = {-INFINITY, -INFINITY};
  for (V3 q : pts) lo[0] = std::min(lo[0], q.x), lo[1] = std::min(lo[1], q.y), hi[0] = std::max(hi[0], q.x), hi[1] = std::max(hi[1], q.y);
  double w = std::max(hi[0] - lo[0], hi[1] - lo[1]) + 1, cx = (lo[0] + hi[0]) / 2, cy = (lo[1] + hi[1]) / 2;
  Tri2 tri(cx - 4 * w, cy - 3 * w, cx + 4 * w, cy - 3 * w, cx, cy + 5 * w);
  std::vector<int> id(pts.size());
  std::unordered_map<int, int> back;
  for (size_t j = 0; j < pts.size(); j++) id[j] = tri.insert(pts[j].x, pts[j].y), back[id[j]] = (int)j;
  for (size_t j = 0; j < pts.size(); j++) tri.keep(id[j], id[(j + 1) % pts.size()]);
  std::vector<int> out;
  for (int q : tri.insideKept()) {
    auto it = back.find(q);
    if (it == back.end()) return {};
    out.push_back(it->second);
  }
  return out;
}

// The arc of a circle (2D) from p to q the way that passes nearest `near`, as a turned outline's piece.
Elem arcThrough(V3 c, double r, V3 p, V3 q, V3 near) {
  double a0 = std::atan2(p.y - c.y, p.x - c.x), a1 = std::atan2(q.y - c.y, q.x - c.x), d = a1 - a0;
  while (d > pi) d -= 2 * pi;
  while (d < -pi) d += 2 * pi;
  double mid = a0 + d / 2, want = std::atan2(near.y - c.y, near.x - c.x);
  double off = std::fabs(std::remainder(mid - want, 2 * pi)), other = std::fabs(std::remainder(mid + pi - want, 2 * pi));
  if (other < off) d += d > 0 ? -2 * pi : 2 * pi;
  Elem e = Elem::arcOf(c.x, c.y, r, a0, a0 + d);
  e.r0 = p.x, e.z0 = p.y, e.r1 = q.x, e.z1 = q.y;
  return e;
}

// The solid of a section turned round an axis (a crease running round a circle): the section's points taken into the
// half-plane of (distance from the axis, height along it); false when it would reach across the axis.
bool ringTool(const Section2 &sec, V3 E, V3 U, V3 N, V3 centre, V3 axis, int around, V3 x0, double d, Solid &out) {
  // Signed distance from the axis (towards the crease's side), so a point past the axis shows as one. The frame's
  // directions in that half-plane are taken square where they are within rounding of it, so points on the faces land
  // on them exactly.
  V3 radial = (E - centre) - axis * dot(E - centre, axis);
  double R = norm(radial);
  if (R < 1e-12) return false;
  radial = radial / R;
  auto snap = [](double x) { return std::fabs(x) < 1e-12 ? 0.0 : std::fabs(x - 1) < 1e-12 ? 1.0 : std::fabs(x + 1) < 1e-12 ? -1.0 : x; };
  double Ur = snap(dot(U, radial)), Uz = snap(dot(U, axis)), Nr = snap(dot(N, radial)), Nz = snap(dot(N, axis));
  double z0 = dot(E - centre, axis);
  if (std::fabs(z0) < 1e-12 * (1 + R)) z0 = 0;
  auto rh = [&](V3 q2) { return p2(R + q2.x * Ur + q2.y * Nr, z0 + q2.x * Uz + q2.y * Nz); };
  std::vector<Elem> profile;
  if (sec.circle) {
    V3 c = rh(sec.centre);
    if (c.x <= sec.radius * 1.0001) return false;
    profile = {Elem::arcOf(c.x, c.y, sec.radius, 0, 2 * pi)};
  } else {
    size_t n = sec.runs.size();
    for (size_t i = 0; i < n; i++) {
      const auto &r = sec.runs[i];
      V3 p = rh(r.p), q = rh(sec.runs[(i + 1) % n].p);
      // On the axis (a section cut off there) within rounding: on it.
      if (std::fabs(p.x) < 1e-9 * (1 + R)) p.x = 0;
      if (std::fabs(q.x) < 1e-9 * (1 + R)) q.x = 0;
      if (p.x < 0) return false;
      if (!r.arc) {
        profile.push_back(Elem::line(p.x, p.y, q.x, q.y));
        continue;
      }
      // Only the arc itself must stay off the axis (its circle may cross it: a rounding wider than a small disc's middle).
      Elem e = arcThrough(rh(r.centre), r.radius, p, q, rh(r.mid));
      for (int k = 0; k <= 32; k++) {
        double er, ez;
        e.at(k / 32.0, er, ez);
        if (er < -1e-9 * (1 + R)) return false;
      }
      profile.push_back(e);
    }
  }
  // Counter-clockwise round its region (the area of its outline, arcs by their chords and bulge).
  double area = 0;
  for (const auto &e : profile) {
    if (!e.arc) {
      area += e.r0 * e.z1 - e.r1 * e.z0;
    } else {
      for (int k = 0; k < 64; k++) {
        double r0, z0, r1, z1;
        e.at(k / 64.0, r0, z0), e.at((k + 1) / 64.0, r1, z1);
        area += r0 * z1 - r1 * z0;
      }
    }
  }
  if (area < 0) {
    std::reverse(profile.begin(), profile.end());
    for (auto &e : profile) {
      std::swap(e.r0, e.r1), std::swap(e.z0, e.z1);
      if (e.arc) std::swap(e.a0, e.a1);
    }
  }
  // Its steps round the axis those of the solid's circle (`around` of them, one at x0), so where it meets the solid's
  // faces along a circle it meets them point for point.
  Affine f;
  V3 x = around >= 3 ? x0 : unit(std::fabs(axis.x) < 0.9 ? cross(axis, V3{1, 0, 0}) : cross(axis, V3{0, 1, 0})), y = cross(axis, x);
  f.m[0] = x.x, f.m[1] = y.x, f.m[2] = axis.x, f.m[3] = centre.x;
  f.m[4] = x.y, f.m[5] = y.y, f.m[6] = axis.y, f.m[7] = centre.y;
  f.m[8] = x.z, f.m[9] = y.z, f.m[10] = axis.z, f.m[11] = centre.z;
  auto model = turnedModel(profile);
  model->around = around;
  mesh(shapeOf(model, f), d, out);
  return true;
}

// A cross-section swept from point to point of a run (each point its own frame and outline, all with as many points),
// its ends closed. A face for each of the outline's runs (`runOf`: each side's run; an arc's chords one face), else each
// side its own. With `radii` (at each point, each side's arc's radius as radiiOf gives it), what each triangle's chords
// miss of the arcs they cut across (Solid::gap).
Solid sweptTool(const std::vector<V3> &E, const std::vector<V3> &U, const std::vector<V3> &N, const std::vector<std::vector<V3>> &outlines,
                bool closed, const std::vector<int> &runOf = {}, const std::vector<std::vector<double>> *radii = nullptr) {
  Solid out;
  size_t n = E.size(), k = outlines[0].size();
  for (size_t i = 0; i < n; i++)
    for (size_t j = 0; j < k; j++) out.p.push_back(at3(E[i], U[i], N[i], outlines[i][j]));
  out.n.assign(out.p.size(), V3{});
  size_t rows = closed ? n : n - 1;
  auto id = [&](size_t i, size_t j) { return (uint32_t)((i % n) * k + j % k); };
  bool byRun = runOf.size() == k;
  size_t sides = byRun ? (size_t)*std::max_element(runOf.begin(), runOf.end()) + 1 : k;
  for (size_t j = 0; j < sides; j++) {
    Solid::Face f;
    out.faces.push_back(f);
  }
  std::vector<double> gap;
  // Outward: an arc bulging out of the tool beyond its chord, less than zero where it curves in.
  auto sag = [&](size_t i, size_t j) {
    if (!radii || (*radii)[i % n].size() != k) return 0.0;
    double r = std::fabs((*radii)[i % n][j]), c = norm(outlines[i % n][(j + 1) % k] - outlines[i % n][j]);
    double h = r > 0 ? r - std::sqrt(std::max(0.0, r * r - c * c / 4)) : 0.0;
    return (*radii)[i % n][j] > 0 ? h : -h;
  };
  for (size_t i = 0; i < rows; i++)
    for (size_t j = 0; j < k; j++) {
      uint32_t a = id(i, j), b = id(i, j + 1), c = id(i + 1, j + 1), dd = id(i + 1, j);
      int face = byRun ? runOf[j] : (int)j;
      out.triangle(a, b, c, face);
      out.triangle(a, c, dd, face);
      // Across the arc only (along the run its chords are straight): at its sides' middles, the arc's sag there.
      double s0 = sag(i, j), s1 = sag(i + 1, j);
      gap.insert(gap.end(), {0, 0, 0, s0, 0, (s0 + s1) / 2});
      gap.insert(gap.end(), {0, 0, 0, (s0 + s1) / 2, s1, 0});
    }
  if (!closed) {
    for (int end = 0; end < 2; end++) {
      size_t i = end == 0 ? 0 : n - 1;
      Solid::Face f;
      out.faces.push_back(f);
      int fid = (int)out.faces.size() - 1;
      std::vector<int> cap = capOf(outlines[i]);
      for (size_t q = 0; q + 2 < cap.size(); q += 3) {
        if (end == 0) out.triangle(id(i, cap[q]), id(i, cap[q + 2]), id(i, cap[q + 1]), fid);
        else out.triangle(id(i, cap[q]), id(i, cap[q + 1]), id(i, cap[q + 2]), fid);
        gap.insert(gap.end(), 6, 0.0);
      }
    }
  }
  // Turned outward, and each face's normals and plane.
  if (out.meshVolume() < 0)
    for (size_t t = 0; t < out.tri.size(); t += 3) {
      std::swap(out.tri[t + 1], out.tri[t + 2]);
      // Corners 0, 2, 1: the midpoints 0–2, 2–1, 1–0.
      std::swap(gap[2 * t + 1], gap[2 * t + 2]), std::swap(gap[2 * t + 3], gap[2 * t + 5]);
    }
  // Every triangle its own points (unwelded, as the engine's meshes are) with its flat normal.
  Solid flat;
  flat.faces = out.faces;
  for (size_t t = 0; t < out.triFace.size(); t++) {
    V3 a = out.p[out.tri[3 * t]], b = out.p[out.tri[3 * t + 1]], c = out.p[out.tri[3 * t + 2]], nn = unit(cross(b - a, c - a));
    uint32_t base = (uint32_t)flat.p.size();
    flat.vertex(a, nn), flat.vertex(b, nn), flat.vertex(c, nn);
    flat.triangle(base, base + 1, base + 2, (int)out.triFace[t]);
  }
  for (auto &f : flat.faces) f.geom.kind = FaceGeom::Curved;
  flat.centroids();
  flat.gap = gap.size() == 6 * flat.triFace.size() ? gap : std::vector<double>(6 * flat.triFace.size(), 0);
  for (size_t t = 0; t < flat.triFace.size(); t++) flat.faces[flat.triFace[t]].deficit += flat.sliver(t);
  return flat;
}

// Each side of an outline (as pointsOf made it, `runOf` each side's run): its arc's radius, 0 where straight; below zero
// as the arc's centre lies outside the tool (a rounding's arc, which curves in from its chords). A cove's disc is left
// at 0: what its chords miss, the cut it makes in the faces' meshes makes up (over random shapes its volumes match
// OpenCascade's within its meshes' error so, and are off by more with its chords' slivers counted).
std::vector<double> radiiOf(const Section2 &sec, const std::vector<int> &runOf) {
  std::vector<double> out(runOf.size(), 0);
  if (sec.circle) return out;
  for (size_t j = 0; j < runOf.size(); j++)
    if (runOf[j] >= 0 && runOf[j] < (int)sec.runs.size() && sec.runs[runOf[j]].arc) out[j] = -sec.runs[runOf[j]].radius;
  return out;
}

// MARK: - runs, ends and corners

// Faces with a triangle at point p.
std::vector<int> facesAt(const Solid &s, V3 p) {
  std::vector<int> out;
  for (size_t c = 0; c < s.tri.size(); c++)
    if (s.p[s.tri[c]] == p) {
      int f = (int)s.triFace[c / 3];
      if (std::find(out.begin(), out.end(), f) == out.end()) out.push_back(f);
    }
  return out;
}

// Whether a mesh is closed: every side of a triangle met by one running the other way (points by position).
bool closed(const Solid &s) {
  std::unordered_map<PKey, uint32_t, PKeyHash> id;
  std::vector<uint32_t> at(s.p.size());
  for (size_t i = 0; i < s.p.size(); i++) at[i] = id.emplace(pkey(s.p[i]), (uint32_t)id.size()).first->second;
  std::unordered_map<uint64_t, int> sides;
  for (size_t t = 0; t < s.tri.size(); t += 3)
    for (int k = 0; k < 3; k++) {
      uint32_t a = at[s.tri[t + k]], b = at[s.tri[t + (k + 1) % 3]];
      if (a == b) return false;
      sides[(uint64_t)a << 32 | b]++;
    }
  for (auto [k, n] : sides) {
    auto o = sides.find(k << 32 | k >> 32);
    if (o == sides.end() || o->second != n) return false;
  }
  return true;
}

// How far each face runs straight from the crease at point i, seen in the cut across it (until its outline turns by more
// than 30°), and whether that run ends on one of `treated` (so the face is shared between two treatments).
struct Runs {
  double a = INFINITY, b = INFINITY;
  bool aShared = false, bShared = false;
  // Where shared: how far the other crease's rounding reaches along this run, for a radius of 1 (cot of its half angle,
  // over the sine at which the run meets it).
  double aOther = 1, bOther = 1;
  // Whether a rounding may run on past the face's end (spill over): the face ends at a corner turning into the material,
  // and the outline beyond keeps to the material's side of the face's plane as far as the tool reaches (what it reaches
  // past the face there is all air; it ends on the next face as that face cuts across it). And that next face's line
  // in the cut (its start, and its way on), to see it does.
  bool aOpen = false, bOpen = false;
  V3 aNext, aWay, bNext, bWay;
  // The next face's number (where the cut knows it), else -1.
  int aNextFace = -1, bNextFace = -1;
};

double distanceTo(const std::vector<V3> &pts, V3 q) {
  double best = INFINITY;
  for (size_t i = 0; i + 1 < pts.size(); i++) {
    V3 a = pts[i], dv = pts[i + 1] - pts[i];
    double l2 = norm2(dv), t = l2 > 0 ? std::clamp(dot(q - a, dv) / l2, 0.0, 1.0) : 0;
    best = std::min(best, norm(a + dv * t - q));
  }
  return best;
}

Runs runsAt(const Solid &s, const Crease &c, size_t i, const std::vector<Crease> &treated, double tol, const Crease *self = nullptr,
            double radius = 0, bool byFace = true) {
  Runs out;
  std::vector<std::vector<int>> faceOf;
  auto loops = sliceAcross(s, c, i, &faceOf);
  // The loop and point nearest the edge.
  const std::vector<std::pair<double, double>> *loop = nullptr;
  const std::vector<int> *sides = nullptr;
  size_t k = 0;
  double best = INFINITY;
  for (size_t q = 0; q < loops.size(); q++)
    for (size_t j = 0; j < loops[q].size(); j++) {
      double d = std::hypot(loops[q][j].first, loops[q][j].second);
      if (d < best) best = d, loop = &loops[q], sides = q < faceOf.size() ? &faceOf[q] : nullptr, k = j;
    }
  if (!loop || loop->size() < 3) return out;
  const auto &L = *loop;
  size_t n = L.size();
  V3 E = c.pts[i], X = -c.ia[i], Y = unit(c.na[i] - X * dot(c.na[i], X));
  // As far as the face it leaves along runs (where the cut knows its sides' faces), else until it has turned by 30°.
  size_t jEnd = k;
  V3 lastHeading;
  auto run = [&](int step, V3 &dir, V3 &end, int &face) {
    double length = 0, turned = 0;
    V3 heading{};
    bool have = false;
    face = -1;
    size_t j = k;
    for (size_t g = 0; g < n; g++) {
      size_t nx = (j + n + step) % n;
      int f = byFace && sides && sides->size() == n ? (*sides)[step > 0 ? j : nx] : -1;
      if (g == 0) face = f;
      V3 d = p2(L[nx].first - L[j].first, L[nx].second - L[j].second);
      double l = norm(d);
      if (l > 1e-12) {
        V3 h = d / l;
        double turn = have ? std::acos(std::clamp(dot(heading, h), -1.0, 1.0)) : 0;
        // Onto another face: on only where they meet smoothly (a rounding going on from it: its mesh's first chord turns
        // by as much as half a step round it).
        if (face >= 0 && f != face) {
          bool rounding = (f >= 0 && s.faces[f].blend) || s.faces[face].blend;
          if (turn > (rounding ? pi / 6 : 5 * pi / 180)) break;
          face = f;
        }
        turned += turn;
        if (face < 0 && turned > pi / 6) break;
        if (!have) dir = h;
        heading = h, have = true, length += l;
      }
      j = nx;
    }
    end = E + X * L[j].first + Y * L[j].second;
    jEnd = j, lastHeading = heading;
    return length;
  };
  // Past a run's end (at point j, coming in heading `h`; `out` its face's outward normal in the cut): the next side turns
  // into the material, and the outline keeps behind the face's plane as far as `within` from the edge.
  auto open = [&](int step, size_t j, V3 h, V3 out, double within, V3 &at, V3 &way, int &nextFace) {
    size_t nx = (j + n + step) % n;
    nextFace = byFace && sides && sides->size() == n ? (*sides)[step > 0 ? j : nx] : -1;
    V3 next = p2(L[nx].first - L[j].first, L[nx].second - L[j].second);
    if (norm(next) < 1e-12 || norm(h) < 0.5 || dot(unit(next), out) > -1e-3) return false;
    // (In the end-on frame of the sections: u into face A, v its normal; the cut's x runs the other way.)
    at = p2(-L[j].first, L[j].second), way = unit(p2(-next.x, next.y));
    for (size_t g = 0; g < n; g++) {
      j = (j + n + step) % n;
      V3 p = p2(L[j].first, L[j].second);
      if (norm(p) > within) break;
      if (dot(p, out) > tol) return false;
    }
    return true;
  };
  V3 d1, d2, e1, e2;
  int f1, f2;
  double l1 = run(1, d1, e1, f1);
  size_t j1 = jEnd;
  V3 h1 = lastHeading;
  double l2 = run(-1, d2, e2, f2);
  size_t j2 = jEnd;
  V3 h2 = lastHeading;
  // Face A leaves along -x.
  bool firstIsA = d1.x < d2.x;
  out.a = firstIsA ? l1 : l2, out.b = firstIsA ? l2 : l1;
  V3 endA = firstIsA ? e1 : e2, endB = firstIsA ? e2 : e1;
  V3 dirA = firstIsA ? d1 : d2, dirB = firstIsA ? d2 : d1;
  int faceA = firstIsA ? f1 : f2, faceB = firstIsA ? f2 : f1;
  if (radius > 0) {
    // As far as a rounding of the radius asked reaches: past its circle, and its tool's reach out past it.
    double within = std::max(out.a, out.b) + 4 * radius;
    V3 nB2 = unit(p2(dot(c.nb[i], X), dot(c.nb[i], Y)));
    out.aOpen = open(firstIsA ? 1 : -1, firstIsA ? j1 : j2, firstIsA ? h1 : h2, p2(0, 1), within, out.aNext, out.aWay, out.aNextFace);
    out.bOpen = open(firstIsA ? -1 : 1, firstIsA ? j2 : j1, firstIsA ? h2 : h1, nB2, within, out.bNext, out.bWay, out.bNextFace);
  }
  // The other crease's reach along the run (`face`), for a radius of 1: r·cot(half its angle) square to it, longer where
  // the run meets it aslant; where a face beside it is curved in its cut, where its circle (of the radius asked) touches.
  auto reach = [&](const Crease &o, V3 end, V3 dir, int face) {
    size_t j = 0;
    double best = INFINITY;
    for (size_t q = 0; q < o.pts.size(); q++)
      if (norm(o.pts[q] - end) < best) best = norm(o.pts[q] - end), j = q;
    double phi = o.angleAt(j) * pi / 180, half = (phi < pi ? phi : 2 * pi - phi) / 2;
    V3 along = o.pts[std::min(j + 1, o.pts.size() - 1)] - o.pts[j > 0 ? j - 1 : 0], world = X * dir.x + Y * dir.y;
    double sine = norm(along) > 0 ? norm(cross(unit(along), unit(world))) : 1, per = 1 / std::tan(half);
    if (radius > 0 && face >= 0 && (o.fa[j] == face || o.fb[j] == face)) {
      V3 U = o.ia[j], N = unit(o.na[j] - U * dot(o.na[j], U));
      V3 db = unit(p2(dot(o.ib[j], U), dot(o.ib[j], N))), nb = unit(p2(dot(o.nb[j], U), dot(o.nb[j], N)));
      Cut2 A = bendOf(s.faces[o.fa[j]].geom, o.pts[j], U, N, p2(1, 0), p2(0, 1), o.na[j]), B = bendOf(s.faces[o.fb[j]].geom, o.pts[j], U, N, db, nb, o.nb[j]);
      V3 C, TA, TB, nA, nB;
      bool convex;
      if ((A.circle || B.circle) && rollAt(A, B, radius, C, TA, TB, nA, nB, convex))
        per = (o.fa[j] == face ? alongCut(A, TA) : alongCut(B, TB)) / radius;
    }
    return per / std::max(sine, 0.2);
  };
  const Crease &me = self ? *self : c;
  // One meeting this one at a corner is beside it, not across the face: how those two share the face is the face's own
  // measure (faceRoom).
  auto besides = [&](const Crease &o) {
    if (me.closed || o.closed) return false;
    for (V3 p : {me.pts.front(), me.pts.back()})
      for (V3 q : {o.pts.front(), o.pts.back()})
        if (norm(p - q) < tol) return true;
    return false;
  };
  for (const auto &o : treated) {
    if (&o == &me || besides(o)) continue;
    if (distanceTo(o.pts, endA) < tol) out.aShared = true, out.aOther = reach(o, endA, dirA, faceA);
    if (distanceTo(o.pts, endB) < tol) out.bShared = true, out.bOther = reach(o, endB, dirB, faceB);
  }
  return out;
}

// How far one section reaches from the edge.
double spanOf2(const Section2 &x) {
  double s = 0;
  if (x.circle) s = std::max(s, norm(x.centre) + x.radius);
  for (const auto &r : x.runs) s = std::max(s, r.arc ? norm(r.centre) + r.radius : norm(r.p));
  return s;
}

// How far a run's tool reaches across it: the largest distance of its section's points from the edge.
double spanOf(const std::vector<Section2> &secs) {
  double s = 0;
  for (const auto &x : secs) s = std::max(s, spanOf2(x));
  return s;
}

// MARK: - rounded straight edges and their corners, as one mesh

// A straight edge between two flat faces, to be rounded: its ends, its faces (A, B) and their outward normals, and the corner
// (a ball) at each end when it has one.
struct Line {
  const Crease *c = nullptr;
  V3 E0, E1, T;
  int fa = -1, fb = -1;
  V3 na, nb;
  int arcs = 1;
  int ball[2] = {-1, -1};
};

// A corner where rounded straight edges meet on flat faces, rounded by a ball of the same radius: its faces and lines in turn
// round it (line i between faces i and i + 1).
struct Ball {
  V3 V, C;
  std::vector<int> faces, lines;
};

// A mesh put together piece by piece: points shared by position (pieces made apart meet point for point where they're made
// from the same numbers), each triangle's face, and each face's form — flat, round about an axis, a ball — for its normals
// and for the volume its chords miss.
struct Builder {
  struct Face {
    FaceGeom geom;
    int kind = 0;  // 0 flat, 1 round about an axis, 2 a ball, 3 rough
    V3 axisPoint, axis, centre;
    double miss = 0;  // the volume its chords miss, before its sign is known
  };
  std::vector<V3> pts;
  std::unordered_map<PKey, uint32_t, PKeyHash> index;
  std::vector<uint32_t> tri;
  std::vector<int> triFace;
  std::vector<Face> faces;

  uint32_t at(V3 p) {
    auto it = index.find(pkey(p));
    if (it != index.end()) return it->second;
    pts.push_back(p);
    return index[pkey(p)] = (uint32_t)pts.size() - 1;
  }
  int face(int kind, FaceGeom g = FaceGeom()) {
    Face f;
    f.kind = kind, f.geom = g;
    faces.push_back(f);
    return (int)faces.size() - 1;
  }
  void triangle(V3 a, V3 b, V3 c, int f) {
    uint32_t i = at(a), j = at(b), k = at(c);
    if (i == j || j == k || i == k) return;
    tri.insert(tri.end(), {i, j, k});
    triFace.push_back(f);
  }
  void quad(V3 a, V3 b, V3 c, V3 d, int f) { triangle(a, b, c, f), triangle(a, c, d, f); }

  // Every triangle turned the same way as its neighbours, each closed piece outward; then as the engine's mesh: each face its
  // own points with normals by its form, its exact form and the volume its chords miss.
  Solid solid() {
    size_t nt = triFace.size();
    std::unordered_map<uint64_t, std::vector<uint32_t>> sides;
    auto key = [](uint32_t a, uint32_t b) { return (uint64_t)std::min(a, b) << 32 | std::max(a, b); };
    for (size_t t = 0; t < nt; t++)
      for (int k = 0; k < 3; k++) sides[key(tri[3 * t + k], tri[3 * t + (k + 1) % 3])].push_back((uint32_t)t);
    std::vector<int> flip(nt, -1);
    auto runs = [&](size_t t, uint32_t a, uint32_t b) {
      for (int k = 0; k < 3; k++)
        if (tri[3 * t + k] == a && tri[3 * t + (k + 1) % 3] == b) return true;
      return false;
    };
    for (size_t seed = 0; seed < nt; seed++) {
      if (flip[seed] >= 0) continue;
      flip[seed] = 0;
      std::vector<size_t> todo{seed}, piece{seed};
      while (!todo.empty()) {
        size_t t = todo.back();
        todo.pop_back();
        for (int k = 0; k < 3; k++) {
          uint32_t a = tri[3 * t + k], b = tri[3 * t + (k + 1) % 3];
          bool forward = flip[t] == 0;  // t runs a → b when unflipped
          for (uint32_t o : sides[key(a, b)]) {
            if (o == t || flip[o] >= 0) continue;
            // A neighbour must run the side the other way.
            bool oRuns = runs(o, a, b);
            flip[o] = (oRuns == forward) ? 1 : 0;
            todo.push_back(o), piece.push_back(o);
          }
        }
      }
      double v = 0;
      for (size_t t : piece) {
        V3 a = pts[tri[3 * t]], b = pts[tri[3 * t + 1]], c = pts[tri[3 * t + 2]];
        double w = dot(a, cross(b, c));
        v += flip[t] ? -w : w;
      }
      if (v < 0)
        for (size_t t : piece) flip[t] ^= 1;
    }
    for (size_t t = 0; t < nt; t++)
      if (flip[t] == 1) std::swap(tri[3 * t + 1], tri[3 * t + 2]);
    Solid out;
    for (const auto &f : faces) {
      Solid::Face sf;
      sf.geom = f.geom;
      out.faces.push_back(sf);
    }
    // A flat face with no form given: the plane of its largest triangle (square where it's within rounding of square).
    std::vector<double> largest(faces.size(), 0);
    for (size_t t = 0; t < nt; t++) {
      Solid::Face &sf = out.faces[triFace[t]];
      if (faces[triFace[t]].kind != 0 || faces[triFace[t]].geom.flat) continue;
      V3 a = pts[tri[3 * t]], nn = cross(pts[tri[3 * t + 1]] - a, pts[tri[3 * t + 2]] - a);
      double area = norm(nn);
      if (area <= largest[triFace[t]]) continue;
      largest[triFace[t]] = area;
      nn = nn / area;
      for (int k = 0; k < 3; k++) {
        double &x = k == 0 ? nn.x : k == 1 ? nn.y : nn.z;
        if (std::fabs(x) < 1e-12) x = 0;
        else if (std::fabs(std::fabs(x) - 1) < 1e-12) x = x > 0 ? 1 : -1;
      }
      sf.geom.kind = FaceGeom::Flat, sf.geom.flat = true, sf.geom.pn = nn, sf.geom.pd = dot(nn, a);
    }
    std::vector<int> signOf(faces.size(), 0);
    for (size_t t = 0; t < nt; t++) {
      V3 a = pts[tri[3 * t]], b = pts[tri[3 * t + 1]], c = pts[tri[3 * t + 2]], nn = unit(cross(b - a, c - a));
      const Face &f = faces[triFace[t]];
      auto normalAt = [&](V3 p) {
        V3 n = nn;
        if (f.kind == 1) n = unit((p - f.axisPoint) - f.axis * dot(p - f.axisPoint, f.axis));
        if (f.kind == 2) n = unit(p - f.centre);
        if (f.kind == 1 || f.kind == 2) {
          if (dot(n, nn) < 0) n = -n;
          if (signOf[triFace[t]] == 0) {
            V3 out = f.kind == 1 ? (a - f.axisPoint) - f.axis * dot(a - f.axisPoint, f.axis) : a - f.centre;
            signOf[triFace[t]] = dot(nn, out) > 0 ? 1 : -1;
          }
        }
        return n;
      };
      uint32_t base = (uint32_t)out.p.size();
      out.vertex(a, normalAt(a)), out.vertex(b, normalAt(b)), out.vertex(c, normalAt(c));
      out.triangle(base, base + 1, base + 2, triFace[t]);
    }
    // A curved face facing away from its middle (a solid's) misses volume beyond its chords; facing towards it (a tool's), the
    // chords take in volume that isn't there.
    for (size_t f = 0; f < faces.size(); f++) out.faces[f].deficit = faces[f].miss * signOf[f];
    out.centroids();
    out.slivers();
    return out;
  }
};

// A line's loop across it at the axis point q (where its rounding's circle has its middle): the tangent points on faces A
// and B with the tool's sides going out from them (by m) to its outer corner, then the arc from B's tangent point back
// towards A's (its inner points).
std::vector<V3> loopAt(const Line &L, V3 q, double r, double m) {
  V3 TA = q + L.na * r, TB = q + L.nb * r;
  std::vector<V3> loop{TA, q + L.na * (r + m), q + (L.na + L.nb) * ((r + m) / (1 + dot(L.na, L.nb))), q + L.nb * (r + m), TB};
  double alpha = std::acos(std::clamp(dot(L.na, L.nb), -1.0, 1.0));
  V3 w = unit(L.na - L.nb * dot(L.na, L.nb));
  for (int j = 1; j < L.arcs; j++) {
    double a = alpha * j / L.arcs;
    loop.push_back(q + (L.nb * std::cos(a) + w * std::sin(a)) * r);
  }
  return loop;
}

// The axis point of a line's rounding across it at distance s along it from its first end.
V3 axisAt(const Line &L, double s, double r) { return L.E0 + L.T * s - (L.na + L.nb) * (r / (1 + dot(L.na, L.nb))); }

V3 slerp(V3 a, V3 b, double t) {
  double th = std::acos(std::clamp(dot(a, b), -1.0, 1.0));
  if (th < 1e-9) return unit(a * (1 - t) + b * t);
  return unit(a * (std::sin((1 - t) * th) / std::sin(th)) + b * (std::sin(t * th) / std::sin(th)));
}

// The ball's patch at a corner: its outline (the lines' arcs one after another) filled in rings towards its middle.
void ballPatch(Builder &B, const Ball &ball, const std::vector<V3> &outline, double r, double d) {
  FaceGeom g;
  g.kind = FaceGeom::Turned, g.elem = Elem::arcOf(0, 0, r, -pi / 2, pi / 2), g.place = Affine::translation(ball.C);
  int f = B.face(2, g);
  B.faces[f].centre = ball.C;
  V3 mid = unit(ball.V - ball.C);
  double widest = 0;
  for (V3 p : outline) widest = std::max(widest, std::acos(std::clamp(dot(unit(p - ball.C), mid), -1.0, 1.0)));
  int rings = std::max(1, (int)std::ceil(widest / (2 * std::acos(std::clamp(1 - d / r, -1.0, 1.0))) - 1e-9));
  rings = std::min(rings, 32);
  size_t n = outline.size();
  std::vector<std::vector<V3>> ring(rings + 1);
  for (int l = 1; l < rings; l++)
    for (V3 p : outline) ring[l].push_back(ball.C + slerp(mid, unit(p - ball.C), (double)l / rings) * r);
  ring[rings] = outline;
  V3 top = ball.C + mid * r;
  double miss = 0;
  auto add = [&](V3 a, V3 b, V3 c) {
    B.triangle(a, b, c, f);
    // The ball's sector over this triangle less the triangle's pyramid from the middle.
    V3 x = a - ball.C, y = b - ball.C, z = c - ball.C;
    double det = dot(x, cross(y, z)), den = r * r * r + dot(x, y) * r + dot(y, z) * r + dot(z, x) * r;
    double omega = 2 * std::atan2(std::fabs(det), den);
    miss += r * r * r * std::fabs(omega) / 3 - std::fabs(det) / 6;
  };
  for (size_t j = 0; j < n; j++) {
    size_t k = (j + 1) % n;
    add(top, ring[1][j], ring[1][k]);
    for (int l = 1; l < rings; l++) {
      add(ring[l][j], ring[l + 1][j], ring[l + 1][k]);
      add(ring[l][j], ring[l + 1][k], ring[l][k]);
    }
  }
  B.faces[f].miss = miss;
}

// Rounding the lines: a tool for each group of lines joined at balls, one mesh each — each line's tool between its end
// loops (at a ball, the loop round the ball's middle; at a free end, run on past the end, or a hair short of it), each
// ball's patch, and over each corner a lid reaching out past the faces.
std::vector<Solid> lineTools(const Solid &s, const std::vector<Line> &lines, const std::vector<Ball> &balls, double r, double d, double tol) {
  // Groups: lines joined through balls.
  std::vector<int> group(lines.size(), -1);
  int groups = 0;
  for (size_t i = 0; i < lines.size(); i++) {
    if (group[i] >= 0) continue;
    std::vector<int> todo{(int)i};
    group[i] = groups;
    while (!todo.empty()) {
      int l = todo.back();
      todo.pop_back();
      for (int e : lines[l].ball)
        if (e >= 0)
          for (int o : balls[e].lines)
            if (group[o] < 0) group[o] = groups, todo.push_back(o);
    }
    groups++;
  }
  std::vector<Solid> out;
  for (int gi = 0; gi < groups; gi++) {
    Builder B;
    double m = reachOut(r, gi);
    std::vector<int> mine;
    for (size_t i = 0; i < lines.size(); i++)
      if (group[i] == gi) mine.push_back((int)i);
    for (int li : mine) {
      const Line &L = lines[li];
      double len = norm(L.E1 - L.E0);
      std::vector<V3> ends[2];
      for (int e = 0; e < 2; e++) {
        if (L.ball[e] >= 0) {
          ends[e] = loopAt(L, balls[L.ball[e]].C, r, m);
          continue;
        }
        // A free end: on past it onto a face there (outside the solid), or stopping a hair short.
        V3 V = e == 0 ? L.E0 : L.E1, out = e == 0 ? -L.T : L.T;
        std::vector<int> others;
        for (int f : facesAt(s, V))
          if (f != L.fa && f != L.fb) others.push_back(f);
        bool onto = others.size() == 1 && s.faces[others[0]].geom.flat && dot(s.faces[others[0]].geom.pn, out) > 0.05;
        double span = r + m + norm(L.na + L.nb) * (r + m), reach = onto ? (span + 0.45 * m) / dot(s.faces[others[0]].geom.pn, out) * 1.2 + span : -hair(r);
        ends[e] = loopAt(L, axisAt(L, e == 0 ? -reach : len + reach, r), r, m);
      }
      size_t k = ends[0].size();
      double between = dot(ends[1][0] - ends[0][0], L.T);
      if (!(between > tol)) return {};
      int wallA = B.face(0), outA = B.face(0), outB = B.face(0), wallB = B.face(0);
      FaceGeom cyl;
      {
        // The rounding's cylinder: a turned face round the line's axis.
        V3 q = axisAt(L, 0, r), x = unit(L.na - L.T * dot(L.na, L.T)), y = cross(L.T, x);
        Affine f;
        f.m[0] = x.x, f.m[1] = y.x, f.m[2] = L.T.x, f.m[3] = q.x;
        f.m[4] = x.y, f.m[5] = y.y, f.m[6] = L.T.y, f.m[7] = q.y;
        f.m[8] = x.z, f.m[9] = y.z, f.m[10] = L.T.z, f.m[11] = q.z;
        cyl.kind = FaceGeom::Turned, cyl.elem = Elem::line(r, -1e6, r, 1e6), cyl.place = f;
      }
      int arc = B.face(1, cyl);
      B.faces[arc].axisPoint = axisAt(L, 0, r), B.faces[arc].axis = L.T;
      int faceOf[4] = {wallA, outA, outB, wallB};
      for (size_t j = 0; j < k; j++) {
        size_t n = (j + 1) % k;
        int f = j < 4 ? faceOf[j] : arc;
        B.quad(ends[0][j], ends[0][n], ends[1][n], ends[1][j], f);
      }
      double a = std::acos(std::clamp(dot(L.na, L.nb), -1.0, 1.0)) / L.arcs;
      B.faces[arc].miss += L.arcs * r * r / 2 * (a - std::sin(a)) * between;
      for (int e = 0; e < 2; e++) {
        if (L.ball[e] >= 0) continue;
        // A free end's cap: a fan from its outer corner.
        int cap = B.face(0);
        for (size_t j = 0; j < k; j++) {
          size_t n = (j + 1) % k;
          if (j == 2 || n == 2) continue;
          B.triangle(ends[e][2], ends[e][j], ends[e][n], cap);
        }
      }
    }
    for (size_t bi = 0; bi < balls.size(); bi++) {
      const Ball &ball = balls[bi];
      if (group[ball.lines[0]] != gi) continue;
      // The patch's outline: each line's arc in turn, from face i's tangent point to face i + 1's.
      std::vector<V3> outline, lid;
      for (size_t i = 0; i < ball.lines.size(); i++) {
        const Line &L = lines[ball.lines[i]];
        std::vector<V3> loop = loopAt(L, ball.C, r, m);
        // loop: TA, PA, O, PB, TB, arc from TB towards TA.
        bool forward = L.fb == ball.faces[i];  // B first: runs from face i to face i + 1 as the arc does
        std::vector<V3> arcPts{loop[4]};
        for (size_t j = 5; j < loop.size(); j++) arcPts.push_back(loop[j]);
        arcPts.push_back(loop[0]);
        if (!forward) std::reverse(arcPts.begin(), arcPts.end());
        for (size_t j = 0; j + 1 < arcPts.size(); j++) outline.push_back(arcPts[j]);
        // The lid: face i's point out, the line's outer corner.
        lid.push_back(forward ? loop[3] : loop[1]);
        lid.push_back(loop[2]);
      }
      ballPatch(B, ball, outline, r, d);
      int top = B.face(3);
      V3 far = ball.V + unit(ball.V - ball.C) * (3 * (r + m));
      for (size_t j = 0; j < lid.size(); j++) B.triangle(far, lid[j], lid[(j + 1) % lid.size()], top);
    }
    out.push_back(B.solid());
  }
  return out;
}

// A section swept straight along T through stations (each its frame's origin, in order): a side per straight run, a round
// face per arc (its form, and the volume its chords miss), the two ends flat.
Solid prismTool(const Section2 &sec, const std::vector<V3> &at, V3 U, V3 N, V3 T, double d) {
  std::vector<int> chords = chordsOf(sec, d), runOf;
  std::vector<V3> outline = pointsOf(sec, chords, &runOf);
  size_t k = outline.size(), n = at.size();
  Builder B;
  std::vector<std::vector<V3>> ring(n);
  for (size_t i = 0; i < n; i++)
    for (V3 q : outline) ring[i].push_back(at3(at[i], U, N, q));
  double length = dot(at.back() - at.front(), T);
  // Each run's face.
  size_t runs = sec.circle ? 1 : sec.runs.size();
  std::vector<int> faceOf(runs);
  for (size_t i = 0; i < runs; i++) {
    bool arc = sec.circle || sec.runs[i].arc;
    if (!arc) {
      faceOf[i] = B.face(0);
      continue;
    }
    V3 c = sec.circle ? sec.centre : sec.runs[i].centre;
    double r = sec.circle ? sec.radius : sec.runs[i].radius, turn = 2 * pi;
    if (!sec.circle) {
      double a0;
      arcTurn(sec.runs[i], sec.runs[(i + 1) % runs].p, a0, turn);
    }
    V3 c3 = at3(at[0], U, N, c), x = U, y = cross(T, x);
    FaceGeom cyl;
    Affine f;
    f.m[0] = x.x, f.m[1] = y.x, f.m[2] = T.x, f.m[3] = c3.x;
    f.m[4] = x.y, f.m[5] = y.y, f.m[6] = T.y, f.m[7] = c3.y;
    f.m[8] = x.z, f.m[9] = y.z, f.m[10] = T.z, f.m[11] = c3.z;
    cyl.kind = FaceGeom::Turned, cyl.elem = Elem::line(r, -1e6, r, 1e6), cyl.place = f;
    faceOf[i] = B.face(1, cyl);
    B.faces[faceOf[i]].axisPoint = c3, B.faces[faceOf[i]].axis = T;
    double a = std::fabs(turn) / chords[i];
    B.faces[faceOf[i]].miss = chords[i] * r * r / 2 * (a - std::sin(a)) * length;
  }
  for (size_t i = 0; i + 1 < n; i++)
    for (size_t j = 0; j < k; j++) {
      size_t m = (j + 1) % k;
      B.quad(ring[i][j], ring[i][m], ring[i + 1][m], ring[i + 1][j], faceOf[runOf[j]]);
    }
  std::vector<int> cap = capOf(outline);
  if (cap.empty()) return Solid();
  for (size_t e : {(size_t)0, n - 1}) {
    int f = B.face(0);
    for (size_t q = 0; q + 2 < cap.size(); q += 3) B.triangle(ring[e][cap[q]], ring[e][cap[q + 1]], ring[e][cap[q + 2]], f);
  }
  return B.solid();
}

// A section carried round an axis through frames (E, U, N at each step, each in a plane through the axis, in order): a
// face per run (none along the axis itself), the two ends flat; the volume each face's chords miss by Pappus, its run's
// moment about the axis turned exactly less as its chords are.
Solid turnTool(const Section2 &sec, const std::vector<V3> &E, const std::vector<V3> &U, const std::vector<V3> &N, V3 centre, V3 axis, double d) {
  std::vector<int> chords = chordsOf(sec, d), runOf;
  std::vector<V3> q = pointsOf(sec, chords, &runOf);
  size_t k = q.size(), n = E.size(), runs = sec.runs.size();
  if (k < 3 || n < 2 || sec.circle) return Solid();
  // In the half-plane through the first step: (distance from the axis, height along it).
  V3 radial0 = unit((E[0] - centre) - axis * dot(E[0] - centre, axis));
  auto rz = [&](V3 p2d) {
    V3 w = at3(E[0], U[0], N[0], p2d) - centre;
    return p2(dot(w, radial0), dot(w, axis));
  };
  // Points on the axis (a run's ends there, or within rounding of it) are one point at every step.
  std::vector<char> onAxis(k, 0);
  double reach = norm(E[0] - centre);
  for (size_t j = 0; j < k; j++) {
    if (sec.runs[runOf[j]].axis) onAxis[j] = onAxis[(j + 1) % k] = 1;
    if (std::fabs(rz(q[j]).x) <= 1e-9 * (1 + reach)) onAxis[j] = 1;
  }
  auto point = [&](size_t i, size_t j) {
    if (onAxis[j]) return centre + axis * dot(at3(E[0], U[0], N[0], q[j]) - centre, axis);
    return at3(E[i], U[i], N[i], q[j]);
  };
  double sweep = 0, sines = 0;
  for (size_t i = 0; i + 1 < n; i++) {
    V3 a = unit((E[i] - centre) - axis * dot(E[i] - centre, axis)), b = unit((E[i + 1] - centre) - axis * dot(E[i + 1] - centre, axis));
    double step = std::atan2(norm(cross(a, b)), dot(a, b));
    sweep += step, sines += std::sin(step);
  }
  Solid out;
  std::vector<int> faceOf(runs, -1);
  std::vector<double> exact(runs, 0), meshed(runs, 0);
  Affine place;
  {
    V3 x = radial0, y = cross(axis, x);
    place.m[0] = x.x, place.m[1] = y.x, place.m[2] = axis.x, place.m[3] = centre.x;
    place.m[4] = x.y, place.m[5] = y.y, place.m[6] = axis.y, place.m[7] = centre.y;
    place.m[8] = x.z, place.m[9] = y.z, place.m[10] = axis.z, place.m[11] = centre.z;
  }
  double total = 0;
  for (size_t r = 0; r < runs; r++) {
    const auto &run = sec.runs[r];
    V3 a = rz(run.p), b = rz(sec.runs[(r + 1) % runs].p);
    Elem e = run.arc ? arcThrough(rz(run.centre), run.radius, a, b, rz(run.mid)) : Elem::line(a.x, a.y, b.x, b.y);
    exact[r] = profileMoment(e), total += exact[r];
    if (run.axis) continue;
    faceOf[r] = (int)out.faces.size();
    Solid::Face f;
    f.geom.kind = FaceGeom::Turned, f.geom.elem = e, f.geom.place = place;
    out.faces.push_back(f);
  }
  for (size_t j = 0; j < k; j++) {
    V3 a = rz(q[j]), b = rz(q[(j + 1) % k]);
    meshed[runOf[j]] += profileMoment(Elem::line(a.x, a.y, b.x, b.y));
  }
  for (size_t r = 0; r < runs; r++)
    if (faceOf[r] >= 0) out.faces[faceOf[r]].deficit = (total < 0 ? -1 : 1) * (sweep * exact[r] - sines * meshed[r]);
  // Each point's outward normal in the section, on the side of each run it ends.
  auto normal2 = [&](size_t j, V3 at2) {
    const auto &run = sec.runs[runOf[j]];
    V3 dir = q[(j + 1) % k] - q[j], out2 = unit(p2(dir.y, -dir.x));
    if (!run.arc) return out2;
    V3 rad = unit(at2 - run.centre);
    return dot(rad, out2) < 0 ? -rad : rad;
  };
  auto tri = [&](V3 a, V3 b, V3 c, V3 na, V3 nb, V3 nc, int f) {
    if (a == b || b == c || a == c) return;
    uint32_t base = (uint32_t)out.p.size();
    out.vertex(a, na), out.vertex(b, nb), out.vertex(c, nc);
    out.triangle(base, base + 1, base + 2, f);
  };
  for (size_t i = 0; i + 1 < n; i++)
    for (size_t j = 0; j < k; j++) {
      int f = faceOf[runOf[j]];
      if (f < 0) continue;
      size_t m = (j + 1) % k;
      V3 nj = normal2(j, q[j]), nm = normal2(j, q[m]);
      V3 a = point(i, j), b = point(i, m), c = point(i + 1, m), dd = point(i + 1, j);
      V3 na = U[i] * nj.x + N[i] * nj.y, nb = U[i] * nm.x + N[i] * nm.y, nc = U[i + 1] * nm.x + N[i + 1] * nm.y, nd = U[i + 1] * nj.x + N[i + 1] * nj.y;
      tri(a, b, c, na, nb, nc, f);
      tri(a, c, dd, na, nc, nd, f);
    }
  std::vector<int> cap = capOf(q);
  if (cap.empty()) return Solid();
  for (size_t e : {(size_t)0, n - 1}) {
    int f = (int)out.faces.size();
    out.faces.push_back(Solid::Face());
    for (size_t t = 0; t + 2 < cap.size(); t += 3) {
      V3 a = point(e, cap[t]), b = point(e, cap[t + 1]), c = point(e, cap[t + 2]);
      if (e == 0) std::swap(b, c);
      V3 nn = unit(cross(b - a, c - a));
      tri(a, b, c, nn, nn, nn, f);
    }
  }
  if (out.meshVolume() < 0)
    for (size_t t = 0; t < out.tri.size(); t += 3) std::swap(out.tri[t + 1], out.tri[t + 2]);
  for (size_t t = 0; t < out.triFace.size(); t++) {
    Solid::Face &f = out.faces[out.triFace[t]];
    if (f.geom.kind == FaceGeom::Turned || f.geom.flat) continue;
    V3 a = out.p[out.tri[3 * t]], nn = unit(cross(out.p[out.tri[3 * t + 1]] - a, out.p[out.tri[3 * t + 2]] - a));
    f.geom.kind = FaceGeom::Flat, f.geom.flat = true, f.geom.pn = nn, f.geom.pd = dot(nn, a);
  }
  out.centroids();
  out.slivers();
  return out;
}

// A disc as two half-circle runs (its points where pointsOf puts a disc's: an odd part of a step round).
Section2 asRuns(const Section2 &s, double d) {
  if (!s.circle) return s;
  int k = chordsOf(s, d)[0];
  double a0 = 2 * pi * 0.381966 / k;
  Section2 out;
  out.fill = s.fill;
  for (int h = 0; h < 2; h++) {
    Section2::Run r;
    r.p = s.centre + p2(std::cos(a0 + pi * h), std::sin(a0 + pi * h)) * s.radius;
    r.arc = true, r.centre = s.centre, r.radius = s.radius;
    r.mid = s.centre + p2(std::cos(a0 + pi * h + pi / 2), std::sin(a0 + pi * h + pi / 2)) * s.radius;
    out.runs.push_back(r);
  }
  return out;
}

// A section kept where dot(p, g) >= h (cut along that line, the cut a straight run marked as lying along the axis);
// `cutAway` false when nothing lies beyond it (the section is returned as it was).
Section2 clipped(const Section2 &s, V3 g, double h, bool &cutAway) {
  cutAway = false;
  auto side = [&](V3 p) { return dot(p, g) - h; };
  std::vector<int> chords = chordsOf(s, 1e-4 * (1 + spanOf2(s)));
  for (V3 p : pointsOf(s, chords))
    if (side(p) < -1e-12 * (1 + std::fabs(h))) cutAway = true;
  if (!cutAway) return s;
  struct Piece {
    Section2::Run run;
    V3 end;
  };
  std::vector<Piece> kept;
  size_t n = s.runs.size();
  for (size_t i = 0; i < n; i++) {
    const auto &r = s.runs[i];
    V3 p = r.p, q = s.runs[(i + 1) % n].p;
    if (!r.arc) {
      double sp = side(p), sq = side(q);
      if (sp >= 0 && sq >= 0) {
        kept.push_back({r, q});
      } else if (sp >= 0 || sq >= 0) {
        V3 X = p + (q - p) * (sp / (sp - sq));
        Section2::Run part = r;
        if (sp >= 0) kept.push_back({part, X});
        else part.p = X, kept.push_back({part, q});
      }
      continue;
    }
    double a0, turn;
    arcTurn(r, q, a0, turn);
    double A = side(r.centre), gl = norm(g), gamma = std::atan2(g.y, g.x);
    std::vector<double> at{0, 1};
    if (std::fabs(A) < r.radius * gl) {
      double off = std::acos(-A / (r.radius * gl));
      for (double a : {gamma + off, gamma - off}) {
        double dlt = std::remainder(a - a0, 2 * pi);
        if (turn > 0 && dlt < 0) dlt += 2 * pi;
        if (turn < 0 && dlt > 0) dlt -= 2 * pi;
        double f = dlt / turn;
        if (f > 1e-12 && f < 1 - 1e-12) at.push_back(f);
      }
    }
    std::sort(at.begin(), at.end());
    auto pointAt = [&](double f) { return f <= 0 ? p : f >= 1 ? q : r.centre + p2(std::cos(a0 + turn * f), std::sin(a0 + turn * f)) * r.radius; };
    for (size_t k = 0; k + 1 < at.size(); k++) {
      double mid = (at[k] + at[k + 1]) / 2;
      if (side(pointAt(mid)) < 0) continue;
      Section2::Run part = r;
      part.p = pointAt(at[k]), part.mid = pointAt(mid);
      kept.push_back({part, pointAt(at[k + 1])});
    }
  }
  Section2 out;
  out.fill = s.fill;
  for (size_t k = 0; k < kept.size(); k++) {
    out.runs.push_back(kept[k].run);
    V3 next = kept[(k + 1) % kept.size()].run.p;
    if (norm(next - kept[k].end) > 1e-12 * (1 + std::fabs(h))) {
      Section2::Run along;
      along.p = kept[k].end, along.axis = true;
      out.runs.push_back(along);
    }
  }
  return out;
}

// The whole solid rounded at once (every edge a line, every corner a ball): its faces shrunk to the tangent points, each
// line's cylinder between its balls, each ball's patch — no tool needed.
bool roundedWhole(const Solid &s, const std::vector<Line> &lines, const std::vector<Ball> &balls, double r, double d, Solid &out) {
  Builder B;
  // Each face: the tangent points of the balls on it, in turn round its normal.
  for (size_t f = 0; f < s.faces.size(); f++) {
    std::vector<V3> pts;
    for (const auto &ball : balls)
      for (int g : ball.faces)
        if (g == (int)f) pts.push_back(ball.C + s.faces[f].geom.pn * r);
    if (pts.size() < 3) return false;
    V3 mid;
    for (V3 p : pts) mid += p;
    mid = mid / (double)pts.size();
    V3 n = s.faces[f].geom.pn, e1 = unit(pts[0] - mid), e2 = cross(n, e1);
    std::sort(pts.begin(), pts.end(), [&](V3 a, V3 b) { return std::atan2(dot(a - mid, e2), dot(a - mid, e1)) < std::atan2(dot(b - mid, e2), dot(b - mid, e1)); });
    int face = B.face(0, s.faces[f].geom);
    for (size_t j = 1; j + 1 < pts.size(); j++) B.triangle(pts[0], pts[j], pts[j + 1], face);
  }
  for (const auto &L : lines) {
    if (L.ball[0] < 0 || L.ball[1] < 0) return false;
    std::vector<V3> ends[2];
    for (int e = 0; e < 2; e++) {
      std::vector<V3> loop = loopAt(L, balls[L.ball[e]].C, r, 0);
      ends[e] = {loop[4]};
      for (size_t j = 5; j < loop.size(); j++) ends[e].push_back(loop[j]);
      ends[e].push_back(loop[0]);
    }
    double between = dot(ends[1][0] - ends[0][0], L.T);
    if (!(between > 0)) return false;
    FaceGeom cyl;
    V3 q = axisAt(L, 0, r), x = unit(L.na - L.T * dot(L.na, L.T)), y = cross(L.T, x);
    Affine f;
    f.m[0] = x.x, f.m[1] = y.x, f.m[2] = L.T.x, f.m[3] = q.x;
    f.m[4] = x.y, f.m[5] = y.y, f.m[6] = L.T.y, f.m[7] = q.y;
    f.m[8] = x.z, f.m[9] = y.z, f.m[10] = L.T.z, f.m[11] = q.z;
    cyl.kind = FaceGeom::Turned, cyl.elem = Elem::line(r, -1e6, r, 1e6), cyl.place = f;
    int arc = B.face(1, cyl);
    B.faces[arc].axisPoint = q, B.faces[arc].axis = L.T;
    for (size_t j = 0; j + 1 < ends[0].size(); j++) B.quad(ends[0][j], ends[0][j + 1], ends[1][j + 1], ends[1][j], arc);
    double a = std::acos(std::clamp(dot(L.na, L.nb), -1.0, 1.0)) / L.arcs;
    B.faces[arc].miss = L.arcs * r * r / 2 * (a - std::sin(a)) * between;
  }
  for (const auto &ball : balls) {
    std::vector<V3> outline;
    for (size_t i = 0; i < ball.lines.size(); i++) {
      const Line &L = lines[ball.lines[i]];
      std::vector<V3> loop = loopAt(L, ball.C, r, 0);
      bool forward = L.fb == ball.faces[i];
      std::vector<V3> arcPts{loop[4]};
      for (size_t j = 5; j < loop.size(); j++) arcPts.push_back(loop[j]);
      arcPts.push_back(loop[0]);
      if (!forward) std::reverse(arcPts.begin(), arcPts.end());
      for (size_t j = 0; j + 1 < arcPts.size(); j++) outline.push_back(arcPts[j]);
    }
    ballPatch(B, ball, outline, r, d);
  }
  out = B.solid();
  return out.meshVolume() > 0;
}

// What's left of tools' faces meant to lie outside what they cut (`aux`): thin bits (no wider than a few chord errors `d`,
// where meshes meet near tangent) taken into the face beside each they share most of their outline with.
// Faces no wider than `width` (twice their area over their outline's length) taken into the face beside them they share
// the most outline with: tools' helper faces (`auxOnly`, into faces that aren't), or any (a hair's remnant of a face where
// two tools meet). Whether any was.
bool fold(Solid &r, double width, bool auxOnly) {
  bool any = !auxOnly;
  for (const auto &f : r.faces) any = any || f.aux;
  if (!any) return false;
  bool folded = false;
  std::unordered_map<PKey, uint32_t, PKeyHash> id;
  std::vector<uint32_t> at(r.p.size());
  for (size_t i = 0; i < r.p.size(); i++) at[i] = id.emplace(pkey(r.p[i]), (uint32_t)id.size()).first->second;
  size_t nt = r.triFace.size();
  std::unordered_map<uint64_t, std::vector<uint32_t>> sides;
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) {
      uint32_t a = at[r.tri[3 * t + k]], b = at[r.tri[3 * t + (k + 1) % 3]];
      sides[(uint64_t)std::min(a, b) << 32 | std::max(a, b)].push_back((uint32_t)t);
    }
  for (int round = 0; round < 8; round++) {
    std::vector<std::vector<uint32_t>> of(r.faces.size());
    for (size_t t = 0; t < nt; t++) of[r.triFace[t]].push_back((uint32_t)t);
    // Each such face's sides shared with other faces, by length; and how wide it is (twice its area over its outline's
    // length).
    std::map<int, std::map<int, double>> beside;
    std::map<int, double> outline;
    for (const auto &[key, ts] : sides) {
      if (ts.size() != 2) continue;
      int fa = (int)r.triFace[ts[0]], fb = (int)r.triFace[ts[1]];
      if (fa == fb) continue;
      V3 p = r.p[r.tri[3 * ts[0]]];
      for (int k = 0; k < 3; k++) {
        uint32_t a = at[r.tri[3 * ts[0] + k]], b = at[r.tri[3 * ts[0] + (k + 1) % 3]];
        if (((uint64_t)std::min(a, b) << 32 | std::max(a, b)) == key) p = r.p[r.tri[3 * ts[0] + k]] - r.p[r.tri[3 * ts[0] + (k + 1) % 3]];
      }
      double len = norm(p);
      outline[fa] += len, outline[fb] += len;
      if (!auxOnly || (r.faces[fa].aux && !r.faces[fb].aux)) beside[fa][fb] += len;
      if (!auxOnly || (r.faces[fb].aux && !r.faces[fa].aux)) beside[fb][fa] += len;
    }
    bool changed = false;
    for (const auto &[f, near] : beside) {
      if (of[f].empty()) continue;
      double area = 0;
      for (uint32_t t : of[f]) {
        V3 a = r.p[r.tri[3 * t]], b = r.p[r.tri[3 * t + 1]], c = r.p[r.tri[3 * t + 2]];
        area += norm(cross(b - a, c - a)) / 2;
      }
      if (!(outline[f] > 0) || 2 * area / outline[f] > width) continue;
      int into = -1;
      for (const auto &[g, len] : near)
        if (!of[g].empty() && (into < 0 || len > near.at(into))) into = g;
      if (into < 0) continue;
      for (uint32_t t : of[f]) r.triFace[t] = (uint32_t)into;
      of[into].insert(of[into].end(), of[f].begin(), of[f].end()), of[f].clear();
      r.faces[into].deficit += r.faces[f].deficit, r.faces[f].deficit = 0;
      changed = folded = true;
    }
    if (!changed) break;
  }
  return folded;
}

void foldAux(Solid &r, double d) { fold(r, 4 * d, true); }

// Of the pieces a treatment that only takes away left, the `keep` largest kept: any more are islands its tools left
// between them (taking away can't part a piece from the shape), each no more than `share` of the whole. Whether any were.
bool dropIslands(Solid &r, int keep, double share) {
  std::unordered_map<PKey, uint32_t, PKeyHash> id;
  std::vector<uint32_t> at(r.p.size());
  for (size_t i = 0; i < r.p.size(); i++) at[i] = id.emplace(pkey(r.p[i]), (uint32_t)id.size()).first->second;
  size_t nt = r.triFace.size();
  std::vector<uint32_t> parent(nt);
  std::iota(parent.begin(), parent.end(), 0);
  auto find = [&](uint32_t x) {
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
  };
  std::unordered_map<uint64_t, uint32_t> side;
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) {
      uint32_t a = at[r.tri[3 * t + k]], b = at[r.tri[3 * t + (k + 1) % 3]];
      auto [it, fresh] = side.emplace((uint64_t)std::min(a, b) << 32 | std::max(a, b), (uint32_t)t);
      if (!fresh) parent[find((uint32_t)t)] = find(it->second);
    }
  std::unordered_map<uint32_t, double> vol;
  double whole = 0;
  for (size_t t = 0; t < nt; t++) {
    double v = dot(r.p[r.tri[3 * t]], cross(r.p[r.tri[3 * t + 1]], r.p[r.tri[3 * t + 2]])) / 6;
    vol[find((uint32_t)t)] += v, whole += v;
  }
  std::vector<std::pair<double, uint32_t>> bySize;
  for (auto [root, v] : vol)
    if (v > 0) bySize.push_back({v, root});
  if ((int)bySize.size() <= keep) return false;
  std::sort(bySize.begin(), bySize.end(), std::greater<>());
  std::set<uint32_t> gone;
  for (size_t k = keep; k < bySize.size(); k++) {
    if (bySize[k].first > share * whole) return false;
    gone.insert(bySize[k].second);
  }
  Solid out;
  out.faces = r.faces;
  for (auto &f : out.faces) f.deficit = 0;
  bool gaps = !r.gap.empty();
  for (size_t t = 0; t < nt; t++) {
    if (gone.count(find((uint32_t)t))) continue;
    uint32_t base = (uint32_t)out.p.size();
    for (int k = 0; k < 3; k++) out.vertex(r.p[r.tri[3 * t + k]], r.n[r.tri[3 * t + k]]);
    out.triangle(base, base + 1, base + 2, (int)r.triFace[t]);
    if (gaps) out.gap.insert(out.gap.end(), r.gap.begin() + 6 * t, r.gap.begin() + 6 * t + 6);
  }
  if (gaps)
    for (size_t t = 0; t < out.triFace.size(); t++) out.faces[out.triFace[t]].deficit += out.sliver(t);
  r = std::move(out);
  return true;
}

// How far a flat face's sides can move in before the face is gone: each side moves in by k times its own setback
// (`setback` per mesh edge, 0 for an edge left as it is), and the largest k that leaves some of the face. Moved in, a side
// shortens by how its neighbours move (its ends slide along it); one shortened to nothing drops out and its neighbours
// meet; the face is gone when fewer than three sides are left, or two facing each other meet. (Each outer outline alone;
// how far a hole's sides are from the outline is the cut across each crease's to tell.)
double faceRoom(const Solid &s, int f, const std::vector<double> &setback) {
  V3 n = s.faces[f].geom.pn, e1 = unit(std::fabs(n.x) < 0.9 ? cross(n, V3{1, 0, 0}) : cross(n, V3{0, 1, 0})), e2 = cross(n, e1);
  // The face's triangles' sides, each the way it runs round its triangle (the face to its left).
  std::set<std::pair<PKey, PKey>> sides;
  for (size_t t = 0; t < s.triFace.size(); t++) {
    if ((int)s.triFace[t] != f) continue;
    for (int k = 0; k < 3; k++) sides.insert({pkey(s.p[s.tri[3 * t + k]]), pkey(s.p[s.tri[3 * t + (k + 1) % 3]])});
  }
  struct Side {
    V3 a, b;  // in place; in a loop, in the face's plane (x, y)
    double t;
  };
  auto flat2 = [&](V3 q) { return p2(dot(q, e1), dot(q, e2)); };
  std::vector<Side> segs;
  for (size_t e = 0; e < s.edges.size(); e++) {
    const auto &edge = s.edges[e];
    if ((edge.f0 != f && edge.f1 != f) || edge.pts.size() < 2) continue;
    std::vector<V3> pts = edge.pts;
    if (!sides.count({pkey(pts[0]), pkey(pts[1])})) std::reverse(pts.begin(), pts.end());
    for (size_t i = 0; i + 1 < pts.size(); i++) segs.push_back({pts[i], pts[i + 1], setback[e]});
  }
  std::map<PKey, size_t> from;
  for (size_t i = 0; i < segs.size(); i++) from[pkey(segs[i].a)] = i;
  std::vector<char> seen(segs.size(), 0);
  double room = INFINITY;
  for (size_t first = 0; first < segs.size(); first++) {
    if (seen[first]) continue;
    // A loop, sides running straight on (and moving in alike) taken as one.
    std::vector<Side> loop;
    for (size_t i = first; !seen[i];) {
      seen[i] = 1;
      Side sd{flat2(segs[i].a), flat2(segs[i].b), segs[i].t};
      if (!loop.empty() && loop.back().t == sd.t && std::fabs(cross2(unit(loop.back().b - loop.back().a), unit(sd.b - sd.a))) < 1e-9 &&
          dot(loop.back().b - loop.back().a, sd.b - sd.a) > 0)
        loop.back().b = sd.b;
      else
        loop.push_back(sd);
      auto it = from.find(pkey(segs[i].b));
      if (it == from.end()) break;
      i = it->second;
    }
    // The loop may have started part way along a side: its two parts, last and first, one side.
    while (loop.size() > 3 && loop.back().t == loop.front().t &&
           std::fabs(cross2(unit(loop.back().b - loop.back().a), unit(loop.front().b - loop.front().a))) < 1e-9 &&
           dot(loop.back().b - loop.back().a, loop.front().b - loop.front().a) > 0) {
      loop.front().a = loop.back().a;
      loop.pop_back();
    }
    std::vector<V3> outline;
    for (auto &sd : loop) outline.push_back(sd.a);
    if (loop.size() < 3 || area2(outline) <= 0) continue;
    // Each side as a line: a point, its way along, and inward (to its left); moved in by k·t at time k.
    struct L {
      V3 a, d, in;
      double t;
    };
    std::vector<L> lines;
    for (auto &sd : loop) {
      V3 d = unit(sd.b - sd.a);
      lines.push_back({sd.a, d, p2(-d.y, d.x), sd.t});
    }
    bool moves = false;
    for (auto &l : lines) moves = moves || l.t > 0;
    if (!moves) continue;
    // Where lines i and j meet at time k (false when they run alongside each other).
    auto corner = [&](const L &i, const L &j, double k, V3 &out) {
      V3 pi = i.a + i.in * (k * i.t), pj = j.a + j.in * (k * j.t);
      double den = cross2(i.d, j.d);
      if (std::fabs(den) < 1e-12) return false;
      out = pi + i.d * (cross2(pj - pi, j.d) / den);
      return true;
    };
    double k = 0;
    for (int guard = 0; lines.size() >= 3 && guard < 10000; guard++) {
      size_t m = lines.size();
      // Each side's length now and a unit of time on; the first to reach nothing.
      double soonest = INFINITY;
      bool gone = false;
      for (size_t j = 0; j < m && !gone; j++) {
        const L &pr = lines[(j + m - 1) % m], &me = lines[j], &nx = lines[(j + 1) % m];
        V3 s0, e0, s1, e1v;
        if (!corner(pr, me, k, s0) || !corner(me, nx, k, e0) || !corner(pr, me, k + 1, s1) || !corner(me, nx, k + 1, e1v)) {
          gone = true;
          break;
        }
        double now = dot(e0 - s0, me.d), later = dot(e1v - s1, me.d), shrink = now - later;
        if (shrink > 1e-12) soonest = std::min(soonest, k + std::max(now, 0.0) / shrink);
      }
      if (gone || !std::isfinite(soonest)) {
        if (gone) room = std::min(room, k);
        break;
      }
      k = soonest;
      // Sides gone by now dropped; two facing each other left side by side: the face is gone.
      std::vector<L> left;
      for (size_t j = 0; j < m; j++) {
        const L &pr = lines[(j + m - 1) % m], &me = lines[j], &nx = lines[(j + 1) % m];
        V3 s0, e0;
        bool ok = corner(pr, me, k, s0) && corner(me, nx, k, e0);
        if (ok && dot(e0 - s0, me.d) > 1e-9 * (1 + k)) left.push_back(me);
      }
      if (left.size() == lines.size()) break;  // never expected: nothing dropped
      lines.swap(left);
      for (size_t j = 0; j < lines.size(); j++)
        if (dot(lines[j].d, lines[(j + 1) % lines.size()].d) < -1 + 1e-9) lines.clear();
      // Two left side by side running the same way: one (the first's line).
      for (size_t j = 0; lines.size() > 3 && j < lines.size();) {
        if (dot(lines[j].d, lines[(j + 1) % lines.size()].d) > 1 - 1e-12) lines.erase(lines.begin() + (long)((j + 1) % lines.size()));
        else j++;
      }
    }
    if (lines.size() < 3) room = std::min(room, k);
  }
  return room;
}

}  // namespace

// With `spare`, every edge asked for: edges beside a hair's remnant of a face left as they are.
static Solid treatedAs(const Solid &s, const Treatment &t, double d, TreatFit &fit, const Solid *onto, bool spare) {
  fit = TreatFit();
  const Solid &base = onto ? *onto : s;
  std::vector<Crease> creases = creasesOf(s, t.kinds.data(), t.picks.data(), (int)t.kinds.size(), &fit.missing);
  // Bevels and coves skip edges where the faces meet flat (and coves inside corners too).
  double size = 0;
  for (V3 q : s.p) size = std::max({size, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
  double tol = 1e-7 * (1 + size);
  bool picked = false;
  for (int k : t.kinds) picked = picked || k == BK_PICK_EDGE || k == BK_PICK_CORNER;
  bool whole = spare;
  // How many edges and faces were left as they are for that (none: nothing to try again for).
  int spared = 0;
  // Each face's area (told when first asked).
  std::vector<double> areas;
  auto areaOf = [&](int f) {
    if (areas.empty()) {
      areas.assign(s.faces.size(), 0);
      for (size_t t3 = 0; t3 < s.triFace.size(); t3++)
        areas[s.triFace[t3]] += norm(cross(s.p[s.tri[3 * t3 + 1]] - s.p[s.tri[3 * t3]], s.p[s.tri[3 * t3 + 2]] - s.p[s.tri[3 * t3]])) / 2;
    }
    return areas[f];
  };
  // How far along a crease its own middle lies: where a run goes on into a seam (two roundings meeting at a corner, going
  // on from the sharp edge that ends there), the middle of its longest edge that isn't one, else its middle.
  auto mainAlongOf = [&](const Crease &c) {
    double whole = c.length / 2, run = 0, longest = -1;
    size_t at0 = 0;
    bool seams = false;
    for (int e : c.edges) {
      size_t a = at0, b = at0 + s.edges[e].pts.size() - 1;
      if (b >= c.pts.size()) return c.length / 2;
      double len = 0;
      for (size_t i = a; i < b; i++) len += norm(c.pts[i + 1] - c.pts[i]);
      size_t m = (a + b) / 2;
      if (s.faces[c.fa[m]].blend && s.faces[c.fb[m]].blend) {
        seams = true;
      } else if (len > longest) {
        longest = len, whole = run + len / 2;
      }
      run += len, at0 = b;
    }
    return seams && longest >= 0 ? whole : c.length / 2;
  };
  std::vector<Crease> work;
  for (auto &c : creases) {
    if (t.kind == Treatment::Cove && c.angle >= 179) continue;
    if (std::fabs(c.angle - 180) <= 1) continue;
    // Beside a rounding made before: an edge near flat is where the rounding meets a face smoothly (its mesh bends there by
    // as much as a step round the rounding), and one beside a part of it no wider than a few chord errors is where it met
    // a face curving away from it (whose mesh lies that far off the face itself).
    bool blendA = false, blendB = false, seam = true;
    for (size_t k = 0; k < c.pts.size(); k++) {
      blendA = blendA || s.faces[c.fa[k]].blend, blendB = blendB || s.faces[c.fb[k]].blend;
      seam = seam && s.faces[c.fa[k]].blend && s.faces[c.fb[k]].blend;
    }
    Crease m = c;
    size_t at = pointAt(m, mainAlongOf(c));
    double angle = m.angleAt(at);
    if ((blendA || blendB) && std::fabs(angle - 180) < 35) continue;
    // Beside a rounding, a knife's edge (its faces a few degrees apart): where the rounding runs out against a face at a
    // graze, with nothing there to treat, unless picked as such.
    if ((blendA || blendB) && !picked && (angle < 15 || angle > 345)) continue;
    // Two roundings either side all along (the seam where they cross at a corner): part of that corner, not an edge of the
    // shape, unless picked as such (or going on from a sharp edge: then part of its run).
    if (seam && !picked) continue;
    if (blendA || blendB || (whole && std::min(areaOf(c.fa[at]), areaOf(c.fb[at])) < 1.0)) {
      // (By how far the outline goes before it turns: where the faces fold back on each other, nowhere.) With every edge
      // asked for, beside any face no wider than a few chord errors (a hair's remnant of a face where roundings met): no
      // edge of the shape's to treat either.
      Runs r = runsAt(s, m, at, {}, std::max(tol, d), nullptr, 0, false);
      if ((blendA && r.a < 4 * d) || (blendB && r.b < 4 * d)) continue;
      if (whole && (r.a < 4 * d || r.b < 4 * d)) {
        spared++;
        continue;
      }
    }
    work.push_back(c);
  }
  if (work.empty() || (t.kind != Treatment::Bevel && t.radius < 0.005) || (t.kind == Treatment::Bevel && std::min(t.legA, t.legB) < 0.005)) return base;

  // Will it fit: each face beside a crease must hold what the treatment takes of it: in the cut across the crease's middle,
  // as far as the face runs (with another treated crease across it, what both take together); and a flat face, its sides
  // all moved in by what each treatment takes, must keep some of itself (faceRoom).
  double most = INFINITY;
  std::vector<double> setback(s.edges.size(), 0), legs(s.edges.size(), 0);
  std::set<int> flats, consumed;
  // Inside-corner roundings run on past a narrow face: by crease, the faces beyond that cut what they fill.
  std::map<size_t, std::vector<int>> fillCut;
  // (Cut a hair short of that face, should cutting on it leave the merge open.)
  bool fillHair = false;
  for (const auto &c : work) {
    for (size_t q = 0; q < c.edges.size(); q++) {
      int e = c.edges[q];
      const auto &pts = s.edges[e].pts;
      V3 at = pts[pts.size() / 2];
      size_t j = 0;
      for (size_t k = 1; k < c.pts.size(); k++)
        if (norm(c.pts[k] - at) < norm(c.pts[j] - at)) j = k;
      double phi = c.angleAt(j) * pi / 180, half = (phi < pi ? phi : 2 * pi - phi) / 2;
      setback[e] = t.kind == Treatment::Round ? 1 / std::tan(half) : 1;
    }
    for (size_t k = 0; k < c.pts.size(); k++)
      for (int f : {c.fa[k], c.fb[k]})
        if (s.faces[f].geom.flat) flats.insert(f);
  }
  for (const auto &whole : work) {
    // Measured across its middle (a point of its own there: a straight edge has only its ends).
    Crease c = whole;
    size_t mid = pointAt(c, mainAlongOf(whole));
    Runs r = runsAt(s, c, mid, work, std::max(tol, d), &whole, t.kind == Treatment::Round ? t.radius : 0);
    double la = r.aShared ? r.a / 2 : r.a, lb = r.bShared ? r.b / 2 : r.b;
    double phi = c.angleAt(mid) * pi / 180, half = (phi < pi ? phi : 2 * pi - phi) / 2;
    if (t.kind == Treatment::Round) {
      // Along a face shared with another rounding, the two reaches together (r·cot of each half angle) fill it at most.
      double own = 1 / std::tan(half);
      double ra = r.a / (own + (r.aShared ? r.aOther : 0)), rb = r.b / (own + (r.bShared ? r.bOther : 0));
      // A face too narrow for it, open past its end and no other rounding's: the rounding runs on past it (spills over),
      // taking all of it, and ends on the next face as that face cuts across its circle. One side at most, at an outside
      // corner (one taken away), the other side holding it as it is.
      bool spillA = false, spillB = false;
      if ((ra < t.radius) != (rb < t.radius)) {
        // Its circle in the end-on frame (u into face A, v A's normal; B's way in at the corner's angle): in the material
        // at an outside corner, in the air at an inside one.
        V3 C = p2(t.radius / std::tan(half), phi < pi ? -t.radius : t.radius);
        auto crosses = [&](V3 at, V3 way) {
          V3 w = C - at;
          return std::fabs(w.x * way.y - w.y * way.x) < t.radius * (1 - 1e-6);
        };
        // At an inside corner, what it fills past the narrow face is cut off by the flat face beyond (as OpenCascade's
        // kernel does): it ends there, square to that face.
        auto flatNext = [&](int f) { return phi < pi || (f >= 0 && s.faces[f].geom.flat); };
        spillA = ra < t.radius && r.aOpen && !r.aShared && crosses(r.aNext, r.aWay) && flatNext(r.aNextFace);
        spillB = rb < t.radius && r.bOpen && !r.bShared && crosses(r.bNext, r.bWay) && flatNext(r.bNextFace);
        size_t ci = (size_t)(&whole - work.data());
        if (phi > pi && spillA) fillCut[ci].push_back(r.aNextFace);
        if (phi > pi && spillB) fillCut[ci].push_back(r.bNextFace);
      }
      if (spillA) ra = INFINITY, consumed.insert(c.fa[mid]);
      if (spillB) rb = INFINITY, consumed.insert(c.fb[mid]);
      // A face curved in the cut: where the rounding's circle touches it as it bends (not r·cot along its tangent), within
      // its share of the run; the largest radius that does, by halving. Measured at a point of the edge's own (one put
      // between two lies a chord's sag off the edge, and so does where it measures from).
      size_t v = 0;
      for (size_t k = 1; k < whole.pts.size(); k++)
        if (norm(whole.pts[k] - c.pts[mid]) < norm(whole.pts[v] - c.pts[mid])) v = k;
      const Crease &w = whole;
      V3 U = w.ia[v], N = unit(w.na[v] - U * dot(w.na[v], U));
      V3 db = unit(p2(dot(w.ib[v], U), dot(w.ib[v], N))), nb = unit(p2(dot(w.nb[v], U), dot(w.nb[v], N)));
      Cut2 A = bendOf(s.faces[w.fa[v]].geom, w.pts[v], U, N, p2(1, 0), p2(0, 1), w.na[v]),
           B = bendOf(s.faces[w.fb[v]].geom, w.pts[v], U, N, db, nb, w.nb[v]);
      if (A.circle || B.circle) r = runsAt(s, whole, v, work, std::max(tol, d), &whole, t.radius);
      if (A.circle || B.circle) {
        // Its own reach and the other's (as the radius asked) together within the run.
        auto fitsAt = [&](double rr) {
          V3 C, TA, TB, nA, nB;
          bool convex;
          return rollAt(A, B, rr, C, TA, TB, nA, nB, convex) && (spillA || alongCut(A, TA) + (r.aShared ? r.aOther * rr : 0) <= r.a) &&
                 (spillB || alongCut(B, TB) + (r.bShared ? r.bOther * rr : 0) <= r.b);
        };
        double lo = 0, hi = std::max({ra, rb, 1e-6}) * 4;
        while (fitsAt(hi) && hi < 1e6) lo = hi, hi *= 2;
        for (int k = 0; k < 60; k++) {
          double m = (lo + hi) / 2;
          (fitsAt(m) ? lo : hi) = m;
        }
        ra = rb = lo;
      }
      most = std::min(most, std::min(ra, rb));
    } else if (t.kind == Treatment::Cove) {
      most = std::min(most, std::min(la, lb));
    } else if (t.legA > la * (1 - 1e-9) || t.legB > lb * (1 - 1e-9)) {
      fit.fits = false;
      fit.why = "bevel: too large for these edges";
      return s;
    }
  }
  for (int f : flats) {
    // With every edge asked for, a hair's remnant of a face (where roundings met) may be used up.
    if (whole && areaOf(f) < 16 * d * d) {
      spared++;
      continue;
    }
    if (t.kind != Treatment::Bevel) {
      if (!consumed.count(f)) most = std::min(most, faceRoom(s, f, setback));
      continue;
    }
    // A bevel's legs: each edge's on this face (A's or B's by which side of its crease the face is).
    std::fill(legs.begin(), legs.end(), 0.0);
    for (const auto &c : work) {
      size_t at0 = 0;
      for (size_t q = 0; q < c.edges.size(); q++) {
        int e = c.edges[q];
        // By the side of the crease the face is on along this edge (a run's faces change where it goes on into a seam).
        size_t m = std::min(at0 + (s.edges[e].pts.size() - 1) / 2, c.fa.size() - 1);
        at0 += s.edges[e].pts.size() - 1;
        if (s.edges[e].f0 == f || s.edges[e].f1 == f) legs[e] = c.fa[m] == f ? t.legA : t.legB;
      }
    }
    if (faceRoom(s, f, legs) < 1 - 1e-9) {
      fit.fits = false;
      fit.why = "bevel: too large for these edges";
      return s;
    }
  }
  if (spare && !spared) {
    fit.fits = false;
    return s;
  }
  // An inward rounding past that may still do (as OpenCascade's kernel takes it): cut, then checked (below).
  bool coveChecked = t.kind == Treatment::Cove && t.radius >= most * (1 - 1e-6);
  auto tooWide = [&]() {
    fit.fits = false;
    fit.most = std::max(0.0, std::floor(most * 0.999 * 100) / 100);
    fit.why = t.kind == Treatment::Round ? "rounding too large" : "cove too large";
    return s;
  };
  if (t.kind != Treatment::Bevel && t.radius >= most * (1 - 1e-6) && !coveChecked) return tooWide();
  auto tooLarge = [&]() {
    fit.fits = false;
    fit.why = t.kind == Treatment::Bevel ? "bevel: too large for these edges" : "too large for these edges";
    return s;
  };

  // Straight edges between flat faces, rounded outward: lines, joined at corners by balls where every edge there is one.
  std::vector<Line> lines;
  std::vector<char> isLine(work.size(), 0);
  if (t.kind == Treatment::Round) {
    for (size_t i = 0; i < work.size(); i++) {
      const Crease &c = work[i];
      if (c.closed || c.edges.size() != 1 || c.angle >= 179) continue;
      bool flat = s.faces[c.fa[0]].geom.flat && s.faces[c.fb[0]].geom.flat;
      V3 T = unit(c.pts.back() - c.pts.front());
      for (V3 q : c.pts) flat = flat && norm(cross(q - c.pts.front(), T)) <= tol * 10;
      if (!flat) continue;
      Line L;
      L.c = &c, L.E0 = c.pts.front(), L.E1 = c.pts.back(), L.T = T, L.fa = c.fa[0], L.fb = c.fb[0];
      L.na = s.faces[L.fa].geom.pn, L.nb = s.faces[L.fb].geom.pn;
      double alpha = std::acos(std::clamp(dot(L.na, L.nb), -1.0, 1.0));
      L.arcs = std::max(1, (int)std::ceil(alpha / (2 * std::acos(std::clamp(1 - d / t.radius, -1.0, 1.0))) - 1e-9));
      L.arcs = std::min(std::max(L.arcs, (int)std::ceil(alpha / 0.35 - 1e-9)), 64);
      lines.push_back(L);
      isLine[i] = 1;
    }
  }
  std::vector<Ball> balls;
  {
    std::map<std::tuple<double, double, double>, std::vector<std::pair<int, int>>> at;  // corner → (line, end)
    for (size_t l = 0; l < lines.size(); l++) {
      at[{lines[l].E0.x, lines[l].E0.y, lines[l].E0.z}].push_back({(int)l, 0});
      at[{lines[l].E1.x, lines[l].E1.y, lines[l].E1.z}].push_back({(int)l, 1});
    }
    for (auto &[key, ends] : at) {
      V3 V{std::get<0>(key), std::get<1>(key), std::get<2>(key)};
      if (ends.size() < 3) continue;
      int creasesHere = 0;
      for (const auto &e : s.edges)
        if (e.f0 >= 0 && e.f1 >= 0 && e.f0 != e.f1 && (e.pts.front() == V || e.pts.back() == V)) creasesHere++;
      if (creasesHere != (int)ends.size()) continue;
      std::vector<int> faces = facesAt(s, V);
      bool flat = faces.size() == ends.size();
      for (int f : faces) flat = flat && s.faces[f].geom.flat;
      if (!flat) continue;
      // The faces and lines in turn round the corner.
      Ball ball;
      ball.V = V;
      std::vector<int> ls;
      for (auto [l, e] : ends) ls.push_back(l);
      int cur = ls[0], face = lines[cur].fa;
      std::set<int> used;
      bool ok = true;
      for (size_t k = 0; k < ls.size() && ok; k++) {
        used.insert(cur);
        ball.faces.push_back(face);
        ball.lines.push_back(cur);
        int other = lines[cur].fa == face ? lines[cur].fb : lines[cur].fa, next = -1;
        for (int l : ls)
          if (!used.count(l) && (lines[l].fa == other || lines[l].fb == other)) next = l;
        if (k + 1 < ls.size()) {
          if (next < 0) ok = false;
          cur = next, face = other;
        }
      }
      if (!ok || ball.lines.size() != ls.size()) continue;
      // The ball's middle: the radius inside every face there (least squares, then checked).
      double ata[3][3] = {}, atb[3] = {};
      for (int f : ball.faces) {
        V3 n = s.faces[f].geom.pn;
        double b = s.faces[f].geom.pd - t.radius;
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) ata[i][j] += n[i] * n[j];
          atb[i] += n[i] * b;
        }
      }
      auto det3 = [](double m[3][3]) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
      };
      double det = det3(ata);
      if (std::fabs(det) < 1e-12) continue;
      for (int k = 0; k < 3; k++) {
        double m[3][3];
        for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++) m[i][j] = j == k ? atb[i] : ata[i][j];
        ball.C[k] = det3(m) / det;
      }
      for (int f : ball.faces) ok = ok && std::fabs(dot(s.faces[f].geom.pn, ball.C) - (s.faces[f].geom.pd - t.radius)) < 1e-6 * (1 + size);
      if (!ok) continue;
      int bi = (int)balls.size();
      balls.push_back(ball);
      for (auto [l, e] : ends) lines[l].ball[e] = bi;
    }
  }

  // Every edge rounded on a solid of flat faces meeting only at balls: the rounded solid made outright.
  if (!onto && t.kind == Treatment::Round && !lines.empty() && lines.size() == work.size()) {
    size_t edgesHere = 0;
    for (const auto &e : s.edges)
      if (e.f0 >= 0 && e.f1 >= 0 && e.f0 != e.f1) edgesHere++;
    bool all = edgesHere == lines.size();
    for (const auto &L : lines) all = all && L.ball[0] >= 0 && L.ball[1] >= 0;
    Solid whole;
    if (all && roundedWhole(s, lines, balls, t.radius, d, whole)) {
      // Its curved faces are roundings, as any rounding's are (meeting the faces beside them smoothly).
      for (auto &f : whole.faces)
        if (!f.geom.flat) f.blend = true;
      finish(whole, d);
      if (pieces(whole) == pieces(s)) return whole;
    }
  }

  std::vector<Solid> take, add;
  // A tool made one of pieces (a chain's): those pieces, by which list and where in it, to cut one by one should the whole
  // fail.
  std::map<std::pair<bool, size_t>, std::vector<Solid>> piecesOf;
  if (!lines.empty()) {
    std::vector<Solid> tools = lineTools(s, lines, balls, t.radius, d, tol);
    if (tools.empty()) return tooLarge();
    for (auto &tool : tools) take.push_back(std::move(tool));
  }
  // Inward roundings meeting at a corner: each there a hair wider than the others, so their cylinders cross rather than
  // touch (alone, each exact).
  std::vector<int> shade(work.size(), 0);
  if (t.kind == Treatment::Cove)
    for (size_t i = 0; i < work.size(); i++) {
      if (work[i].closed) continue;
      std::set<int> taken;
      for (size_t j = 0; j < i; j++) {
        if (work[j].closed) continue;
        bool meets = false;
        for (V3 p : {work[i].pts.front(), work[i].pts.back()})
          for (V3 q : {work[j].pts.front(), work[j].pts.back()}) meets = meets || norm(p - q) <= tol * 10;
        if (meets) taken.insert(shade[j]);
      }
      while (taken.count(shade[i])) shade[i]++;
    }
  // One merge or cut; should it leave a hole, again with crossing points a little farther apart made one (a tool touching
  // a face along a line crosses it only roughly there).
  auto step = [](const Solid &a, const Solid &b, int op) {
    Solid r = combine(a, b, op);
    for (double merge : {1e-9, 1e-7, 1e-6}) {
      if (closed(r)) break;
      r = combine(a, b, op, merge);
    }
    return r;
  };
  // A run of edges meeting smoothly (a face's edges round a rounded corner): each edge its own piece. Straight ones
  // between flat faces are prisms, running on through a corner between two of them that turns away from the material
  // (where what they take is taken by the corner's piece too); arcs between faces turned round their axis (or flat across
  // it) are the section carried round that axis through the solid's own steps, cut off at the axis; anything else is
  // swept. Pieces meeting share the ring of points where they meet, a straight one and an arc ending there both; the
  // pieces of each section then made one tool (cut one by one, two meeting there would leave a face between them).
  auto chain = [&](const Crease &c, const Treatment &tw, int which) {
    std::map<std::pair<bool, size_t>, std::vector<Solid>> made;
    auto put = [&](bool fill, size_t q, Solid tool) { made[{fill, q}].push_back(std::move(tool)); };
    std::vector<std::pair<size_t, size_t>> range;
    size_t at0 = 0;
    for (int e : c.edges) {
      size_t n = s.edges[e].pts.size();
      if (n < 2) return false;
      range.push_back({at0, at0 + n - 1});
      at0 += n - 1;
    }
    if (at0 + 1 != c.pts.size()) return false;
    size_t np = range.size();
    struct Info {
      int kind = 2;  // 0 straight, 1 arc, 2 swept
      size_t mid = 0;
      V3 centre, axis, U, N;
      double R = 0;
      bool convex = true;
    };
    std::vector<Info> info(np);
    auto exactFrame = [&](size_t i, V3 centre, V3 axis, V3 &U, V3 &N) {
      V3 radial = (c.pts[i] - centre) - axis * dot(c.pts[i] - centre, axis), along = unit(cross(axis, radial));
      V3 exact = unit(cross(along, c.na[i]));
      U = dot(exact, c.ia[i]) < 0 ? -exact : exact;
      N = unit(c.na[i] - U * dot(c.na[i], U));
    };
    for (size_t k = 0; k < np; k++) {
      auto [a, b] = range[k];
      Info &in = info[k];
      in.mid = std::max(a + 1, (a + b) / 2);
      size_t m = in.mid;
      const FaceGeom &ga = s.faces[c.fa[m]].geom, &gb = s.faces[c.fb[m]].geom;
      const Solid::Edge &edge = s.edges[c.edges[k]];
      bool joined[2] = {k > 0 || c.closed, k + 1 < np || c.closed};
      if (ga.flat && gb.flat) {
        V3 T = unit(c.pts[b] - c.pts[a]);
        bool straight = true;
        for (size_t i = a; i <= b; i++) straight = straight && norm(cross(c.pts[i] - c.pts[a], T)) <= tol * 10;
        if (straight) {
          in.kind = 0, in.U = c.ia[m], in.N = unit(c.na[m] - in.U * dot(c.na[m], in.U));
          continue;
        }
      }
      if (edge.geom.kind == EdgeGeom::Circle && edge.geom.exact && joined[0] && joined[1]) {
        V3 centre = edge.geom.place.point({0, 0, edge.geom.z}), axis = unit(edge.geom.place.vector({0, 0, 1}));
        bool coaxial = true;
        for (const FaceGeom *g : {&ga, &gb}) {
          if (g->flat) {
            coaxial = coaxial && norm(cross(g->pn, axis)) < 1e-9;
          } else if (g->kind == FaceGeom::Turned && g->exact) {
            V3 ax = unit(g->place.vector({0, 0, 1})), o = g->place.point({0, 0, 0});
            V3 off = (o - centre) - axis * dot(o - centre, axis);
            coaxial = coaxial && norm(cross(ax, axis)) < 1e-9 && norm(off) < tol * 10;
          } else {
            coaxial = false;
          }
        }
        if (coaxial) {
          in.kind = 1, in.centre = centre, in.axis = axis, in.R = norm(edge.geom.place.vector({edge.geom.r, 0, 0}));
          V3 p = c.pts[m], toAxis = (centre + axis * dot(p - centre, axis)) - p;
          in.convex = dot(toAxis, c.ia[m] + c.ib[m]) > 0;
          exactFrame(m, centre, axis, in.U, in.N);
        }
      }
    }
    // One section all along (its faces meet at one angle all along a smooth run), from a straight piece's frame if any.
    size_t ref = np;
    for (size_t k = 0; k < np && ref == np; k++)
      if (info[k].kind == 0) ref = k;
    for (size_t k = 0; k < np && ref == np; k++)
      if (info[k].kind == 1) ref = k;
    std::vector<Section2> secs;
    if (ref < np) {
      Crease cr = c;
      size_t m = info[ref].mid;
      cr.ia[m] = info[ref].U;
      if (info[ref].kind == 1) {
        V3 Ub, Nb;
        Crease flip = c;
        std::swap(flip.na[m], flip.nb[m]), std::swap(flip.ia[m], flip.ib[m]);
        exactFrame(m, info[ref].centre, info[ref].axis, Ub, Nb);
        (void)Nb;
        V3 radial = (c.pts[m] - info[ref].centre) - info[ref].axis * dot(c.pts[m] - info[ref].centre, info[ref].axis), along = unit(cross(info[ref].axis, radial));
        V3 exactB = unit(cross(along, c.nb[m]));
        cr.ib[m] = dot(exactB, c.ib[m]) < 0 ? -exactB : exactB;
      }
      bool fits = true;
      secs = sectionsAt(cr, m, tw, which, &fits);
      if (!fits) return false;
    }
    double span = spanOf(secs);
    for (size_t k = 0; k < np; k++) {
      auto [a, b] = range[k];
      const Info &in = info[k];
      int before = k > 0 ? (int)k - 1 : c.closed ? (int)np - 1 : -1, after = k + 1 < np ? (int)k + 1 : c.closed ? 0 : -1;
      if (in.kind == 0) {
        V3 T = unit(c.pts[b] - c.pts[a]);
        for (size_t q = 0; q < secs.size(); q++) {
          const Section2 &sec = secs[q];
          // How far each end runs on: through a corner turning away, a hair into anything else; at a free end onto the
          // face there (or a hair short), what's added stopping on it.
          double ext[2];
          int endFace[2] = {-1, -1};
          for (int e = 0; e < 2; e++) {
            int o = e == 0 ? before : after;
            V3 V = c.pts[e == 0 ? a : b], out = e == 0 ? -T : T;
            if (o >= 0) {
              const Info &n = info[o];
              bool through = !sec.fill && t.kind != Treatment::Cove && (n.kind == 0 || (n.kind == 1 && n.convex));
              ext[e] = sec.fill ? 0 : through ? (n.kind == 1 ? n.R : 0) + span + reachOut(span) : hair(span);
              continue;
            }
            std::vector<int> others;
            for (int f : facesAt(s, V))
              if (f != c.fa[in.mid] && f != c.fb[in.mid]) others.push_back(f);
            bool onto = others.size() == 1 && s.faces[others[0]].geom.flat && dot(s.faces[others[0]].geom.pn, out) > 0.05;
            double square = onto ? dot(s.faces[others[0]].geom.pn, out) : 0;
            double reach = onto ? (span + 0.45 * reachOut(span)) / square * 1.2 + span : -hair(span);
            ext[e] = !sec.fill ? reach : !onto ? -hair(span) : square > 1 - 1e-12 ? 0 : reach;
            if (sec.fill && onto && square <= 1 - 1e-12) endFace[e] = others[0];
          }
          std::vector<V3> stations;
          if (ext[0] > 0) stations.push_back(c.pts[a] - T * ext[0]);
          stations.push_back(ext[0] < 0 ? c.pts[a] - T * ext[0] : c.pts[a]);
          stations.push_back(ext[1] < 0 ? c.pts[b] + T * ext[1] : c.pts[b]);
          if (ext[1] > 0) stations.push_back(c.pts[b] + T * ext[1]);
          Solid tool = prismTool(sec, stations, in.U, in.N, T, d);
          for (int e = 0; e < 2; e++)
            if (endFace[e] >= 0) tool = cut(tool, c.pts[e == 0 ? a : b], -s.faces[endFace[e]].geom.pn, 0);
          put(sec.fill, q, std::move(tool));
        }
        continue;
      }
      if (in.kind == 1) {
        // The steps: the run's points on the circle (where the solid's mesh steps round it), the ends shared with the
        // pieces either side (in a straight one's frame where it's one).
        std::vector<V3> E, U, N;
        double R = in.R;
        V3 planeCentre = in.centre;
        for (size_t i = a; i <= b; i++) {
          V3 radial = (c.pts[i] - planeCentre) - in.axis * dot(c.pts[i] - planeCentre, in.axis);
          bool end = i == a || i == b;
          if (!end && std::fabs(norm(radial) - R) > 1e-9 * (1 + R)) continue;
          V3 u, n;
          int o = i == a ? before : after;
          if (end && o >= 0 && info[o].kind == 0) {
            u = info[o].U, n = info[o].N;
          } else {
            exactFrame(i, in.centre, in.axis, u, n);
          }
          E.push_back(end ? c.pts[i] : planeCentre + in.axis * dot(c.pts[i] - planeCentre, in.axis) + unit(radial) * R);
          U.push_back(u), N.push_back(n);
        }
        // Cut off at the axis: in the section's frame at the middle step, the distance from the axis is R + u·Ur + v·Nr.
        V3 Uref = in.U, Nref = in.N, radialRef = unit((c.pts[in.mid] - in.centre) - in.axis * dot(c.pts[in.mid] - in.centre, in.axis));
        V3 g = p2(dot(Uref, radialRef), dot(Nref, radialRef));
        for (size_t q = 0; q < secs.size(); q++) {
          const Section2 &sec = secs[q];
          bool cutAway;
          Section2 part = clipped(asRuns(sec, d), g, -R, cutAway);
          if (part.runs.size() < 2) continue;
          Solid tool = turnTool(part, E, U, N, in.centre, in.axis, d);
          if (tool.tri.empty()) return false;
          put(sec.fill, q, std::move(tool));
        }
        continue;
      }
      // Swept from point to point, each its own frame and section; a hair on past a joint, onto the face at a free end.
      // Where its faces come to meet flat at a free end (a seam between two roundings, going on from a sharp edge, ending
      // where both meet a third face smoothly) the treatment comes to nothing: it stops where they last meet at an angle
      // (its sections there the same way out as the run's).
      bool stopped[2] = {false, false};
      auto flat = [&](size_t i) { return std::fabs(c.angleAt(i) - 180) < 0.3; };
      if (before < 0)
        while (a < b && flat(a)) a++, stopped[0] = true;
      if (after < 0)
        while (b > a && flat(b)) b--, stopped[1] = true;
      if (b == a) continue;
      size_t n = b - a + 1;
      std::vector<std::vector<Section2>> per(n);
      bool fits = true;
      int convexity = 0;
      if (stopped[0] || stopped[1]) {
        Crease m = c;
        convexity = m.angleAt(pointAt(m, mainAlongOf(c))) < 180 ? 1 : -1;
      }
      for (size_t i = 0; i < n; i++) per[i] = sectionsAt(c, a + i, tw, which, &fits, false, convexity, &s);
      if (!fits) return false;
      size_t count = per[0].size();
      for (size_t i = 0; i < n; i++)
        if (per[i].size() != count) return false;
      bool alike = true;
      for (size_t i = 1; i < n; i++)
        for (size_t q = 0; q < count; q++) alike = alike && per[i][q].runs.size() == per[0][q].runs.size();
      if (!alike)
        for (size_t i = 0; i < n; i++) per[i] = sectionsAt(c, a + i, tw, which, &fits, true, convexity, &s);
      double spanHere = 0;
      for (const auto &p : per) spanHere = std::max(spanHere, spanOf(p));
      for (size_t q = 0; q < count; q++) {
        std::vector<int> chords = chordsOf(per[0][q], d);
        for (size_t i = 1; i < n; i++) {
          std::vector<int> here = chordsOf(per[i][q], d);
          for (size_t j = 0; j < chords.size() && j < here.size(); j++) chords[j] = std::max(chords[j], here[j]);
        }
        std::vector<V3> E, U, N;
        std::vector<std::vector<V3>> outlines;
        std::vector<std::vector<double>> radii;
        std::vector<int> runOf;
        for (size_t i = 0; i < n; i++) {
          V3 u = c.ia[a + i];
          E.push_back(c.pts[a + i]), U.push_back(u), N.push_back(unit(c.na[a + i] - u * dot(c.na[a + i], u)));
          outlines.push_back(pointsOf(per[i][q], chords, i == 0 ? &runOf : nullptr));
          radii.push_back(radiiOf(per[i][q], runOf));
        }
        for (int e = 0; e < 2; e++) {
          size_t i = e == 0 ? a : b;
          V3 T = c.tangent(i) * (e == 0 ? -1.0 : 1.0);
          double reach = hair(spanHere);
          if (stopped[e]) continue;
          if ((e == 0 ? before : after) < 0) {
            std::vector<int> others;
            for (int f : facesAt(s, c.pts[i]))
              if (f != c.fa[i] && f != c.fb[i]) others.push_back(f);
            bool onto = others.size() == 1 && s.faces[others[0]].geom.flat && dot(s.faces[others[0]].geom.pn, T) > 0.05;
            reach = onto ? (spanHere + 0.45 * reachOut(spanHere)) / dot(s.faces[others[0]].geom.pn, T) * 1.2 + spanHere : 0;
          }
          if (!(reach > 0)) continue;
          V3 Ex = c.pts[i] + T * reach;
          if (e == 0) {
            E.insert(E.begin(), Ex), U.insert(U.begin(), U.front()), N.insert(N.begin(), N.front()), outlines.insert(outlines.begin(), outlines.front());
            radii.insert(radii.begin(), radii.front());
          } else {
            E.push_back(Ex), U.push_back(U.back()), N.push_back(N.back()), outlines.push_back(outlines.back()), radii.push_back(radii.back());
          }
        }
        // A seam swept round a bend tighter than its section reaches across folds through itself (each point of the section
        // must move on along the run from one step to the next): left as it is, the run's other pieces made.
        size_t mid = (a + b) / 2;
        if (s.faces[c.fa[mid]].blend && s.faces[c.fb[mid]].blend) {
          bool folds = false;
          for (size_t i = 0; i + 1 < E.size() && !folds; i++) {
            V3 T = unit(E[i + 1] - E[i]);
            for (size_t j = 0; j < outlines[i].size() && !folds; j++)
              folds = dot(at3(E[i + 1], U[i + 1], N[i + 1], outlines[i + 1][j]) - at3(E[i], U[i], N[i], outlines[i][j]), T) <= 0;
          }
          if (folds) continue;
        }
        put(per[0][q].fill, q, sweptTool(E, U, N, outlines, false, runOf, &radii));
      }
    }
    for (auto &[key, parts] : made) {
      Solid one = parts[0];
      for (size_t i = 1; i < parts.size() && closed(one); i++) one = step(one, parts[i], BK_UNION);
      auto &into = key.first ? add : take;
      if (parts.size() > 1 && closed(one) && !one.tri.empty()) {
        piecesOf[{key.first, into.size()}] = std::move(parts);
        into.push_back(std::move(one));
      } else {
        for (auto &part : parts) into.push_back(std::move(part));
      }
    }
    return true;
  };

  // Every other crease's tools (inward roundings meeting at corners made `shadeBy` wider each, as shaded).
  std::vector<Solid> lineTake = take;
  auto tools = [&](double shadeBy) {
    take = lineTake, add.clear(), piecesOf.clear();
    int which = 0;
    for (size_t ci = 0; ci < work.size(); ci++) {
      const Crease &c = work[ci];
      which++;
      if (isLine[ci]) continue;
      // Straight between two flat faces: one section, swept straight. Round a circle between faces turned round its axis:
      // turned. Otherwise from point to point.
      bool flatFaces = true;
      for (size_t i = 0; i < c.pts.size(); i++) flatFaces = flatFaces && s.faces[c.fa[i]].geom.flat && s.faces[c.fb[i]].geom.flat;
      bool straight = !c.closed && c.edges.size() == 1 && flatFaces;
      if (straight) {
        V3 T = unit(c.pts.back() - c.pts.front());
        for (V3 q : c.pts) straight = straight && norm(cross(q - c.pts.front(), T)) <= tol * 10;
      }
      const Solid::Edge &first = s.edges[c.edges[0]];
      bool ring = c.closed && c.edges.size() == 1 && first.geom.kind == EdgeGeom::Circle && first.geom.exact;
      V3 centre, axis;
      if (ring) {
        centre = first.geom.place.point({0, 0, first.geom.z});
        axis = unit(first.geom.place.vector({0, 0, 1}));
        // Both faces turned round that axis (or flat across it).
        for (int f : {c.fa[0], c.fb[0]}) {
          const FaceGeom &g = s.faces[f].geom;
          if (g.flat) {
            ring = ring && norm(cross(g.pn, axis)) < 1e-9;
          } else if (g.kind == FaceGeom::Turned && g.exact) {
            V3 ax = unit(g.place.vector({0, 0, 1})), o = g.place.point({0, 0, 0});
            V3 off = (o - centre) - axis * dot(o - centre, axis);
            // A ball's face is turned round any line through its middle.
            V3 gx = g.place.vector({1, 0, 0}), gy = g.place.vector({0, 1, 0}), gz = g.place.vector({0, 0, 1});
            bool even = std::fabs(norm(gx) - norm(gz)) < 1e-12 * norm(gz) && std::fabs(norm(gy) - norm(gz)) < 1e-12 * norm(gz) &&
                        std::fabs(dot(gx, gy)) + std::fabs(dot(gy, gz)) + std::fabs(dot(gz, gx)) < 1e-12 * dot(gz, gz);
            if (g.elem.arc && g.elem.cr == 0 && even) {
              V3 m = g.place.point({0, 0, g.elem.cz});
              V3 offM = (m - centre) - axis * dot(m - centre, axis);
              ring = ring && norm(offM) < tol * 10;
              continue;
            }
            ring = ring && norm(cross(ax, axis)) < 1e-9 && norm(off) < tol * 10;
          } else {
            ring = false;
          }
        }
      }
      Treatment tw = t;
      tw.radius = t.radius * (1 + shadeBy * shade[ci]);
      if (c.edges.size() > 1) {
        if (!chain(c, tw, which)) {
          tooLarge();
          return false;
        }
        continue;
      }
      if (straight) {
        bool fits = true;
        std::vector<Section2> secs = sectionsAt(c, 0, tw, which, &fits);
        if (!fits) {
          tooLarge();
          return false;
        }
        V3 E = c.pts.front(), T = unit(c.pts.back() - c.pts.front()), U = c.ia[0], N = unit(c.na[0] - U * dot(c.na[0], U));
        double len = norm(c.pts.back() - c.pts.front()), span = spanOf(secs);
        // Each end runs on past the face there (outside the solid, where it takes nothing); with no single flat face there to
        // run onto, it stops a hair short.
        // What's added at an inside corner stops on that face instead: there if it's square to the edge, else cut off by it.
        double s0 = 0, s1 = len, f0 = 0, f1 = len;
        int endFace[2] = {-1, -1};
        for (int end = 0; end < 2; end++) {
          V3 V = end == 0 ? c.pts.front() : c.pts.back(), out = end == 0 ? -T : T;
          std::vector<int> others;
          for (int f : facesAt(s, V))
            if (f != c.fa[0] && f != c.fb[0]) others.push_back(f);
          bool onto = others.size() == 1 && s.faces[others[0]].geom.flat && dot(s.faces[others[0]].geom.pn, out) > 0.05;
          double square = onto ? dot(s.faces[others[0]].geom.pn, out) : 0;
          double reach = onto ? (span + 0.45 * reachOut(span)) / square * 1.2 + span : -hair(span);
          double fill = !onto ? -hair(span) : square > 1 - 1e-12 ? 0 : reach;
          if (onto && square <= 1 - 1e-12) endFace[end] = others[0];
          if (end == 0) s0 = -reach, f0 = -fill;
          else s1 = len + reach, f1 = len + fill;
        }
        for (const auto &sec : secs) {
          if (!sec.fill) {
            take.push_back(prismTool(sec, {E + T * s0, E + T * s1}, U, N, T, d));
            continue;
          }
          Solid tool = prismTool(sec, {E + T * f0, E + T * f1}, U, N, T, d);
          for (int end = 0; end < 2; end++)
            if (endFace[end] >= 0) tool = cut(tool, end == 0 ? c.pts.front() : c.pts.back(), -s.faces[endFace[end]].geom.pn, 0);
          for (int f : fillCut[ci]) tool = cut(tool, s.faces[f].geom.pn * (s.faces[f].geom.pd - (fillHair ? hair(t.radius) : 0)), -s.faces[f].geom.pn, 0);
          add.push_back(tool);
        }
      } else if (ring) {
        // The cut across it on the circle itself, in the plane through the axis (the mesh's points may sit on its chords, its
        // into-directions lean along them).
        Crease cr = c;
        V3 radial = (c.pts[0] - centre) - axis * dot(c.pts[0] - centre, axis);
        V3 E = centre + unit(radial) * norm(first.geom.place.vector({first.geom.r, 0, 0})), along = unit(cross(axis, radial));
        for (auto [into, n] : {std::pair<V3 *, V3>{&cr.ia[0], c.na[0]}, {&cr.ib[0], c.nb[0]}}) {
          V3 exact = unit(cross(along, n));
          *into = dot(exact, *into) < 0 ? -exact : exact;
        }
        bool fits = true;
        std::vector<Section2> secs = sectionsAt(cr, 0, tw, which, &fits);
        if (!fits) {
          tooLarge();
          return false;
        }
        V3 U = cr.ia[0], N = unit(c.na[0] - U * dot(c.na[0], U));
        // A face turned from an arc is curved in this cut: the rounding meets it there, not its tangent at the edge; a bevel's
        // leg ends on it (that far from the edge in a straight line), not on its tangent.
        if (t.kind != Treatment::Cove && secs.size() == 1) {
          auto in2 = [&](V3 v) { return unit(p2(dot(v, U), dot(v, N))); };
          Cut2 A = cutOf(s.faces[c.fa[0]].geom, E, U, N, in2(cr.ia[0]), in2(c.na[0])), B = cutOf(s.faces[c.fb[0]].geom, E, U, N, in2(cr.ib[0]), in2(c.nb[0]));
          Section2 exact;
          if (t.kind == Treatment::Round && (A.circle || B.circle) && roundCurved(A, B, tw.radius, which, exact)) secs[0] = exact;
          if (t.kind == Treatment::Bevel && (A.circle || B.circle)) {
            bool ok;
            exact = bevelCorner(p2(0, 0), A.dir, A.n, B.dir, B.n, tw.legA, tw.legB, tw.corner, which, ok, &A, &B);
            if (ok) secs[0] = exact;
          }
        }
        // The circle's steps in the solid's mesh: its points on the circle itself, evenly round it (else the tool's own).
        int around = 0;
        V3 x0;
        {
          double R = norm(E - centre);
          std::vector<V3> on;
          for (V3 q : c.pts) {
            V3 rq = (q - centre) - axis * dot(q - centre, axis);
            if (std::fabs(norm(rq) - R) <= 1e-9 * (1 + R)) on.push_back(unit(rq));
          }
          if (on.size() >= 3) {
            // Counted from the point most square to the world's axes (where a shape's own steps start).
            V3 e1 = on[0];
            auto squareness = [](V3 q) { return std::max({std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)}); };
            for (V3 q : on)
              if (squareness(q) > squareness(e1)) e1 = q;
            V3 e2 = cross(axis, e1);
            std::vector<double> angle;
            for (V3 q : on) {
              double a = std::atan2(dot(q, e2), dot(q, e1));
              angle.push_back(a < -1e-9 ? a + 2 * pi : std::max(a, 0.0));
            }
            std::sort(angle.begin(), angle.end());
            angle.erase(std::unique(angle.begin(), angle.end(), [](double a, double b) { return b - a < 1e-9; }), angle.end());
            if (angle.size() > 1 && 2 * pi - angle.back() < 1e-9) angle.pop_back();
            size_t n = angle.size();
            bool even = n >= 3;
            for (size_t k = 0; k < n && even; k++) even = std::fabs(angle[k] - 2 * pi * k / n) < 1e-7;
            if (even) around = (int)n, x0 = e1;
          }
        }
        // A section reaching past the axis (a bevel wide on a small disc) cut off there: nothing lies beyond it.
        V3 out = (E - centre) - axis * dot(E - centre, axis), g = p2(dot(U, unit(out)), dot(N, unit(out)));
        for (auto &sec : secs) {
          bool cutAway;
          Section2 part = clipped(asRuns(sec, d), g, -norm(out), cutAway);
          if (cutAway && part.runs.size() >= 2) sec = part;
        }
        for (const auto &sec : secs) {
          Solid tool;
          if (!ringTool(sec, E, U, N, centre, axis, around, x0, d, tool)) {
            fit.fits = false;
            fit.why = "too large for this circle";
            return false;
          }
          (sec.fill ? add : take).push_back(tool);
        }
      } else {
        // Swept: each point its own frame and sections; an open run reaches on past its ends onto the face there.
        size_t n = c.closed ? c.pts.size() - 1 : c.pts.size();
        std::vector<std::vector<Section2>> per(n);
        bool fits = true;
        for (size_t i = 0; i < n; i++) per[i] = sectionsAt(c, i, tw, which, &fits, false, 0, &s);
        if (!fits) {
          tooLarge();
          return false;
        }
        size_t count = per[0].size();
        for (size_t i = 0; i < n; i++)
          if (per[i].size() != count) count = 0;
        if (count == 0) continue;
        double span = 0;
        for (const auto &p : per) span = std::max(span, spanOf(p));
        for (size_t k = 0; k < count; k++) {
          // As many points at every point of the run: each arc as many chords as the most it needs anywhere.
          std::vector<int> chords = chordsOf(per[0][k], d);
          for (size_t i = 1; i < n; i++) {
            std::vector<int> here = chordsOf(per[i][k], d);
            for (size_t j = 0; j < chords.size() && j < here.size(); j++) chords[j] = std::max(chords[j], here[j]);
          }
          std::vector<V3> E, U, N;
          std::vector<std::vector<V3>> outlines;
          std::vector<std::vector<double>> radii;
          std::vector<int> runOf;
          for (size_t i = 0; i < n; i++) {
            V3 u = c.ia[i];
            E.push_back(c.pts[i]), U.push_back(u), N.push_back(unit(c.na[i] - u * dot(c.na[i], u)));
            outlines.push_back(pointsOf(per[i][k], chords, i == 0 ? &runOf : nullptr));
            radii.push_back(radiiOf(per[i][k], runOf));
          }
          // Sections that change their make along the run (a run more or fewer somewhere): no tool sweeps between them.
          for (const auto &o : outlines)
            if (o.size() != outlines[0].size()) {
              tooLarge();
              return false;
            }
          if (!c.closed) {
            for (int end = 0; end < 2; end++) {
              size_t i = end == 0 ? 0 : n - 1;
              V3 T = c.tangent(i) * (end == 0 ? -1.0 : 1.0), V = c.pts[i];
              std::vector<int> others;
              for (int f : facesAt(s, V))
                if (f != c.fa[i] && f != c.fb[i]) others.push_back(f);
              bool onto = others.size() == 1 && s.faces[others[0]].geom.flat && dot(s.faces[others[0]].geom.pn, T) > 0.05;
              double reach = onto ? (span + 0.45 * reachOut(span)) / dot(s.faces[others[0]].geom.pn, T) * 1.2 + span : 0;
              if (reach > 0) {
                V3 Ex = V + T * reach;
                if (end == 0) {
                  E.insert(E.begin(), Ex), U.insert(U.begin(), U.front()), N.insert(N.begin(), N.front());
                  outlines.insert(outlines.begin(), outlines.front()), radii.insert(radii.begin(), radii.front());
                } else {
                  E.push_back(Ex), U.push_back(U.back()), N.push_back(N.back()), outlines.push_back(outlines.back()), radii.push_back(radii.back());
                }
              }
            }
          }
          Solid tool = sweptTool(E, U, N, outlines, c.closed, runOf, &radii);
          if (per[0][k].fill && !c.closed)
            for (int end = 0; end < 2; end++) {
              size_t i = end == 0 ? 0 : n - 1;
              std::vector<int> others;
              for (int f : facesAt(s, c.pts[i]))
                if (f != c.fa[i] && f != c.fb[i]) others.push_back(f);
              if (others.size() == 1 && s.faces[others[0]].geom.flat) tool = cut(tool, c.pts[i], -s.faces[others[0]].geom.pn, 0);
            }
          if (per[0][k].fill)
            for (int f : fillCut[ci]) tool = cut(tool, s.faces[f].geom.pn * (s.faces[f].geom.pd - (fillHair ? hair(t.radius) : 0)), -s.faces[f].geom.pn, 0);
          (per[0][k].fill ? add : take).push_back(tool);
        }
      }
    }
    return true;
  };

  // Where three bevelled edges meet at a corner of three flat faces, the corner cut off flat too, through the points where
  // the bevels' lines meet on each face (as OpenCascade's kernel does).
  struct Corner {
    V3 mid, n, V;
    double reach;
  };
  std::vector<Corner> corners;
  if (t.kind == Treatment::Bevel && !(t.corner > 0.005)) {
    // Each crease's end there: which crease, and its point at that end and the one next to it (a run's end piece, straight
    // between flat faces, as a lone edge would be: a run going on into a seam at its other end still meets its corner).
    struct End {
      size_t i, k, next;
    };
    std::map<std::tuple<double, double, double>, std::vector<End>> at;
    for (size_t i = 0; i < work.size(); i++) {
      const Crease &c = work[i];
      size_t n = c.pts.size();
      if (c.closed || n < 2) continue;
      for (auto [k, next] : {std::pair<size_t, size_t>{0, 1}, {n - 1, n - 2}}) {
        if (c.angleAt(k) >= 179 || !s.faces[c.fa[k]].geom.flat || !s.faces[c.fb[k]].geom.flat) continue;
        if (c.edges.size() > 1 && s.edges[c.edges[k == 0 ? 0 : c.edges.size() - 1]].pts.size() != 2) continue;
        V3 v = c.pts[k];
        at[{v.x, v.y, v.z}].push_back({i, k, next});
      }
    }
    for (auto &[key, list] : at) {
      if (list.size() != 3) continue;
      V3 V{std::get<0>(key), std::get<1>(key), std::get<2>(key)};
      std::vector<int> faces = facesAt(s, V);
      if (faces.size() != 3) continue;
      std::vector<V3> P;
      for (int f : faces) {
        if (!s.faces[f].geom.flat) break;
        // The two bevelled edges of this face at V: each one's line on the face, its leg in from the edge.
        V3 n = s.faces[f].geom.pn, dir[2], into[2];
        double leg[2];
        int k = 0;
        for (const End &end : list) {
          const Crease &c = work[end.i];
          if (c.fa[end.k] != f && c.fb[end.k] != f) continue;
          if (k == 2) break;
          dir[k] = unit(c.pts[end.next] - V);
          leg[k] = c.fa[end.k] == f ? t.legA : t.legB;
          k++;
        }
        if (k != 2) break;
        for (int j = 0; j < 2; j++) {
          into[j] = unit(cross(n, dir[j]));
          if (dot(into[j], dir[1 - j]) < 0) into[j] = -into[j];
        }
        // V + dir0·a + into0·leg0 = V + dir1·b + into1·leg1, in the face's plane.
        V3 rhs = into[1] * leg[1] - into[0] * leg[0];
        double m00 = dot(dir[0], dir[0]), m01 = -dot(dir[0], dir[1]), m11 = dot(dir[1], dir[1]);
        double r0 = dot(dir[0], rhs), r1 = -dot(dir[1], rhs), det = m00 * m11 - m01 * m01;
        if (std::fabs(det) < 1e-12) break;
        double a = (r0 * m11 - m01 * r1) / det;
        P.push_back(V + dir[0] * a + into[0] * leg[0]);
      }
      if (P.size() != 3) continue;
      V3 mid = (P[0] + P[1] + P[2]) / 3.0, n = unit(cross(P[1] - P[0], P[2] - P[0]));
      if (dot(n, V - mid) < 0) n = -n;
      if (!(dot(n, V - mid) > tol)) continue;
      double reach = 0;
      for (V3 q : P) reach = std::max(reach, norm(q - V));
      corners.push_back({mid, n, V, reach * 1.5});
    }
  }

  // The tools taken away one by one, then those added; failing that, each kind all together.
  auto made = [&](bool together) {
    Solid r = base;
    for (int op : {BK_SUBTRACT, BK_UNION}) {
      const std::vector<Solid> &tools = op == BK_SUBTRACT ? take : add;
      if (tools.empty()) continue;
      if (!together) {
        for (size_t i = 0; i < tools.size(); i++) {
          Solid next = step(r, tools[i], op);
          auto parts = piecesOf.find({op == BK_UNION, i});
          if (!closed(next) && parts != piecesOf.end()) {
            next = r;
            for (const auto &part : parts->second)
              if (closed(next)) next = step(next, part, op);
          }
          r = std::move(next);
          if (!closed(r)) return r;
        }
        continue;
      }
      // A tool left open takes nothing away (or anything): no result, not the shape as it was (nor going on with it).
      Solid all = tools[0];
      for (size_t i = 1; i < tools.size() && closed(all); i++) all = step(all, tools[i], BK_UNION);
      if (!closed(all)) return Solid();
      r = step(r, all, op);
    }
    // The corners cut off, each only where all it cuts lies near its corner.
    for (const auto &k : corners) {
      bool near = true;
      for (V3 q : r.p)
        if (dot(q - k.mid, k.n) > tol && norm(q - k.V) > k.reach) near = false;
      if (near) r = cut(r, k.mid, -k.n, 0);
    }
    return r;
  };
  // Each tool's faces that lie wholly outside the solid where it takes away (inside it where it adds): there only to close
  // the tool (sampled at points within some of each face's triangles; a tool's triangles may run its whole length).
  auto markAux = [&](std::vector<Solid> &tools, bool fill) {
    static const double within[7][3] = {{1 / 3.0, 1 / 3.0, 1 / 3.0}, {0.45, 0.45, 0.1}, {0.45, 0.1, 0.45}, {0.1, 0.45, 0.45},
                                        {0.7, 0.15, 0.15}, {0.15, 0.7, 0.15}, {0.15, 0.15, 0.7}};
    for (auto &tool : tools) {
      std::vector<std::vector<size_t>> of(tool.faces.size());
      for (size_t t = 0; t < tool.triFace.size(); t++) of[tool.triFace[t]].push_back(t);
      for (size_t f = 0; f < tool.faces.size(); f++) {
        if (of[f].empty()) continue;
        bool aux = true;
        size_t step = std::max<size_t>(1, of[f].size() / 8);
        for (size_t i = 0; i < of[f].size() && aux; i += step) {
          size_t t = of[f][i];
          V3 a = tool.p[tool.tri[3 * t]], b = tool.p[tool.tri[3 * t + 1]], c = tool.p[tool.tri[3 * t + 2]];
          for (int k = 0; k < 7 && aux; k++) aux = inside(s, a * within[k][0] + b * within[k][1] + c * within[k][2]) == fill;
        }
        tool.faces[f].aux = aux;
      }
    }
  };
  // Inward roundings exact first; where their cylinders only touch at a corner, a little wider each there.
  Solid result;
  bool done = false;
  for (double shadeBy : {0.0, 1e-4, 2e-3}) {
    if (shadeBy > 0 && t.kind != Treatment::Cove) break;
    if (!tools(shadeBy)) return s;
    markAux(take, false), markAux(add, true);
    // A rounding's faces (its tools' curved ones that are left) meet the faces beside them smoothly: so marked for what's
    // done next.
    if (t.kind != Treatment::Bevel)
      for (auto *list : {&take, &add})
        for (auto &tool : *list)
          for (auto &f : tool.faces)
            if (!f.geom.flat && !f.aux) f.blend = true;
    result = made(false);
    if (!closed(result) && take.size() + add.size() > 1) result = made(true);
    if ((done = !result.tri.empty() && closed(result))) break;
  }
  if (!done && !fillCut.empty()) {
    fillHair = true;
    if (!tools(0)) return s;
    markAux(take, false), markAux(add, true);
    for (auto *list : {&take, &add})
      for (auto &tool : *list)
        for (auto &f : tool.faces)
          if (!f.geom.flat && !f.aux) f.blend = true;
    result = made(false);
    if (!closed(result) && take.size() + add.size() > 1) result = made(true);
    done = !result.tri.empty() && closed(result);
  }
  if (!done) return coveChecked ? tooWide() : tooLarge();
  // Wider than the faces beside it hold: fine unless one of them is gone, or a face that has no corner on a coved edge
  // is cut into (the shape's faces keep their numbers through the cuts).
  if (coveChecked) {
    std::vector<double> before(s.faces.size(), 0), after(s.faces.size(), 0);
    auto areas = [](const Solid &m, std::vector<double> &out) {
      for (size_t t = 0; t < m.triFace.size(); t++) {
        if (m.triFace[t] >= out.size()) continue;
        V3 a = m.p[m.tri[3 * t]], b = m.p[m.tri[3 * t + 1]], c = m.p[m.tri[3 * t + 2]];
        out[m.triFace[t]] += norm(cross(b - a, c - a)) / 2;
      }
    };
    areas(s, before), areas(result, after);
    std::set<int> beside, near;
    for (const auto &c : work) {
      for (size_t k = 0; k < c.pts.size(); k++) beside.insert(c.fa[k]), beside.insert(c.fb[k]);
      for (int e : c.edges)
        for (V3 end : {s.edges[e].pts.front(), s.edges[e].pts.back()})
          for (int f : facesAt(s, end)) near.insert(f);
    }
    for (size_t f = 0; f < s.faces.size(); f++) {
      bool touched = std::fabs(after[f] - before[f]) > 1e-9 * (1 + before[f]);
      // (A speck of a face far smaller than the mesh tells apart, where a cut grazed a corner, may go.)
      bool gone = after[f] <= 1e-9 * (1 + before[f]) && before[f] > 0.1 * d * d;
      if (beside.count((int)f) ? gone : touched && !near.count((int)f)) return tooWide();
    }
  }
  foldAux(result, d);
  finish(result, d);
  // A hair's remnant of a face left where two tools meet (a run's straight part and the part swept on from it): into the
  // face beside it.
  if (fold(result, 0.05 * d, false)) finish(result, d);
  int was = pieces(base);
  if (pieces(result) > was && add.empty() && dropIslands(result, was, 0.02)) finish(result, d);
  if (pieces(result) != was) return coveChecked ? tooWide() : tooLarge();
  return result;
}

Solid treated(const Solid &s, const Treatment &t, double d, TreatFit &fit, const Solid *onto) {
  Solid made = treatedAs(s, t, d, fit, onto, false);
  // Every edge asked for and not every one fitting: those beside a hair's remnant of a face (where roundings or cuts met)
  // left as they are, if then the rest fit.
  bool whole = !t.kinds.empty();
  for (int k : t.kinds) whole = whole && k == BK_PICK_BODY;
  if (fit.fits || !whole) return made;
  TreatFit spared;
  Solid other = treatedAs(s, t, d, spared, onto, true);
  if (!spared.fits) return made;
  fit = spared;
  return other;
}

bool foldThin(Solid &r, double width) { return fold(r, width, false); }

bool shut(const Solid &s) { return closed(s); }

}  // namespace bce
