// Bcad's geometry engine: hollowing. The shape less a void inside it, the void made from the shape's own tree: each
// primitive shrunk by the walls (its faces moved in: a flat-sided one's planes, a turned one's or a tube's outline), an
// opening's face moved out through the shape instead and a face with a wall of its own by that; the parts of a merge
// shrunk alike (a face hidden inside the merge moved out, into the next part's void), what's taken away grown, a cut's
// face moved in; roundings narrower by the walls, bevels moved in, inward roundings wider round the same edges.
#include "Engine/Model.hpp"
#include "Engine/Treat.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <cmath>
#include <map>

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
      return std::atan2(dot(pts[a] - centre, e2), dot(pts[a] - centre, e1)) < std::atan2(dot(pts[b] - centre, e2), dot(pts[b] - centre, e1));
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
bool outlineInset(const std::vector<Elem> &prof, const std::vector<double> &move, std::vector<Elem> &out) {
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
    if (joint[k].x < 0) return false;
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
    double a0 = std::atan2(from.y - m.cz, from.x - m.cr), a1 = std::atan2(to.y - m.cz, to.x - m.cr), turn = e.a1 - e.a0;
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
    double th = 2 * pi * j / n, c = std::cos(th), sn = std::sin(th);
    V3 nn = unit(p2(b * c, a * sn));
    ring[j] = p2(a * c, b * sn) - nn * side, nrm[j] = nn;
  }
  for (int j = 0; j < n; j++) polyArea += cross2(ring[j], ring[(j + 1) % n]) / 2;
  // The oval's length round, closely (its arc summed finely).
  for (int k = 0, m = 4096; k < m; k++) {
    double th = 2 * pi * (k + 0.5) / m;
    perimeter += std::hypot(a * std::sin(th), b * std::cos(th)) * 2 * pi / m;
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
      most = std::max(most, std::hypot(nr / std::max(across, 1e-12), nz / std::max(sz, 1e-12)));
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
  std::vector<Elem> moved;
  if (!outlineInset(prof, move, moved)) {
    // An opening moved out so far that a curved piece beside it no longer reaches it (a dome round an open base): only
    // just out instead, as far as needed to break through.
    bool opening = false;
    for (double &mv : move)
      if (mv < 0) mv = -0.02 * r.size, opening = true;
    if (!opening || !outlineInset(prof, move, moved)) return false;
  }
  std::shared_ptr<Model> model = m.kind == Model::Turned ? turnedModel(moved) : sweptModel(m.a, m.b, m.phi, moved);
  mesh(shapeOf(model, Wn), r.d, out);
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
  std::map<std::tuple<long, long, long>, std::vector<double>> groups;  // (radius or legs, in 1e-9) → edge picks
  for (const auto &c : creases) {
    if (c.pts.size() < 2) continue;
    size_t m = c.pts.size() / 2;
    double oA = moveOf(r, child.faces[c.fa[m]].geom, 1, flip, inBool), oB = moveOf(r, child.faces[c.fb[m]].geom, 1, flip, inBool);
    V3 na = c.na[m], nb = c.nb[m];
    double kk = dot(na, nb), det = 1 - kk * kk;
    if (std::fabs(det) < 1e-9) continue;
    double alpha = (-oA + oB * kk) / det, beta = (-oB + oA * kk) / det;
    // Picked at the edge's middle (an edge of two points: halfway along it, not at its end).
    V3 mid = c.pts.size() == 2 ? (c.pts[0] + c.pts[1]) / 2.0 : c.pts[m];
    V3 at = mid + na * alpha + nb * beta, dir = c.tangent(m);
    std::tuple<long, long, long> key;
    if (t.kind == Treatment::Round) {
      double rad = t.radius - std::max(oA, oB);
      if (rad < 0.005) continue;
      key = {std::lround(rad * 1e9), 0, 0};
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
      key = {std::lround(la * 1e9), std::lround(lb * 1e9), 0};
    }
    double pick[6] = {at.x, at.y, at.z, dir.x, dir.y, dir.z};
    auto &g = groups[key];
    g.insert(g.end(), pick, pick + 6);
  }
  for (const auto &[key, edgePicks] : groups) {
    Treatment there = t;
    there.picks = edgePicks;
    there.kinds.assign(edgePicks.size() / 6, BK_PICK_EDGE);
    if (t.kind == Treatment::Round) there.radius = std::get<0>(key) * 1e-9;
    else there.legA = std::get<0>(key) * 1e-9, there.legB = std::get<1>(key) * 1e-9, there.corner = std::max(0.0, t.corner - r.t);
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

}  // namespace

bool hollowed(const Shape &s, const Hollowing &h, double d, Solid &out, int *missing) {
  if (missing) *missing = 0;
  Solid whole;
  mesh(s, d, whole);
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
  for (size_t i = 0; i + 5 < h.open.size(); i += 6) {
    int f = faceAt(whole, &h.open[i]);
    if (f < 0) lost++;
    else rules.open.push_back(whole.faces[f].geom);
  }
  for (size_t i = 0; i + 5 < h.walls.size() && i / 6 < h.wallThickness.size(); i += 6) {
    int f = faceAt(whole, &h.walls[i]);
    if (f < 0) lost++;
    else rules.own.push_back({whole.faces[f].geom, std::max(h.wallThickness[i / 6], 0.01)});
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
    return v > 0 && v < all * 0.999 && pieces(out) == pieces(whole);
  }
  Ctx ctx{rules};
  Solid hole;
  if (!voidOf(s, Affine(), 1, false, false, ctx, hole)) return false;
  out = combine(whole, hole, BK_SUBTRACT);
  finish(out, d);
  double v = out.meshVolume(), all = whole.meshVolume(), taken = all - v, inside = hole.meshVolume();
  if (!(v > 0) || !(v < all * 0.999) || pieces(out) != pieces(whole)) return false;
  // Shut, the void must lie wholly inside (walls too thick at a rounding would break through).
  if (rules.open.empty() && taken < inside * (1 - 1e-6) - 1e-9 * rules.size * rules.size * rules.size) return false;
  return true;
}

}  // namespace bce
