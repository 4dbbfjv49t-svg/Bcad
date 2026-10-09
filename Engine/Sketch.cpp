// Bcad's geometry engine: a sketch's curves cut where they meet, the regions they bound, and solids made of chosen
// regions: stood up (extruded) or turned (revolved) exactly.
#include "Engine/Sketch.hpp"

#include "Engine/Triangulate.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <tuple>

namespace bce {

namespace {

constexpr double pi = M_PI;
constexpr int maxPoints = 4000, maxCurves = 4000, maxRules = 8000;
constexpr long maxCrossings = 200000;

double cross2(V3 a, V3 b) { return a.x * b.y - a.y * b.x; }
double dot2(V3 a, V3 b) { return a.x * b.x + a.y * b.y; }
double len2(V3 a) { return trig::hypot(a.x, a.y); }

// Chords for an arc of `span` and radius r within `d` of it (as a body's mesh: none turning past 0.35).
int chordsFor(double span, double r, double d) {
  double c = 1 - d / std::max(r, 1e-300), a = c <= -1 ? pi : 2 * trig::acos(std::clamp(c, -1.0, 1.0));
  a = std::min(std::max(a, 1e-9), 0.35);
  double n = std::ceil(std::fabs(span) / a - 1e-9);
  return n >= 100000 ? 100000 : std::max(1, (int)(n > 0 ? n : 1));
}

// The largest (x, y) · (dx, dy) over a piece.
double reach(const Elem &e, double dx, double dy) {
  double best = std::max(e.r0 * dx + e.z0 * dy, e.r1 * dx + e.z1 * dy);
  if (e.arc && (dx != 0 || dy != 0)) {
    double a = trig::atan2(dy, dx), lo = std::min(e.a0, e.a1), hi = std::max(e.a0, e.a1);
    while (a < lo) a += 2 * pi;
    while (a >= lo + 2 * pi) a -= 2 * pi;
    if (a <= hi) best = std::max(best, (e.cr + e.rad * trig::cos(a)) * dx + (e.cz + e.rad * trig::sin(a)) * dy);
  }
  return best;
}

// How far a piece turns round q (q off it): a straight piece by the angle it spans from q, an arc by its chord's angle
// and, where q lies between the chord and the arc, a whole turn its way (an arc whose chord q lies on, by its halves).
double turnRound(const Elem &e, V3 q, int depth = 0) {
  V3 s{e.r0, e.z0, 0}, t{e.r1, e.z1, 0}, a = s - q, b = t - q;
  double turn = trig::atan2(cross2(a, b), dot2(a, b));
  if (!e.arc) return turn;
  double sweep = e.a1 - e.a0;
  bool whole = std::fabs(sweep) >= 2 * pi - 1e-9;
  if (!whole && depth < 8 && std::fabs(cross2(t - s, q - s)) <= 1e-12 * len2(t - s) * len2(q - s) && dot2(a, b) <= 0) {
    double am = (e.a0 + e.a1) / 2;
    Elem first = e, second = e;
    first.r1 = second.r0 = e.cr + e.rad * trig::cos(am), first.z1 = second.z0 = e.cz + e.rad * trig::sin(am);
    first.a1 = second.a0 = am;
    return turnRound(first, q, depth + 1) + turnRound(second, q, depth + 1);
  }
  if (!(trig::hypot(q.x - e.cr, q.y - e.cz) < e.rad)) return turn;
  bool between = true;
  if (!whole) {
    double am = (e.a0 + e.a1) / 2;
    V3 m{e.cr + e.rad * trig::cos(am), e.cz + e.rad * trig::sin(am), 0};
    between = cross2(t - s, q - s) * cross2(t - s, m - s) > 0;
  }
  return between ? turn + (sweep > 0 ? 2 * pi : -2 * pi) : turn;
}

// How many times a closed loop of pieces winds round q (q off it).
double windingOf(const std::vector<Elem> &loop, V3 q) {
  double sum = 0;
  for (const Elem &e : loop) sum += turnRound(e, q);
  return sum / (2 * pi);
}

double loopArea(const std::vector<Elem> &loop) {
  double a = 0;
  for (const Elem &e : loop) a += pieceArea(e);
  return a;
}

// Points within eps of each other as one (the first made kept where it was), found by a grid of cells eps across.
struct Vertices {
  double eps = 0;
  std::vector<V3> at;
  std::map<std::pair<long long, long long>, std::vector<int>> grid;

  std::pair<long long, long long> cell(V3 p) const { return {(long long)std::floor(p.x / (2 * eps)), (long long)std::floor(p.y / (2 * eps))}; }
  int find(V3 p) const {
    auto c = cell(p);
    int best = -1;
    for (long long dx = -1; dx <= 1; dx++)
      for (long long dy = -1; dy <= 1; dy++) {
        auto it = grid.find({c.first + dx, c.second + dy});
        if (it == grid.end()) continue;
        for (int i : it->second)
          if (len2(at[i] - p) <= eps && (best < 0 || i < best)) best = i;
      }
    return best;
  }
  int add(V3 p) {
    int i = find(p);
    if (i >= 0) return i;
    i = (int)at.size();
    at.push_back(p);
    grid[cell(p)].push_back(i);
    return i;
  }
};

// A curve taking part: a segment a → b, an arc round c from angle a0 to a1 (a1 > a0, from a to b), or a whole circle
// (a and b its point at angle 0); where it's cut (its own parameter there: t along a segment, the angle round an arc).
struct Piece {
  int curve = -1, kind = 0;
  V3 a, b, c;
  double r = 0, a0 = 0, a1 = 0;
  double lo[2], hi[2];
  std::vector<std::pair<double, int>> cuts;
};

double paramOf(const Piece &p, V3 q) {
  if (p.kind == BK_CURVE_LINE) {
    V3 d = p.b - p.a;
    return std::clamp(dot2(q - p.a, d) / dot2(d, d), 0.0, 1.0);
  }
  double ang = trig::atan2(q.y - p.c.y, q.x - p.c.x);
  if (p.kind == BK_CURVE_CIRCLE) return ang < 0 ? ang + 2 * pi : ang;
  while (ang < p.a0) ang += 2 * pi;
  while (ang >= p.a0 + 2 * pi) ang -= 2 * pi;
  if (ang > p.a1) return ang - p.a1 <= p.a0 + 2 * pi - ang ? p.a1 : p.a0;
  return ang;
}

bool onPiece(const Piece &p, V3 q, double eps) {
  if (p.kind == BK_CURVE_LINE) {
    V3 d = p.b - p.a;
    double l = len2(d), off = std::fabs(cross2(d, q - p.a)) / l, along = dot2(q - p.a, d) / l;
    return off <= eps && along >= -eps && along <= l + eps;
  }
  if (std::fabs(len2(q - p.c) - p.r) > eps) return false;
  if (p.kind == BK_CURVE_CIRCLE) return true;
  double ang = trig::atan2(q.y - p.c.y, q.x - p.c.x), tol = eps / p.r;
  while (ang < p.a0) ang += 2 * pi;
  while (ang >= p.a0 + 2 * pi) ang -= 2 * pi;
  return ang <= p.a1 + tol || ang >= p.a0 + 2 * pi - tol;
}

// Where two pieces' lines or circles cross (to be checked against both pieces' ends): one point where they touch.
void crossings(const Piece &p, const Piece &q, double eps, std::vector<V3> &out) {
  bool pl = p.kind == BK_CURVE_LINE, ql = q.kind == BK_CURVE_LINE;
  if (pl && ql) {
    V3 d = p.b - p.a, e = q.b - q.a;
    double den = cross2(d, e);
    // (Parallel: where they overlap, each one's ends cut the other.)
    if (std::fabs(den) <= 1e-12 * len2(d) * len2(e)) return;
    out.push_back(p.a + d * (cross2(q.a - p.a, e) / den));
  } else if (pl || ql) {
    const Piece &l = pl ? p : q, &c = pl ? q : p;
    V3 d = l.b - l.a, u = d / len2(d);
    V3 foot = l.a + u * dot2(c.c - l.a, u);
    double h = cross2(u, c.c - l.a), h2 = c.r * c.r - h * h;
    if (h2 < -2 * c.r * eps) return;
    if (h2 <= 2 * c.r * eps) {
      out.push_back(foot);
      return;
    }
    double w = std::sqrt(h2);
    out.push_back(foot - u * w), out.push_back(foot + u * w);
  } else {
    V3 d = q.c - p.c;
    double D = len2(d);
    if (D <= eps) return;  // (on one centre: where they overlap, each one's ends cut the other)
    double a = (p.r * p.r - q.r * q.r + D * D) / (2 * D), h2 = p.r * p.r - a * a, big = std::max(p.r, q.r);
    if (h2 < -2 * big * eps) return;
    V3 u = d / D, n{-u.y, u.x, 0}, base = p.c + u * a;
    if (h2 <= 2 * big * eps) {
      out.push_back(base);
      return;
    }
    double h = std::sqrt(h2);
    out.push_back(base - n * h), out.push_back(base + n * h);
  }
}

// The curves cut where they meet into edges between vertices, loose ends taken off, every edge's two half-edges (2e from
// v0 to v1, 2e + 1 back) in order round each vertex, the cycles they make with each face on its left, and the regions:
// each a cycle running counter-clockwise with the cycles round the parts inside it as its holes.
struct Arrangement {
  double eps = 0, size = 0;
  std::vector<V3> verts;
  struct Edge {
    int v0 = 0, v1 = 0;
    bool arc = false;
    V3 c;
    double r = 0, a0 = 0, a1 = 0;
    std::vector<std::pair<int, bool>> curves;  // the curves it lies on, and whether each runs v0 → v1 too
  };
  std::vector<Edge> edges;
  std::vector<int> next, cycleOf;
  std::vector<std::vector<int>> out;  // per vertex, half-edges leaving it counter-clockwise
  std::vector<int> posOf;             // each half-edge's place in its vertex's list
  std::vector<std::vector<int>> cycles;
  std::vector<double> cycleArea;
  struct Face {
    int outer;
    std::vector<int> holes;
    double area;
  };
  std::vector<Face> faces;

  int origin(int h) const { return h % 2 ? edges[h / 2].v1 : edges[h / 2].v0; }
  int dest(int h) const { return h % 2 ? edges[h / 2].v0 : edges[h / 2].v1; }
  Elem elemOf(int h) const {
    const Edge &e = edges[h / 2];
    bool fwd = h % 2 == 0;
    V3 p = verts[fwd ? e.v0 : e.v1], q = verts[fwd ? e.v1 : e.v0];
    if (!e.arc) return Elem::line(p.x, p.y, q.x, q.y);
    Elem x = Elem::line(p.x, p.y, q.x, q.y);
    x.arc = true, x.cr = e.c.x, x.cz = e.c.y, x.rad = e.r, x.a0 = fwd ? e.a0 : e.a1, x.a1 = fwd ? e.a1 : e.a0;
    return x;
  }
  std::vector<Elem> loopOf(int cycle) const {
    std::vector<Elem> l;
    for (int h : cycles[cycle]) l.push_back(elemOf(h));
    return l;
  }
  // Whether q lies inside face f (off its sides).
  bool holds(int f, V3 q) const {
    double w = windingOf(loopOf(faces[f].outer), q);
    for (int c : faces[f].holes) w += windingOf(loopOf(c), q);
    return std::lround(w) != 0;
  }
  // A face's sides, as bk_sketch_regions gives them.
  std::vector<int> sidesOf(int f) const {
    std::vector<int> s;
    std::vector<int> cs{faces[f].outer};
    cs.insert(cs.end(), faces[f].holes.begin(), faces[f].holes.end());
    for (int c : cs)
      for (int h : cycles[c])
        for (auto [curve, same] : edges[h / 2].curves) s.push_back(2 * curve + ((h % 2 == 0) == same ? 0 : 1));
    std::sort(s.begin(), s.end());
    s.erase(std::unique(s.begin(), s.end()), s.end());
    return s;
  }
};

bool arrange(const Sketch &s, Arrangement &A, std::string &why) {
  const auto &P = s.points;
  // The size the sketch reaches to: what counts as one point.
  double reachOut = 0;
  for (const auto &c : s.curves) {
    if (c.flags & BK_CURVE_CONSTRUCTION) continue;
    for (int k = 0; k < (c.kind == BK_CURVE_LINE ? 2 : c.kind == BK_CURVE_ARC ? 3 : 1); k++)
      reachOut = std::max({reachOut, std::fabs(P[c.p[k]].x), std::fabs(P[c.p[k]].y)});
    if (c.kind == BK_CURVE_CIRCLE) reachOut = std::max({reachOut, std::fabs(P[c.p[0]].x) + c.radius, std::fabs(P[c.p[0]].y) + c.radius});
  }
  double eps = 1e-9 * (1 + reachOut);
  A.eps = eps, A.size = reachOut;
  Vertices V;
  V.eps = eps;
  std::vector<Piece> pieces;
  for (int k = 0; k < (int)s.curves.size(); k++) {
    const SketchCurve &c = s.curves[k];
    if (c.flags & BK_CURVE_CONSTRUCTION) continue;
    Piece p;
    p.curve = k, p.kind = c.kind;
    if (c.kind == BK_CURVE_LINE) {
      p.a = P[c.p[0]], p.b = P[c.p[1]];
      if (len2(p.b - p.a) <= eps) continue;
      p.lo[0] = std::min(p.a.x, p.b.x), p.lo[1] = std::min(p.a.y, p.b.y), p.hi[0] = std::max(p.a.x, p.b.x), p.hi[1] = std::max(p.a.y, p.b.y);
    } else {
      p.c = P[c.p[0]];
      if (c.kind == BK_CURVE_ARC) {
        // (Its end where its circle meets the way to the end point given: on it, as the rules keep it.)
        p.a = P[c.p[1]], p.b = P[c.p[2]], p.r = len2(p.a - p.c);
        if (p.r <= eps || len2(p.b - p.c) <= eps) continue;
        if (std::fabs(len2(p.b - p.c) - p.r) > eps) p.b = p.c + (p.b - p.c) * (p.r / len2(p.b - p.c));
        if (len2(p.b - p.a) <= eps) continue;
        p.a0 = trig::atan2(p.a.y - p.c.y, p.a.x - p.c.x), p.a1 = trig::atan2(p.b.y - p.c.y, p.b.x - p.c.x);
        while (p.a1 <= p.a0) p.a1 += 2 * pi;
      } else {
        p.r = c.radius;
        if (!(p.r > eps)) continue;
        p.a = p.b = p.c + V3{p.r, 0, 0}, p.a0 = 0, p.a1 = 2 * pi;
      }
      p.lo[0] = p.c.x - p.r, p.lo[1] = p.c.y - p.r, p.hi[0] = p.c.x + p.r, p.hi[1] = p.c.y + p.r;
    }
    pieces.push_back(p);
  }
  // The ends first (a sketch's own points where they are), then where curves cross.
  for (Piece &p : pieces)
    if (p.kind != BK_CURVE_CIRCLE) {
      p.cuts.push_back({p.kind == BK_CURVE_LINE ? 0.0 : p.a0, V.add(p.a)});
      p.cuts.push_back({p.kind == BK_CURVE_LINE ? 1.0 : p.a1, V.add(p.b)});
    }
  std::vector<int> order(pieces.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int i, int j) { return pieces[i].lo[0] != pieces[j].lo[0] ? pieces[i].lo[0] < pieces[j].lo[0] : i < j; });
  std::vector<std::pair<int, int>> pairs;
  for (size_t x = 0; x < order.size(); x++)
    for (size_t y = x + 1; y < order.size(); y++) {
      const Piece &p = pieces[order[x]], &q = pieces[order[y]];
      if (q.lo[0] > p.hi[0] + eps) break;
      if (q.lo[1] > p.hi[1] + eps || p.lo[1] > q.hi[1] + eps) continue;
      pairs.push_back({std::min(order[x], order[y]), std::max(order[x], order[y])});
    }
  std::sort(pairs.begin(), pairs.end());
  long crossed = 0;
  std::vector<V3> found;
  for (auto [i, j] : pairs) {
    Piece &p = pieces[i], &q = pieces[j];
    // Each one's ends on the other (where one ends on the other, or they overlap).
    for (int pass = 0; pass < 2; pass++) {
      Piece &from = pass ? q : p, &on = pass ? p : q;
      if (from.kind == BK_CURVE_CIRCLE) continue;
      for (int k = 0; k < 2; k++) {
        int v = from.cuts[k].second;
        if (onPiece(on, V.at[v], eps)) on.cuts.push_back({paramOf(on, V.at[v]), v});
      }
    }
    found.clear();
    crossings(p, q, eps, found);
    for (V3 x : found) {
      if (!onPiece(p, x, eps) || !onPiece(q, x, eps)) continue;
      if (++crossed > maxCrossings) return why = "sketch: too many crossings", false;
      int v = V.add(x);
      p.cuts.push_back({paramOf(p, V.at[v]), v});
      q.cuts.push_back({paramOf(q, V.at[v]), v});
    }
  }
  // A whole circle nothing cuts: a point of its own.
  for (Piece &p : pieces)
    if (p.kind == BK_CURVE_CIRCLE && p.cuts.empty()) p.cuts.push_back({0.0, V.add(p.a)});
  A.verts = V.at;

  // Edges between each piece's cuts in order; one edge where two pieces run along each other.
  std::map<std::tuple<int, int, int>, std::vector<int>> byEnds;
  auto addEdge = [&](Arrangement::Edge e, int curve) {
    auto key = std::make_tuple(std::min(e.v0, e.v1), std::max(e.v0, e.v1), (int)e.arc);
    auto &list = byEnds[key];
    for (int k : list) {
      Arrangement::Edge &o = A.edges[k];
      if (!e.arc) {
        o.curves.push_back({curve, o.v0 == e.v0});
        return;
      }
      double am = (e.a0 + e.a1) / 2, om = (o.a0 + o.a1) / 2;
      V3 m = e.c + V3{trig::cos(am), trig::sin(am), 0} * e.r, n = o.c + V3{trig::cos(om), trig::sin(om), 0} * o.r;
      if (len2(e.c - o.c) <= eps && std::fabs(e.r - o.r) <= eps && len2(m - n) <= eps && o.v0 == e.v0) {
        o.curves.push_back({curve, true});
        return;
      }
    }
    e.curves = {{curve, true}};
    list.push_back((int)A.edges.size());
    A.edges.push_back(e);
  };
  for (Piece &p : pieces) {
    std::sort(p.cuts.begin(), p.cuts.end());
    std::vector<std::pair<double, int>> cuts;
    for (auto c : p.cuts)
      if (cuts.empty() || cuts.back().second != c.second) cuts.push_back(c);
    if (p.kind == BK_CURVE_CIRCLE) {
      while (cuts.size() > 1 && cuts.back().second == cuts.front().second) cuts.pop_back();
      size_t n = cuts.size();
      for (size_t k = 0; k < n; k++) {
        Arrangement::Edge e;
        e.v0 = cuts[k].second, e.v1 = cuts[(k + 1) % n].second, e.arc = true, e.c = p.c, e.r = p.r;
        e.a0 = cuts[k].first, e.a1 = k + 1 < n ? cuts[k + 1].first : cuts[0].first + 2 * pi;
        addEdge(e, p.curve);
      }
      continue;
    }
    for (size_t k = 0; k + 1 < cuts.size(); k++) {
      Arrangement::Edge e;
      e.v0 = cuts[k].second, e.v1 = cuts[k + 1].second;
      if (p.kind == BK_CURVE_ARC) e.arc = true, e.c = p.c, e.r = p.r, e.a0 = cuts[k].first, e.a1 = cuts[k + 1].first;
      addEdge(e, p.curve);
    }
  }
  // Loose ends off: an edge with an end nothing else meets bounds nothing.
  std::vector<int> degree(A.verts.size(), 0);
  std::vector<char> alive(A.edges.size(), 1);
  std::vector<std::vector<int>> at(A.verts.size());
  for (int k = 0; k < (int)A.edges.size(); k++) {
    degree[A.edges[k].v0]++, degree[A.edges[k].v1]++;
    at[A.edges[k].v0].push_back(k), at[A.edges[k].v1].push_back(k);
  }
  std::vector<int> queue;
  for (int v = 0; v < (int)A.verts.size(); v++)
    if (degree[v] == 1) queue.push_back(v);
  for (size_t q = 0; q < queue.size(); q++) {
    int v = queue[q];
    if (degree[v] != 1) continue;
    for (int k : at[v]) {
      if (!alive[k]) continue;
      alive[k] = 0;
      int w = A.edges[k].v0 == v ? A.edges[k].v1 : A.edges[k].v0;
      degree[v]--, degree[w]--;
      if (degree[w] == 1) queue.push_back(w);
    }
  }
  std::vector<Arrangement::Edge> kept;
  for (size_t k = 0; k < A.edges.size(); k++)
    if (alive[k]) kept.push_back(A.edges[k]);
  A.edges = std::move(kept);

  // Round each vertex counter-clockwise: by the way each half-edge leaves (a hair apart counted the same), then by how it
  // bends (right first, then straight, then left), then by number.
  int nh = 2 * (int)A.edges.size();
  std::vector<double> angle(nh), bend(nh);
  for (int h = 0; h < nh; h++) {
    const auto &e = A.edges[h / 2];
    bool fwd = h % 2 == 0;
    double a;
    if (!e.arc) {
      V3 d = A.verts[fwd ? e.v1 : e.v0] - A.verts[fwd ? e.v0 : e.v1];
      a = trig::atan2(d.y, d.x), bend[h] = 0;
    } else {
      a = fwd ? e.a0 + pi / 2 : e.a1 - pi / 2, bend[h] = fwd ? 1 / e.r : -1 / e.r;
    }
    a = std::remainder(a, 2 * pi);
    if (a < -1e-9) a += 2 * pi;
    if (a >= 2 * pi - 1e-9) a -= 2 * pi;
    angle[h] = a;
  }
  A.out.assign(A.verts.size(), {});
  for (int h = 0; h < nh; h++) A.out[A.origin(h)].push_back(h);
  A.posOf.assign(nh, 0);
  for (auto &list : A.out) {
    std::sort(list.begin(), list.end(), [&](int x, int y) { return angle[x] != angle[y] ? angle[x] < angle[y] : x < y; });
    // Runs of directions within a hair of each other, ordered by how they bend.
    for (size_t i = 0; i < list.size();) {
      size_t j = i + 1;
      while (j < list.size() && angle[list[j]] - angle[list[j - 1]] <= 1e-9) j++;
      std::sort(list.begin() + i, list.begin() + j, [&](int x, int y) { return bend[x] != bend[y] ? bend[x] < bend[y] : x < y; });
      i = j;
    }
    for (size_t i = 0; i < list.size(); i++) A.posOf[list[i]] = (int)i;
  }
  // Each half-edge's next: round its far end, the one just clockwise of the way back.
  A.next.assign(nh, -1);
  for (int h = 0; h < nh; h++) {
    const auto &list = A.out[A.dest(h)];
    int p = A.posOf[h ^ 1];
    A.next[h] = list[(p + list.size() - 1) % list.size()];
  }
  A.cycleOf.assign(nh, -1);
  for (int h = 0; h < nh; h++) {
    if (A.cycleOf[h] >= 0) continue;
    int c = (int)A.cycles.size();
    A.cycles.push_back({});
    for (int g = h; A.cycleOf[g] < 0; g = A.next[g]) {
      A.cycleOf[g] = c;
      A.cycles[c].push_back(g);
    }
    A.cycleArea.push_back(loopArea(A.loopOf(c)));
  }
  // Pieces that touch are one part; a part's outside cycle is a hole in the smallest face of another part round it.
  std::vector<int> part(A.verts.size());
  std::iota(part.begin(), part.end(), 0);
  std::function<int(int)> find = [&](int x) { return part[x] == x ? x : part[x] = find(part[x]); };
  for (const auto &e : A.edges) {
    int a = find(e.v0), b = find(e.v1);
    if (a != b) part[std::max(a, b)] = std::min(a, b);
  }
  std::vector<int> faceOf(A.cycles.size(), -1);
  for (int c = 0; c < (int)A.cycles.size(); c++)
    if (A.cycleArea[c] > 0) faceOf[c] = (int)A.faces.size(), A.faces.push_back({c, {}, A.cycleArea[c]});
  for (int c = 0; c < (int)A.cycles.size(); c++) {
    if (A.cycleArea[c] > 0) continue;
    V3 q = A.verts[A.origin(A.cycles[c][0])];
    int mine = find(A.origin(A.cycles[c][0])), best = -1;
    for (int f = 0; f < (int)A.faces.size(); f++) {
      int o = A.faces[f].outer;
      if (find(A.origin(A.cycles[o][0])) == mine) continue;
      if (best >= 0 && !(A.cycleArea[o] < A.cycleArea[A.faces[best].outer])) continue;
      if (std::lround(windingOf(A.loopOf(o), q)) != 0) best = f;
    }
    if (best >= 0) A.faces[best].holes.push_back(c), A.faces[best].area += A.cycleArea[c];
  }
  return true;
}

// Each loop's pieces joined where one runs straight on into the next (one line on, or one circle).
void simplify(std::vector<Elem> &loop, double eps) {
  auto joins = [&](const Elem &a, const Elem &b) {
    if (a.arc != b.arc) return false;
    if (!a.arc) {
      V3 d{a.r1 - a.r0, a.z1 - a.z0, 0}, e{b.r1 - b.r0, b.z1 - b.z0, 0}, end{b.r1 - a.r0, b.z1 - a.z0, 0};
      return dot2(d, e) > 0 && std::fabs(cross2(d, end)) <= eps * len2(d);
    }
    return trig::hypot(a.cr - b.cr, a.cz - b.cz) <= eps && std::fabs(a.rad - b.rad) <= eps && (a.a1 - a.a0) * (b.a1 - b.a0) > 0;
  };
  bool merged = true;
  while (merged && loop.size() > 1) {
    merged = false;
    for (size_t k = 0; k < loop.size() && loop.size() > 1; k++) {
      size_t n = (k + 1) % loop.size();
      if (!joins(loop[k], loop[n])) continue;
      // (Two straight pieces left: no loop to make of one.)
      if (loop.size() == 2 && !loop[k].arc) break;
      Elem e = loop[k];
      e.r1 = loop[n].r1, e.z1 = loop[n].z1;
      if (e.arc) e.a1 = loop[k].a1 + (loop[n].a1 - loop[n].a0);
      loop[k] = e;
      loop.erase(loop.begin() + n);
      merged = true;
      break;
    }
  }
}

// The loops round the chosen faces (sides between two chosen ones dissolved; a loop through a point twice parted there),
// simplified, each outline followed by its holes, and the region each loop bounds.
bool chosenLoops(const Arrangement &A, const std::vector<int> &chosen, std::vector<std::vector<Elem>> &loops, std::vector<int> &region, std::string &why) {
  int nh = (int)A.next.size();
  std::vector<char> in(nh, 0);
  for (int f : chosen) {
    for (int h : A.cycles[A.faces[f].outer]) in[h] = 1;
    for (int c : A.faces[f].holes)
      for (int h : A.cycles[c]) in[h] = 1;
  }
  std::vector<char> left(nh, 0);
  for (int h = 0; h < nh; h++) left[h] = in[h] && !in[h ^ 1];
  std::vector<char> used(nh, 0);
  std::vector<std::vector<Elem>> traced;
  for (int h = 0; h < nh; h++) {
    if (!left[h] || used[h]) continue;
    std::vector<Elem> loop;
    int g = h, guard = 0;
    while (!used[g]) {
      used[g] = 1;
      loop.push_back(A.elemOf(g));
      // Round the far end, clockwise from the way back, to the first side still left.
      const auto &list = A.out[A.dest(g)];
      int p = A.posOf[g ^ 1], n = (int)list.size(), nx = -1;
      for (int k = 1; k <= n; k++) {
        int c = list[(p - k + 2 * n) % n];
        if (left[c]) {
          nx = c;
          break;
        }
      }
      if (nx < 0 || ++guard > nh) return why = "sketch: the regions can't be traced", false;
      g = nx;
    }
    if (g != h) return why = "sketch: the regions can't be traced", false;
    simplify(loop, A.eps);
    traced.push_back(std::move(loop));
  }
  // Outlines (counter-clockwise), and each hole in the smallest outline round it.
  std::vector<int> outlines;
  std::vector<double> area(traced.size());
  for (size_t k = 0; k < traced.size(); k++)
    if ((area[k] = loopArea(traced[k])) > 0) outlines.push_back((int)k);
  loops.clear(), region.clear();
  std::vector<std::vector<int>> holes(outlines.size());
  for (size_t k = 0; k < traced.size(); k++) {
    if (area[k] > 0) continue;
    double x, y;
    traced[k][0].point(0.5, x, y);
    int best = -1;
    for (size_t o = 0; o < outlines.size(); o++) {
      if (best >= 0 && !(area[outlines[o]] < area[outlines[best]])) continue;
      if (std::lround(windingOf(traced[outlines[o]], {x, y, 0})) != 0) best = (int)o;
    }
    if (best < 0) return why = "sketch: the regions can't be traced", false;
    holes[best].push_back((int)k);
  }
  for (size_t o = 0; o < outlines.size(); o++) {
    loops.push_back(traced[outlines[o]]), region.push_back((int)o);
    for (int k : holes[o]) loops.push_back(traced[k]), region.push_back((int)o);
  }
  if (loops.empty()) return why = "sketch: no region chosen", false;
  return true;
}

// A loop's points with each arc in chords within `d` of it (at least `least` of them, as fewestChords says).
std::vector<V3> polyline(const std::vector<Elem> &loop, double d, const std::vector<int> &least) {
  std::vector<V3> out;
  for (size_t k = 0; k < loop.size(); k++) {
    const Elem &e = loop[k];
    int n = e.arc ? std::max(chordsFor(e.a1 - e.a0, e.rad, d), least[k]) : 1;
    for (int j = 0; j < n; j++) {
      double x, y;
      e.point((double)j / n, x, y);
      out.push_back({x, y, 0});
    }
  }
  return out;
}

// Triangles covering loops (an outline and its holes), by a triangulation keeping their sides; empty where it can't.
std::vector<V3> cover(const std::vector<std::vector<V3>> &loops) {
  double lo[2] = {INFINITY, INFINITY}, hi[2] = {-INFINITY, -INFINITY};
  for (const auto &l : loops)
    for (V3 q : l) lo[0] = std::min(lo[0], q.x), lo[1] = std::min(lo[1], q.y), hi[0] = std::max(hi[0], q.x), hi[1] = std::max(hi[1], q.y);
  double w = std::max(hi[0] - lo[0], hi[1] - lo[1]) + 1, cx = (lo[0] + hi[0]) / 2, cy = (lo[1] + hi[1]) / 2;
  Tri2 t(cx - 4 * w, cy - 3 * w, cx + 4 * w, cy - 3 * w, cx, cy + 5 * w);
  std::vector<std::vector<int>> ids;
  for (const auto &l : loops) {
    ids.push_back({});
    for (V3 q : l) ids.back().push_back(t.insert(q.x, q.y));
  }
  for (const auto &id : ids)
    for (size_t j = 0; j < id.size(); j++)
      if (id[j] != id[(j + 1) % id.size()] && !t.keep(id[j], id[(j + 1) % id.size()])) return {};
  if (!t.made().empty()) return {};
  std::vector<V3> out;
  for (int q : t.insideKept()) out.push_back({t.px(q), t.py(q), 0});
  return out;
}

// A point well inside face f: the middle of its largest triangle (else a hair in from the middle of its first side).
V3 seedOf(const Arrangement &A, int f, const std::vector<V3> &tris) {
  std::vector<std::pair<double, int>> big;
  for (size_t t = 0; t + 2 < tris.size(); t += 3) big.push_back({-std::fabs(cross2(tris[t + 1] - tris[t], tris[t + 2] - tris[t])), (int)t});
  std::sort(big.begin(), big.end());
  for (size_t k = 0; k < big.size() && k < 16; k++) {
    int t = big[k].second;
    V3 c = (tris[t] + tris[t + 1] + tris[t + 2]) / 3;
    if (A.holds(f, c)) return c;
  }
  Elem e = A.elemOf(A.cycles[A.faces[f].outer][0]);
  double x, y;
  e.point(0.5, x, y);
  V3 way{e.r1 - e.r0, e.z1 - e.z0, 0};
  if (e.arc) {
    double am = (e.a0 + e.a1) / 2, s = e.a1 > e.a0 ? 1 : -1;
    way = {-s * trig::sin(am), s * trig::cos(am), 0};
  }
  V3 in = way / std::max(len2(way), 1e-300);
  in = {-in.y, in.x, 0};
  for (double step = 1e-3 * (1 + A.size); step > A.eps; step /= 4) {
    V3 q = V3{x, y, 0} + in * step;
    if (A.holds(f, q)) return q;
  }
  return {x, y, 0};
}

// The faces regions chosen earlier are now: the one with the same sides (of several, the one holding the seed), else the
// one holding the seed, else -1.
void matchIn(const Arrangement &A, const std::vector<RegionRef> &refs, std::vector<int> &out) {
  out.assign(refs.size(), -1);
  std::vector<std::vector<int>> sides(A.faces.size());
  for (size_t f = 0; f < A.faces.size(); f++) sides[f] = A.sidesOf((int)f);
  for (size_t k = 0; k < refs.size(); k++) {
    std::vector<int> same;
    for (size_t f = 0; f < A.faces.size(); f++)
      if (sides[f] == refs[k].sides) same.push_back((int)f);
    if (same.size() == 1) {
      out[k] = same[0];
      continue;
    }
    int pick = -1;
    for (int f : same)
      if (A.holds(f, refs[k].seed)) {
        pick = f;
        break;
      }
    for (int f = 0; f < (int)A.faces.size() && pick < 0; f++)
      if (A.holds(f, refs[k].seed)) pick = f;
    if (pick < 0 && !same.empty()) pick = same[0];
    out[k] = pick;
  }
}

}  // namespace

bool sketchValid(const Sketch &s, std::string &why) {
  int np = (int)s.points.size();
  if (np > maxPoints || (int)s.curves.size() > maxCurves || (int)s.rules.size() > maxRules) return why = "sketch: too large", false;
  if (!s.fixed.empty() && (int)s.fixed.size() != np) return why = "sketch: a point fixed or not for each point", false;
  for (V3 p : s.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return why = "sketch: sizes must be numbers", false;
    if (std::fabs(p.x) > 1e5 || std::fabs(p.y) > 1e5) return why = "sketch: a sketch is at most 100 m across", false;
  }
  for (const auto &c : s.curves) {
    int need = c.kind == BK_CURVE_LINE ? 2 : c.kind == BK_CURVE_ARC ? 3 : c.kind == BK_CURVE_CIRCLE ? 1 : -1;
    if (need < 0) return why = "sketch: unknown curve", false;
    for (int k = 0; k < need; k++)
      if (c.p[k] < 0 || c.p[k] >= np) return why = "sketch: a curve's point isn't one of the sketch's", false;
    if (c.kind == BK_CURVE_CIRCLE && !(c.radius >= 0 && c.radius <= 1e5)) return why = "sketch: a circle's radius must be a size", false;
  }
  for (const auto &r : s.rules) {
    if (r.kind < BK_RULE_COINCIDENT || r.kind > BK_DIM_ANGLE) return why = "sketch: unknown rule", false;
    for (int k = 0; k < 3; k++)
      if (r.p[k] < -1 || r.p[k] >= np) return why = "sketch: a rule's point isn't one of the sketch's", false;
    for (int k = 0; k < 2; k++)
      if (r.c[k] < -1 || r.c[k] >= (int)s.curves.size()) return why = "sketch: a rule's curve isn't one of the sketch's", false;
    if (!std::isfinite(r.value) || std::fabs(r.value) > 1e5) return why = "sketch: sizes must be numbers", false;
  }
  return true;
}

bool sketchRegions(const Sketch &s, double deflection, std::vector<SketchRegion> &out, std::string &why) {
  out.clear();
  if (!sketchValid(s, why)) return false;
  Arrangement A;
  if (!arrange(s, A, why)) return false;
  double d = std::isfinite(deflection) && deflection > 0 ? std::max(deflection, 1e-4 * (1 + A.size) * 1e-3) : 0.05;
  for (int f = 0; f < (int)A.faces.size(); f++) {
    SketchRegion r;
    r.area = A.faces[f].area;
    r.sides = A.sidesOf(f);
    // Its loops with any line it lies on both sides of (one joining its outline to a hole) left out.
    std::vector<std::vector<Elem>> loops;
    std::vector<int> region;
    if (!chosenLoops(A, {f}, loops, region, why)) return false;
    // (Arcs' chords of loops a hair apart may cross: finer ones then.)
    auto least = fewestChords(loops);
    for (int k = 0; k < 4 && r.triangles.empty(); k++) {
      r.loops.clear();
      for (size_t l = 0; l < loops.size(); l++) r.loops.push_back(polyline(loops[l], d / (1 << (2 * k)), least[l]));
      r.triangles = cover(r.loops);
    }
    r.seed = seedOf(A, f, r.triangles);
    out.push_back(std::move(r));
  }
  return true;
}

bool sketchMatch(const Sketch &s, const std::vector<RegionRef> &refs, std::vector<int> &out, std::string &why) {
  out.assign(refs.size(), -1);
  if (!sketchValid(s, why)) return false;
  Arrangement A;
  if (!arrange(s, A, why)) return false;
  matchIn(A, refs, out);
  return true;
}

bool sketchShape(const Sketch &s, const std::vector<RegionRef> &refs, const SketchForm &form, Shape &out, std::string &why) {
  if (!sketchValid(s, why)) return false;
  if (refs.empty()) return why = "sketch: no region chosen", false;
  if (!std::isfinite(form.low) || !std::isfinite(form.high)) return why = "sketch: sizes must be numbers", false;
  if (form.kind == BK_FORM_EXTRUDE) {
    if (!(form.high - form.low >= 0.001)) return why = "sketch: the extrusion has no depth", false;
    if (std::fabs(form.low) > 1e5 || std::fabs(form.high) > 1e5) return why = "sketch: a shape is at most 100 m across", false;
  } else if (form.kind == BK_FORM_REVOLVE) {
    if (!(form.high - form.low > 0 && form.high - form.low <= 360 + 1e-9) || std::fabs(form.low) > 720 || std::fabs(form.high) > 720)
      return why = "sketch: an angle must be above 0 and at most 360 degrees", false;
    if (form.axis < 0 || form.axis >= (int)s.curves.size() || s.curves[form.axis].kind != BK_CURVE_LINE) return why = "sketch: the axis isn't a line", false;
  } else {
    return why = "sketch: unknown form", false;
  }
  Arrangement A;
  if (!arrange(s, A, why)) return false;
  std::vector<int> chosen;
  matchIn(A, refs, chosen);
  for (int f : chosen)
    if (f < 0) return why = "sketch: a chosen region is gone", false;
  std::sort(chosen.begin(), chosen.end());
  chosen.erase(std::unique(chosen.begin(), chosen.end()), chosen.end());
  std::vector<std::vector<Elem>> loops;
  std::vector<int> region;
  if (!chosenLoops(A, chosen, loops, region, why)) return false;
  if (form.kind == BK_FORM_EXTRUDE) {
    out = shapeOf(extrudedModel(std::move(loops), std::move(region), form.low, form.high));
    return true;
  }
  // Turned: each point as (r, z) — r its distance from the axis line on the side the profile lies, z along the line.
  const SketchCurve &axis = s.curves[form.axis];
  V3 a = s.points[axis.p[0]], b = s.points[axis.p[1]];
  double l = len2(b - a);
  if (!(l > A.eps)) return why = "sketch: the axis isn't a line", false;
  V3 D = (b - a) / l, N{-D.y, D.x, 0};
  double most = -INFINITY, least = INFINITY;
  for (const auto &loop : loops)
    for (const Elem &e : loop) {
      Elem moved = e;
      moved.r0 -= a.x, moved.z0 -= a.y, moved.r1 -= a.x, moved.z1 -= a.y, moved.cr -= a.x, moved.cz -= a.y;
      most = std::max(most, reach(moved, N.x, N.y)), least = std::min(least, -reach(moved, -N.x, -N.y));
    }
  if (most <= A.eps) N = -N, std::swap(most, least), most = -most, least = -least;
  if (least < -A.eps) return why = "sketch: the profile crosses the axis", false;
  // (N, D) turns the plane over when N is on the left of the line: the loops then run the other way, put back after.
  double turnsOver = N.x * D.y - N.y * D.x;
  auto rz = [&](double x, double y, double &r, double &z) {
    V3 q{x - a.x, y - a.y, 0};
    r = dot2(q, N), z = dot2(q, D);
    if (std::fabs(r) <= A.eps) r = 0;
  };
  for (auto &loop : loops) {
    for (Elem &e : loop) {
      Elem m = e;
      rz(e.r0, e.z0, m.r0, m.z0), rz(e.r1, e.z1, m.r1, m.z1);
      if (e.arc) {
        V3 c{e.cr - a.x, e.cz - a.y, 0};
        m.cr = dot2(c, N), m.cz = dot2(c, D);
        V3 u{trig::cos(e.a0), trig::sin(e.a0), 0};
        m.a0 = trig::atan2(dot2(u, D), dot2(u, N)), m.a1 = m.a0 + (e.a1 - e.a0) * (turnsOver < 0 ? -1 : 1);
      }
      e = m;
    }
    if (turnsOver < 0) {
      std::reverse(loop.begin(), loop.end());
      for (Elem &e : loop) std::swap(e.r0, e.r1), std::swap(e.z0, e.z1), std::swap(e.a0, e.a1);
    }
  }
  double turn = (form.high - form.low) * pi / 180;
  std::shared_ptr<Model> m;
  // (A turned profile's own mesh gives every arc one chord at least: one whose ends are another piece's, as revolvedModel
  // gives two, would lie on that piece.)
  bool twins = false;
  for (const auto &l : fewestChords(loops))
    for (int n : l) twins = twins || n > 1;
  if (turn >= 2 * pi - 1e-12 && loops.size() == 1 && !twins) m = turnedModel(loops[0]);
  else m = revolvedModel(std::move(loops), std::move(region), std::min(turn, 2 * pi));
  // Its frame: x out along N, z along the line, y the way it turns (z × x), at the line's start; turned by `low` first.
  V3 Y = cross(D, N);
  Affine frame;
  frame.m[0] = N.x, frame.m[1] = Y.x, frame.m[2] = D.x, frame.m[3] = a.x;
  frame.m[4] = N.y, frame.m[5] = Y.y, frame.m[6] = D.y, frame.m[7] = a.y;
  frame.m[8] = 0, frame.m[9] = Y.z, frame.m[10] = 0, frame.m[11] = 0;
  Affine start;
  double low = form.low * pi / 180, c = trig::cos(low), sn = trig::sin(low);
  if (form.low != 0) start.m[0] = c, start.m[1] = -sn, start.m[4] = sn, start.m[5] = c;
  out = shapeOf(m, start.then(frame));
  return true;
}

}  // namespace bce
