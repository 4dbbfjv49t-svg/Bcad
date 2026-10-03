// A mesh's edges as the corners they are: each edge between two faces described point by point (the faces' normals there,
// exactly where the faces' forms are known, and the directions running from the edge into each), what a pick stands for,
// runs of edges meeting smoothly taken whole, and the cut across an edge.
#include "Engine/Treat.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <unordered_map>

namespace bce {

namespace {

// Points by their exact position.
struct Key {
  uint64_t x, y, z;
  bool operator==(const Key &o) const { return x == o.x && y == o.y && z == o.z; }
};
struct KeyHash {
  size_t operator()(const Key &k) const { return (size_t)(k.x * 0x9E3779B97F4A7C15ull ^ (k.y + 0x7F4A7C159E3779B9ull) * 31 ^ k.z * 0xBF58476D1CE4E5B9ull); }
};
Key keyOf(V3 p) {
  Key k;
  // -0 and +0 are one place.
  double x = p.x + 0.0, y = p.y + 0.0, z = p.z + 0.0;
  std::memcpy(&k.x, &x, 8), std::memcpy(&k.y, &y, 8), std::memcpy(&k.z, &z, 8);
  return k;
}

// The triangle corners (3t + k) at each position of a mesh.
struct Index {
  const Solid &s;
  std::unordered_map<Key, std::vector<uint32_t>, KeyHash> corners;
  explicit Index(const Solid &solid) : s(solid) {
    corners.reserve(s.p.size());
    for (size_t c = 0; c < s.tri.size(); c++) corners[keyOf(s.p[s.tri[c]])].push_back((uint32_t)c);
  }
  const std::vector<uint32_t> *at(V3 p) const {
    auto it = corners.find(keyOf(p));
    return it == corners.end() ? nullptr : &it->second;
  }
  // Face f's outward normal at p: its plane's; on a turned face its exact one (at `exact`, p's place on the face itself
  // when known); otherwise its mesh's there.
  V3 normal(int f, V3 p) const { return normal(f, p, p); }
  V3 normal(int f, V3 p, V3 exact) const {
    const auto &face = s.faces[f];
    if (face.geom.flat) return face.geom.pn;
    V3 sum;
    if (auto *cs = at(p))
      for (uint32_t c : *cs)
        if ((int)s.triFace[c / 3] == f) sum += s.n[s.tri[c]];
    V3 mesh = norm(sum) > 0 ? unit(sum) : face.normal;
    const FaceGeom &g = face.geom;
    if (g.kind == FaceGeom::Turned && g.exact) {
      V3 q = g.place.inverse().point(exact);
      double r = std::hypot(q.x, q.y), nr, nz;
      g.elem.normalAt(g.elem.nearest(r, q.z), nr, nz);
      V3 local = r > 1e-12 ? V3{nr * q.x / r, nr * q.y / r, nz} : V3{0, 0, nz >= 0 ? 1.0 : -1.0};
      V3 w = g.place.normal(local);
      if (norm(w) > 0) return dot(w, mesh) < 0 ? -w : w;
    }
    return mesh;
  }
  // Where a flat face and a turned one meet near p, on both exactly (an edge's mesh point between them lies a chord's sag
  // off it, a degree or so round a small turned face): p moved onto each in turn until it stays. p itself otherwise.
  V3 onBoth(int fa, int fb, V3 p) const {
    const FaceGeom *flat = nullptr, *turned = nullptr;
    for (int f : {fa, fb}) {
      const FaceGeom &g = s.faces[f].geom;
      if (g.flat) flat = &g;
      else if (g.kind == FaceGeom::Turned && g.exact) turned = &g;
    }
    if (!flat || !turned) return p;
    Affine back = turned->place.inverse();
    V3 q = p;
    // Slow where they meet at a shallow angle (each round takes off the cosine squared of it): rounds enough for that.
    for (int k = 0; k < 400; k++) {
      V3 L = back.point(q);
      double r = std::hypot(L.x, L.y), er, ez;
      if (r < 1e-12) return p;
      turned->elem.at(turned->elem.nearest(r, L.z), er, ez);
      V3 on = turned->place.point(V3{L.x / r * er, L.y / r * er, ez});
      V3 next = on - flat->pn * (dot(flat->pn, on) - flat->pd);
      bool still = norm(next - q) < 1e-12 * (1 + norm(q));
      q = next;
      if (still) break;
    }
    // Only near: farther off than a mesh's sag, they don't meet here.
    return norm(q - p) < 0.25 * (1 + std::sqrt(norm(p))) ? q : p;
  }
  // +1 when a triangle of face f runs from p0 to p1 (the face lies to the left of that way, seen from outside), -1 when one
  // runs back, 0 when none has that side.
  int runs(int f, V3 p0, V3 p1) const {
    auto *cs = at(p0);
    if (!cs) return 0;
    for (uint32_t c : *cs) {
      uint32_t t = c / 3, k = c % 3;
      if ((int)s.triFace[t] != f) continue;
      if (s.p[s.tri[3 * t + (k + 1) % 3]] == p1) return 1;
      if (s.p[s.tri[3 * t + (k + 2) % 3]] == p1) return -1;
    }
    return 0;
  }
};

double lengthOf(const std::vector<V3> &pts) {
  double l = 0;
  for (size_t i = 1; i < pts.size(); i++) l += norm(pts[i] - pts[i - 1]);
  return l;
}

// The material angle from face A's into-direction and normal and face B's into-direction (as OpenCascade's kernel counts it).
double materialAngle(V3 ia, V3 na, V3 ib) {
  double deg = std::atan2(dot(ib, na), -dot(ib, ia)) * 180 / M_PI - 180;
  return deg <= 0 ? deg + 360 : deg;
}

// Face A of two faces with these outward normals: the one pointing most up (ties: most +y, then +x).
bool firstUp(V3 n1, V3 n2) {
  if (std::fabs(n1.z - n2.z) >= 1e-6) return n1.z > n2.z;
  if (std::fabs(n1.y - n2.y) >= 1e-6) return n1.y > n2.y;
  return n1.x > n2.x;
}

bool isEdge(const Solid::Edge &e) { return e.f0 >= 0 && e.f1 >= 0 && e.f0 != e.f1 && e.pts.size() >= 2; }

// Normals and into-directions of faces fa and fb at the points of a line of points, filled into c (from index `from`).
void describe(const Index &ix, Crease &c, size_t from) {
  size_t n = c.pts.size();
  c.na.resize(n), c.nb.resize(n), c.ia.resize(n), c.ib.resize(n);
  for (size_t i = from; i < n; i++) {
    V3 p = c.pts[i];
    // The side of the line beside this point, for which way round each face runs it.
    size_t a = i + 1 < n ? i : i - 1, b = a + 1;
    if (c.closed && i + 1 == n) a = i - 1, b = i;
    V3 dir = unit(c.pts[b] - c.pts[a]), t = c.tangent(i);
    V3 exact = ix.onBoth(c.fa[i], c.fb[i], p);
    c.na[i] = ix.normal(c.fa[i], p, exact), c.nb[i] = ix.normal(c.fb[i], p, exact);
    // The way along it: square to both faces' normals where they meet at an angle (a line of mesh points crossing a
    // curved face zigzags by a chord's sag, turning its own way by degrees from point to point).
    V3 along = cross(c.na[i], c.nb[i]);
    if (norm(along) > 0.05) {
      along = unit(along);
      if (dot(along, t) < 0) along = -along;
      t = along;
    }
    auto side = [&](int f, V3 nrm) {
      int r = ix.runs(f, c.pts[a], c.pts[b]);
      V3 into = cross(nrm, dir) * (r < 0 ? -1.0 : 1.0);
      if (r == 0) {
        // No triangle of the face along this side (a seam point): from the face's middle.
        V3 toward = ix.s.faces[f].centroid - p;
        into = cross(nrm, dir);
        if (dot(into, toward) < 0) into = -into;
      }
      return unit(into - t * dot(into, t));
    };
    c.ia[i] = side(c.fa[i], c.na[i]), c.ib[i] = side(c.fb[i], c.nb[i]);
  }
}

double tolOf(const Solid &s) {
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (V3 q : s.p) lo = vmin(lo, q), hi = vmax(hi, q);
  double diag = s.p.empty() ? 10 : norm(hi - lo);
  return std::max(0.3, diag * 0.02);
}

// The point of a line of points nearest q, its segment, and the tangent there.
double nearestOn(const std::vector<V3> &pts, V3 q, size_t &seg, V3 &at) {
  double best = INFINITY;
  for (size_t i = 0; i + 1 < pts.size(); i++) {
    V3 a = pts[i], d = pts[i + 1] - pts[i];
    double l2 = norm2(d), t = l2 > 0 ? std::clamp(dot(q - a, d) / l2, 0.0, 1.0) : 0;
    V3 p = a + d * t;
    double dist = norm(p - q);
    if (dist < best) best = dist, seg = i, at = p;
  }
  return best;
}

// The edge described as a crease from its own two faces (A the one pointing most up at its middle).
Crease single(const Index &ix, const Solid::Edge &e, int edge) {
  Crease c;
  c.pts = e.pts;
  c.edges = {edge};
  c.closed = c.pts.size() > 2 && c.pts.front() == c.pts.back();
  c.length = lengthOf(c.pts);
  // The middle by length.
  double half = c.length / 2, run = 0;
  V3 mid = c.pts[0];
  for (size_t i = 1; i < c.pts.size(); i++) {
    double l = norm(c.pts[i] - c.pts[i - 1]);
    if (run + l >= half) {
      mid = c.pts[i - 1] + (c.pts[i] - c.pts[i - 1]) * (l > 0 ? (half - run) / l : 0);
      break;
    }
    run += l;
  }
  // Normals at the mesh point nearest the middle.
  size_t k = 0;
  double best = INFINITY;
  for (size_t i = 0; i < c.pts.size(); i++)
    if (norm(c.pts[i] - mid) < best) best = norm(c.pts[i] - mid), k = i;
  bool up = firstUp(ix.normal(e.f0, c.pts[k]), ix.normal(e.f1, c.pts[k]));
  c.fa.assign(c.pts.size(), up ? e.f0 : e.f1);
  c.fb.assign(c.pts.size(), up ? e.f1 : e.f0);
  describe(ix, c, 0);
  c.angle = c.angleAt(k);
  return c;
}

// Edges meeting at a point: (edge, which end: 0 its first point, 1 its last).
using Ends = std::unordered_map<Key, std::vector<std::pair<int, int>>, KeyHash>;

Ends endsOf(const Solid &s, const std::vector<char> &usable) {
  Ends ends;
  for (size_t e = 0; e < s.edges.size(); e++) {
    if (!usable[e]) continue;
    const auto &pts = s.edges[e].pts;
    if (pts.front() == pts.back()) continue;
    ends[keyOf(pts.front())].push_back({(int)e, 0});
    ends[keyOf(pts.back())].push_back({(int)e, 1});
  }
  return ends;
}

// The way an edge leaves one of its ends (into the edge).
V3 leaving(const Solid::Edge &e, int end) {
  const auto &p = e.pts;
  V3 chord = end == 0 ? unit(p[1] - p[0]) : unit(p[p.size() - 2] - p.back());
  if (e.geom.kind != EdgeGeom::Circle || !e.geom.exact) return chord;
  // Round a circle: the circle's own tangent there (a chord leaves half a chord's turn off it).
  V3 centre = e.geom.place.point({0, 0, e.geom.z}), axis = unit(e.geom.place.vector({0, 0, 1}));
  V3 at = end == 0 ? p.front() : p.back(), t = unit(cross(axis, at - centre));
  return dot(t, chord) < 0 ? -t : t;
}

bool shareFace(const Solid::Edge &a, const Solid::Edge &b) { return a.f0 == b.f0 || a.f0 == b.f1 || a.f1 == b.f0 || a.f1 == b.f1; }

}  // namespace

V3 Crease::tangent(size_t i) const {
  size_t n = pts.size();
  if (n < 2) return {0, 0, 1};
  V3 back = i > 0 ? pts[i] - pts[i - 1] : closed ? pts[n - 1] - pts[n - 2] : V3{};
  V3 ahead = i + 1 < n ? pts[i + 1] - pts[i] : closed ? pts[1] - pts[0] : V3{};
  V3 t = unit(back) + unit(ahead);
  return norm(t) > 1e-12 ? unit(t) : unit(norm(ahead) > 0 ? ahead : back);
}

double Crease::angleAt(size_t i) const { return materialAngle(ia[i], na[i], ib[i]); }

int faceAt(const Solid &s, const double *pick) {
  V3 a{pick[0], pick[1], pick[2]}, b{pick[3], pick[4], pick[5]};
  double tol = tolOf(s);
  // The face whose middle is nearest with a normal facing the same way; failing that, the face the point lies on.
  V3 n = norm(a) > 1e-9 ? unit(a) : V3{};
  double best = tol;
  int hit = -1;
  for (size_t f = 0; f < s.faces.size(); f++) {
    bool facing = norm(n) == 0 || dot(s.faces[f].normal, n) >= 0.7;
    double dist = norm(s.faces[f].centroid - b) + (facing ? 0 : tol);
    if (dist < best) best = dist, hit = (int)f;
  }
  if (hit >= 0) return hit;
  double close = std::min(tol, 0.05);
  for (size_t t = 0; t < s.triFace.size(); t++) {
    int f = (int)s.triFace[t];
    if (norm(n) > 0 && dot(s.faces[f].normal, n) < 0.7) continue;
    V3 A = s.p[s.tri[3 * t]], B = s.p[s.tri[3 * t + 1]], C = s.p[s.tri[3 * t + 2]], nn = unit(cross(B - A, C - A));
    double h = std::fabs(dot(b - A, nn));
    // Inside the triangle (by its sides' turns) and on its plane.
    V3 p = b - nn * dot(b - A, nn);
    bool in = dot(cross(B - A, p - A), nn) >= 0 && dot(cross(C - B, p - B), nn) >= 0 && dot(cross(A - C, p - C), nn) >= 0;
    if (in && h < close) close = h, hit = f;
  }
  return hit;
}

std::vector<Crease> creasesOf(const Solid &s, const int *kinds, const double *picks, int count, int *missing) {
  Index ix(s);
  double tol = tolOf(s);
  // Edges between two faces that don't meet smoothly.
  std::vector<char> usable(s.edges.size(), 0);
  std::vector<Crease> described(s.edges.size());
  for (size_t e = 0; e < s.edges.size(); e++) {
    if (!isEdge(s.edges[e])) continue;
    described[e] = single(ix, s.edges[e], (int)e);
    usable[e] = std::fabs(described[e].angle - 180) > 1;
  }
  std::set<int> chosen;
  int miss = 0;
  for (int i = 0; i < count; i++) {
    const double *q = picks + 6 * i;
    V3 a{q[0], q[1], q[2]}, b{q[3], q[4], q[5]};
    bool found = false;
    switch (kinds[i]) {
    case BK_PICK_BODY:
      for (size_t e = 0; e < s.edges.size(); e++)
        if (usable[e]) chosen.insert((int)e);
      found = true;
      break;
    case BK_PICK_EDGE: {
      // The edge running through the picked point, roughly along the picked direction.
      double best = tol;
      int hit = -1;
      for (size_t e = 0; e < s.edges.size(); e++) {
        if (!usable[e]) continue;
        size_t seg = 0;
        V3 at;
        double dist = nearestOn(s.edges[e].pts, a, seg, at);
        V3 t = unit(s.edges[e].pts[seg + 1] - s.edges[e].pts[seg]);
        if (norm(b) > 1e-9 && std::fabs(dot(t, unit(b))) < 0.8) dist += tol;
        if (dist < best) best = dist, hit = (int)e;
      }
      if (hit >= 0) chosen.insert(hit), found = true;
      break;
    }
    case BK_PICK_CORNER: {
      // The corner nearest the picked point, and of its edges the one running most along the face's normal.
      double best = tol;
      V3 corner;
      bool any = false;
      for (V3 c : s.corners)
        if (norm(c - b) < best) best = norm(c - b), corner = c, any = true;
      if (!any || norm(a) < 1e-9) break;
      V3 n = unit(a);
      double bestDot = 0.3;
      int hit = -1;
      for (size_t e = 0; e < s.edges.size(); e++) {
        if (!usable[e]) continue;
        const auto &pts = s.edges[e].pts;
        for (int end = 0; end < 2; end++) {
          if (norm((end == 0 ? pts.front() : pts.back()) - corner) > 1e-9 * (1 + norm(corner))) continue;
          double d = std::fabs(dot(leaving(s.edges[e], end), n));
          if (d > bestDot) bestDot = d, hit = (int)e;
        }
      }
      if (hit >= 0) chosen.insert(hit), found = true;
      break;
    }
    case BK_PICK_FACE: {
      int hit = faceAt(s, q);
      if (hit >= 0) {
        for (size_t e = 0; e < s.edges.size(); e++)
          if (usable[e] && (s.edges[e].f0 == hit || s.edges[e].f1 == hit)) chosen.insert((int)e);
        found = true;
      }
      break;
    }
    default:
      break;
    }
    if (!found) miss++;
  }
  if (missing) *missing = miss;

  // Runs meeting smoothly are taken whole (as OpenCascade rounds them): an edge going on from a chosen one's end the same
  // way, beside one of its faces — or beside none, its faces going on smoothly from the edge's (where two roundings
  // meet at a corner, the line between them goes on from the sharp edge that ends there).
  Ends ends = endsOf(s, usable);
  const double smooth = std::cos(3 * M_PI / 180), alike = std::cos(1.5 * M_PI / 180), along = std::cos(20 * M_PI / 180);
  auto carriesOn = [&](const Solid::Edge &a, const Solid::Edge &b, V3 at) {
    if (a.f0 < 0 || a.f1 < 0 || b.f0 < 0 || b.f1 < 0) return false;
    V3 n0 = ix.normal(a.f0, at), n1 = ix.normal(a.f1, at), m0 = ix.normal(b.f0, at), m1 = ix.normal(b.f1, at);
    return (dot(n0, m0) >= alike && dot(n1, m1) >= alike) || (dot(n0, m1) >= alike && dot(n1, m0) >= alike);
  };
  auto onward = [&](int e, int end) {
    std::vector<int> next;
    V3 out = -leaving(s.edges[e], end);
    const auto &pts = s.edges[e].pts;
    V3 at = end == 0 ? pts.front() : pts.back();
    auto it = ends.find(keyOf(at));
    if (it == ends.end()) return next;
    for (auto [o, oe] : it->second) {
      if (o == e) continue;
      double d = dot(leaving(s.edges[o], oe), out);
      if (shareFace(s.edges[e], s.edges[o]) ? d >= smooth : d >= along && carriesOn(s.edges[e], s.edges[o], at)) next.push_back(o);
    }
    return next;
  };
  std::vector<int> todo(chosen.begin(), chosen.end());
  while (!todo.empty()) {
    int e = todo.back();
    todo.pop_back();
    for (int end = 0; end < 2; end++)
      for (int o : onward(e, end))
        if (chosen.insert(o).second) todo.push_back(o);
  }

  // The runs: from an end that doesn't go on (or anywhere on a loop), edge after edge.
  std::vector<Crease> out;
  std::set<int> walked;
  auto walk = [&](int first, int startEnd) {
    Crease c;
    int e = first, from = startEnd;
    std::vector<std::pair<int, bool>> parts;  // edge, reversed
    while (e >= 0 && !walked.count(e)) {
      walked.insert(e);
      parts.push_back({e, from == 1});
      int other = 1 - from, next = -1, nextEnd = 0;
      for (int o : onward(e, other))
        if (chosen.count(o) && !walked.count(o)) {
          const auto &op = s.edges[o].pts;
          const auto &ep = s.edges[e].pts;
          V3 at = other == 0 ? ep.front() : ep.back();
          next = o, nextEnd = op.front() == at ? 0 : 1;
          break;
        }
      e = next, from = nextEnd;
    }
    if (parts.empty()) return;
    // Points, and each point's faces: those of the edge it belongs to, A kept on the side of the face the edges share.
    int prevA = -1, prevB = -1;
    for (size_t k = 0; k < parts.size(); k++) {
      const Solid::Edge &edge = s.edges[parts[k].first];
      std::vector<V3> pts = edge.pts;
      if (parts[k].second) std::reverse(pts.begin(), pts.end());
      int A, B;
      if (k == 0) {
        A = described[parts[k].first].fa[0], B = described[parts[k].first].fb[0];
      } else if (edge.f0 == prevA || edge.f1 == prevB) {
        A = edge.f0, B = edge.f1;
      } else if (edge.f1 == prevA || edge.f0 == prevB) {
        A = edge.f1, B = edge.f0;
      } else {
        // No face in common: A goes on as the face turned most like the last A.
        V3 was = ix.normal(prevA, pts[0]);
        A = edge.f0, B = edge.f1;
        if (dot(ix.normal(edge.f0, pts[0]), was) < dot(ix.normal(edge.f1, pts[0]), was)) std::swap(A, B);
      }
      size_t skip = c.pts.empty() ? 0 : 1;
      for (size_t i = skip; i < pts.size(); i++) c.pts.push_back(pts[i]), c.fa.push_back(A), c.fb.push_back(B);
      c.edges.push_back(parts[k].first);
      prevA = A, prevB = B;
    }
    c.closed = c.pts.size() > 2 && c.pts.front() == c.pts.back();
    c.length = lengthOf(c.pts);
    describe(ix, c, 0);
    // Face A by the rule at the run's middle: the faces swapped all along when the other points more up.
    size_t mid = c.pts.size() / 2;
    if (firstUp(c.nb[mid], c.na[mid])) {
      std::swap(c.fa, c.fb);
      std::swap(c.na, c.nb);
      std::swap(c.ia, c.ib);
    }
    c.angle = c.angleAt(mid);
    out.push_back(c);
  };
  for (int e : chosen) {
    if (walked.count(e)) continue;
    // Back to the start of its run, unless it's a loop.
    int cur = e, end = 0;
    std::set<int> seen{e};
    for (;;) {
      int prev = -1, prevEnd = 0;
      // Not back into a run already walked: where three smooth edges meet, the third starts a run of its own.
      for (int o : onward(cur, end))
        if (chosen.count(o) && !seen.count(o) && !walked.count(o)) {
          const auto &op = s.edges[o].pts;
          const auto &cp = s.edges[cur].pts;
          V3 at = end == 0 ? cp.front() : cp.back();
          prev = o, prevEnd = op.front() == at ? 1 : 0;
          break;
        }
      if (prev < 0) break;
      seen.insert(prev);
      cur = prev, end = prevEnd;
    }
    walk(cur, end);
  }
  return out;
}

size_t pointAt(Crease &c, double along) {
  double run = 0;
  for (size_t i = 0; i + 1 < c.pts.size(); i++) {
    double l = norm(c.pts[i + 1] - c.pts[i]);
    if (run + l < along && i + 2 < c.pts.size()) {
      run += l;
      continue;
    }
    double t = l > 0 ? std::clamp((along - run) / l, 0.0, 1.0) : 0;
    if (t <= 1e-9) return i;
    if (t >= 1 - 1e-9) return i + 1;
    auto mix = [&](std::vector<V3> &v) { v.insert(v.begin() + (long)i + 1, unit(v[i] * (1 - t) + v[i + 1] * t)); };
    c.pts.insert(c.pts.begin() + (long)i + 1, c.pts[i] + (c.pts[i + 1] - c.pts[i]) * t);
    c.fa.insert(c.fa.begin() + (long)i + 1, c.fa[i]);
    c.fb.insert(c.fb.begin() + (long)i + 1, c.fb[i]);
    mix(c.na), mix(c.nb), mix(c.ia), mix(c.ib);
    return i + 1;
  }
  return c.pts.size() / 2;
}

bool creaseAt(const Solid &s, int kind, const double *pick, Crease &out, size_t &at) {
  int missing = 0;
  std::vector<Crease> found = creasesOf(s, &kind, pick, 1, &missing);
  // A body pick shows its longest convex edge; any other its longest.
  const Crease *best = nullptr;
  for (const auto &c : found) {
    if (kind == BK_PICK_BODY && c.angle >= 179) continue;
    if (!best || c.length > best->length) best = &c;
  }
  if (!best) return false;
  out = *best;
  double along = out.length / 2;
  if (kind == BK_PICK_EDGE) {
    // The point of the run nearest the pick, kept off the ends of an open run.
    V3 q{pick[0], pick[1], pick[2]}, p;
    size_t seg = 0;
    nearestOn(out.pts, q, seg, p);
    along = 0;
    for (size_t i = 0; i < seg; i++) along += norm(out.pts[i + 1] - out.pts[i]);
    along += norm(p - out.pts[seg]);
    if (!out.closed) along = std::clamp(along, out.length * 0.02, out.length * 0.98);
  }
  at = pointAt(out, along);
  return true;
}

std::vector<std::vector<std::pair<double, double>>> sliceAcross(const Solid &s, const Crease &c, size_t i) {
  V3 E = c.pts[i], x = -c.ia[i], y = unit(c.na[i] - x * dot(c.na[i], x)), z = cross(x, y);
  // Cut keeping the side behind the plane: its new face faces +z, its outline running counter-clockwise round it.
  Solid half = cut(s, E, -z, 0);
  std::vector<std::vector<std::pair<double, double>>> loops;
  if (half.faces.size() <= s.faces.size()) return loops;
  uint32_t cap = (uint32_t)half.faces.size() - 1;
  // The cap's sides that no other cap triangle shares: its outline.
  std::unordered_map<Key, std::vector<std::pair<Key, V3>>, KeyHash> next;
  std::set<std::pair<std::array<uint64_t, 3>, std::array<uint64_t, 3>>> sides;
  auto arr = [](Key k) { return std::array<uint64_t, 3>{k.x, k.y, k.z}; };
  for (size_t t = 0; t < half.triFace.size(); t++)
    if (half.triFace[t] == cap)
      for (int k = 0; k < 3; k++) sides.insert({arr(keyOf(half.p[half.tri[3 * t + k]])), arr(keyOf(half.p[half.tri[3 * t + (k + 1) % 3]]))});
  for (size_t t = 0; t < half.triFace.size(); t++) {
    if (half.triFace[t] != cap) continue;
    for (int k = 0; k < 3; k++) {
      V3 a = half.p[half.tri[3 * t + k]], b = half.p[half.tri[3 * t + (k + 1) % 3]];
      if (sides.count({arr(keyOf(b)), arr(keyOf(a))})) continue;
      next[keyOf(a)].push_back({keyOf(b), b});
    }
  }
  std::unordered_map<Key, V3, KeyHash> where;
  for (size_t t = 0; t < half.triFace.size(); t++)
    if (half.triFace[t] == cap)
      for (int k = 0; k < 3; k++) where[keyOf(half.p[half.tri[3 * t + k]])] = half.p[half.tri[3 * t + k]];
  std::set<std::pair<std::array<uint64_t, 3>, std::array<uint64_t, 3>>> used;
  for (auto &[start, outs] : next) {
    for (auto &first : outs) {
      if (used.count({arr(start), arr(first.first)})) continue;
      std::vector<std::pair<double, double>> loop;
      Key at = start;
      V3 atP = where[start];
      std::pair<Key, V3> step = first;
      for (size_t guard = 0; guard < 1000000; guard++) {
        used.insert({arr(at), arr(step.first)});
        V3 d = atP - E;
        loop.push_back({dot(d, x), dot(d, y)});
        at = step.first, atP = step.second;
        if (at == start) break;
        auto it = next.find(at);
        if (it == next.end()) break;
        const std::pair<Key, V3> *go = nullptr;
        for (auto &o : it->second)
          if (!used.count({arr(at), arr(o.first)})) go = &o;
        if (!go) break;
        step = *go;
      }
      if (loop.size() >= 3) loops.push_back(loop);
    }
  }
  return loops;
}

}  // namespace bce
