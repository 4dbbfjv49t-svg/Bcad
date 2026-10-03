// Merge, subtract and intersect on meshes. Both shapes' points first go on one fine grid (so faces meant to lie on each
// other do, exactly). Every decision about where they meet is then an exact orientation; where two triangles cross, each
// crossing point is made once — named by the edge and triangle (or two edges) it lies on — and used by every triangle it
// touches, so both shapes are cut at the same points and the result is closed (crossing points that come out a hair
// apart by different roads are made one). Faces lying on each other are cut in their shared plane. Each region the cuts
// leave is inside or outside the other shape by one winding number; what's kept depends on the operation, and faces
// lying on each other follow the usual rules (a merge keeps one of two faces facing the same way and drops two facing
// each other; a subtract leaves no skin). Every piece kept takes its share of its triangle's slivers (Solid::gap).
#include "Engine/Model.hpp"
#include "Engine/Triangulate.hpp"
#include "Engine/Weld.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace bce {

thread_local CombineReport combineReport;

namespace {

uint64_t pairKey(uint32_t a, uint32_t b) { return (uint64_t)std::min(a, b) << 32 | std::max(a, b); }

// MARK: - flat tests in a triangle's plane

struct Flat {
  int axis;
  bool swap;
  // The plane's two coordinates kept (exact copies), in an order that keeps the triangle counter-clockwise.
  void at(V3 p, double &x, double &y) const {
    double u = axis == 0 ? p.y : axis == 1 ? p.z : p.x, v = axis == 0 ? p.z : axis == 1 ? p.x : p.y;
    x = swap ? v : u, y = swap ? u : v;
  }
};

Flat flatOf(V3 a, V3 b, V3 c) {
  V3 n = cross(b - a, c - a);
  int axis = std::fabs(n.x) >= std::fabs(n.y) && std::fabs(n.x) >= std::fabs(n.z) ? 0 : std::fabs(n.y) >= std::fabs(n.z) ? 1 : 2;
  return {axis, n[axis] < 0};
}

int orientFlat(const Flat &f, V3 a, V3 b, V3 c) {
  double ax, ay, bx, by, cx, cy;
  f.at(a, ax, ay), f.at(b, bx, by), f.at(c, cx, cy);
  return orient2d(ax, ay, bx, by, cx, cy);
}

// Where p (in the triangle's plane) lies: -1 outside, 0 inside, 1 + m on side m (corner m to m + 1), 4 + m on corner m.
int where(const Flat &f, const V3 t[3], V3 p) {
  int o[3], zeros = 0;
  for (int m = 0; m < 3; m++) {
    o[m] = orientFlat(f, t[m], t[(m + 1) % 3], p);
    if (o[m] < 0) return -1;
    zeros += o[m] == 0;
  }
  if (zeros == 0) return 0;
  if (zeros == 1)
    for (int m = 0; m < 3; m++)
      if (o[m] == 0) return 1 + m;
  for (int m = 0; m < 3; m++)
    if (o[m] == 0 && o[(m + 1) % 3] == 0) return 4 + (m + 1) % 3;
  return -1;
}

// MARK: - the two shapes being cut

struct Part {
  Welded w;
  uint32_t base = 0;                          // the global number of its first point
  std::vector<uint64_t> edges;                // its edges as (point, point), sorted: an edge's number is its place
  std::vector<std::vector<uint32_t>> onEdge;  // points to put on each edge
  struct Cut {
    std::vector<uint32_t> in;
    std::vector<std::pair<uint32_t, uint32_t>> segs;
    std::vector<uint32_t> partners;  // the other shape's triangles lying flat on this one
    bool any() const { return !in.empty() || !segs.empty() || !partners.empty(); }
  };
  std::vector<Cut> cut;  // per triangle

  uint32_t corner(uint32_t t, int k) const { return base + w.tri[3 * t + k]; }
  uint32_t edgeOf(uint32_t a, uint32_t b) const { return (uint32_t)(std::lower_bound(edges.begin(), edges.end(), pairKey(a, b)) - edges.begin()); }
};

struct Box {
  V3 lo, hi;
};

// A tree of boxes over one shape's triangles, for finding the other shape's triangles near each one.
struct Boxes {
  struct Node {
    Box b;
    int left = -1, right = -1, first = 0, count = 0;
  };
  std::vector<Node> nodes;
  std::vector<uint32_t> order;
  std::vector<Box> boxes;

  explicit Boxes(const std::vector<Box> &bx) : boxes(bx) {
    order.resize(bx.size());
    std::iota(order.begin(), order.end(), 0);
    if (!bx.empty()) make(0, (int)bx.size());
  }
  int make(int first, int count) {
    int at = (int)nodes.size();
    nodes.push_back({});
    Box b = boxes[order[first]];
    for (int i = first + 1; i < first + count; i++) b.lo = vmin(b.lo, boxes[order[i]].lo), b.hi = vmax(b.hi, boxes[order[i]].hi);
    nodes[at].b = b;
    if (count <= 4) {
      nodes[at].first = first, nodes[at].count = count;
      return at;
    }
    V3 ext = b.hi - b.lo;
    int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : ext.y >= ext.z ? 1 : 2;
    int half = count / 2;
    std::nth_element(order.begin() + first, order.begin() + first + half, order.begin() + first + count, [&](uint32_t u, uint32_t v) {
      return boxes[u].lo[axis] + boxes[u].hi[axis] < boxes[v].lo[axis] + boxes[v].hi[axis];
    });
    int l = make(first, half), r = make(first + half, count - half);
    nodes[at].left = l, nodes[at].right = r;
    return at;
  }
  template <typename F> void overlapping(const Box &q, F f) const {
    if (nodes.empty()) return;
    int stack[128], top = 0;
    stack[top++] = 0;
    while (top) {
      const Node &n = nodes[stack[--top]];
      if (n.b.lo.x > q.hi.x || n.b.hi.x < q.lo.x || n.b.lo.y > q.hi.y || n.b.hi.y < q.lo.y || n.b.lo.z > q.hi.z || n.b.hi.z < q.lo.z) continue;
      if (n.left < 0) {
        for (int i = n.first; i < n.first + n.count; i++) {
          const Box &b = boxes[order[i]];
          if (b.lo.x > q.hi.x || b.hi.x < q.lo.x || b.lo.y > q.hi.y || b.hi.y < q.lo.y || b.lo.z > q.hi.z || b.hi.z < q.lo.z) continue;
          f(order[i]);
        }
      } else if (top < 126) {
        stack[top++] = n.left, stack[top++] = n.right;
      }
    }
  }
};

struct KeyHash {
  size_t operator()(const std::array<uint32_t, 3> &k) const { return (size_t)k[0] * 0x9E3779B97F4A7C15ull ^ (size_t)k[1] * 0xC2B2AE3D27D4EB4Full ^ k[2]; }
};

struct Cutter {
  Part A, B;
  std::vector<V3> P;            // every point: A's, B's, then the new ones
  std::vector<uint32_t> parent;  // points found to be one
  std::unordered_map<std::array<uint32_t, 3>, uint32_t, KeyHash> made;
  std::vector<uint32_t> scratch;
  // Each point's number in the triangulation being made (an array kept between triangles, cleared as it's used).
  struct Local {
    std::vector<int> at;
    std::vector<uint32_t> used;
    void begin(size_t n) {
      for (uint32_t q : used) at[q] = -1;
      used.clear();
      if (at.size() < n) at.resize(n, -1);
    }
    bool has(uint32_t q) const { return at[q] >= 0; }
    int get(uint32_t q) const { return at[q]; }
    void set(uint32_t q, int i) {
      if (at[q] < 0) used.push_back(q);
      at[q] = i;
    }
  } local;

  uint32_t rep(uint32_t x) {
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
  }
  void unite(uint32_t a, uint32_t b) {
    a = rep(a), b = rep(b);
    if (a != b) parent[std::max(a, b)] = std::min(a, b);
  }
  template <typename F> uint32_t point(std::array<uint32_t, 3> key, const F &at) {
    auto it = made.find(key);
    if (it != made.end()) return it->second;
    uint32_t id = (uint32_t)P.size();
    P.push_back(at());
    parent.push_back(id);
    made[key] = id;
    return id;
  }
  // Kinds of crossing point: A's edge through B's triangle, A's triangle through B's edge, A's edge across B's edge.
  enum { EdgeTri = 1, TriEdge = 2, EdgeEdge = 3 };

  static V3 lineThroughPlane(V3 u, V3 v, V3 a, V3 b, V3 c) {
    V3 n = cross(b - a, c - a);
    double du = dot(u - a, n), dv = dot(v - a, n), den = du - dv;
    double t = den != 0 ? std::clamp(du / den, 0.0, 1.0) : 0.5;
    return u + (v - u) * t;
  }
  static V3 linesMeet(V3 p, V3 q, V3 r, V3 s) {
    V3 d1 = q - p, d2 = s - r, w = p - r;
    double a = dot(d1, d1), b = dot(d1, d2), c = dot(d2, d2), d = dot(d1, w), e = dot(d2, w), den = a * c - b * b;
    double t1 = den > 0 ? std::clamp((b * e - c * d) / den, 0.0, 1.0) : 0, t2 = den > 0 ? std::clamp((a * e - b * d) / den, 0.0, 1.0) : 0;
    return (p + d1 * t1 + r + d2 * t2) * 0.5;
  }

  void addOnEdge(Part &part, uint32_t a, uint32_t b, uint32_t p) { part.onEdge[part.edgeOf(a, b)].push_back(p); }
  void addIn(Part &part, uint32_t t, uint32_t p) { part.cut[t].in.push_back(p); }

  // Shape X's triangle tx against shape Y's triangle ty, X's edges through Y's triangle (its vertices on Y's plane
  // included). `xIsA` says which way round the names go. Points found are added to `found`.
  void edgesThrough(Part &X, uint32_t tx, Part &Y, uint32_t ty, const int oX[3], bool xIsA, std::vector<uint32_t> &found) {
    V3 y[3];
    uint32_t gy[3], gx[3];
    for (int k = 0; k < 3; k++) gy[k] = Y.corner(ty, k), y[k] = P[gy[k]], gx[k] = X.corner(tx, k);
    Flat fy = flatOf(y[0], y[1], y[2]);
    // X's corners on Y's plane.
    for (int k = 0; k < 3; k++) {
      if (oX[k] != 0) continue;
      int at = where(fy, y, P[gx[k]]);
      if (at < 0) continue;
      if (at == 0) addIn(Y, ty, gx[k]);
      else if (at <= 3) addOnEdge(Y, gy[at - 1], gy[at % 3], gx[k]);
      else unite(gx[k], gy[at - 4]);
      found.push_back(gx[k]);
    }
    for (int k = 0; k < 3; k++) {
      uint32_t u = gx[k], v = gx[(k + 1) % 3];
      int su = oX[k], sv = oX[(k + 1) % 3];
      if (su == 0 && sv == 0) {
        // The edge lies in Y's plane: where it crosses Y's edges, in that plane.
        for (int m = 0; m < 3; m++) {
          uint32_t c = gy[m], d = gy[(m + 1) % 3];
          int o1 = orientFlat(fy, P[u], P[v], P[c]), o2 = orientFlat(fy, P[u], P[v], P[d]);
          int o3 = orientFlat(fy, P[c], P[d], P[u]), o4 = orientFlat(fy, P[c], P[d], P[v]);
          if (o1 * o2 < 0 && o3 * o4 < 0) {
            uint32_t ex = X.edgeOf(u, v), ey = Y.edgeOf(c, d);
            std::array<uint32_t, 3> key = xIsA ? std::array<uint32_t, 3>{EdgeEdge, ex, ey} : std::array<uint32_t, 3>{EdgeEdge, ey, ex};
            uint32_t q = point(key, [&] { return linesMeet(P[u], P[v], P[c], P[d]); });
            addOnEdge(X, u, v, q), addOnEdge(Y, c, d, q);
            found.push_back(q);
          }
        }
        continue;
      }
      if (su * sv >= 0) continue;
      // Through Y's plane: inside Y's triangle, across one of its edges, or through one of its corners.
      int t[3], zeros = 0, pos = 0, neg = 0;
      for (int m = 0; m < 3; m++) {
        t[m] = orient3d(P[u], P[v], y[m], y[(m + 1) % 3]);
        zeros += t[m] == 0, pos += t[m] > 0, neg += t[m] < 0;
      }
      if (pos && neg) continue;
      uint32_t ex = X.edgeOf(u, v);
      if (zeros == 0) {
        std::array<uint32_t, 3> key{xIsA ? (uint32_t)EdgeTri : (uint32_t)TriEdge, xIsA ? ex : ty, xIsA ? ty : ex};
        uint32_t q = point(key, [&] { return lineThroughPlane(P[u], P[v], y[0], y[1], y[2]); });
        addOnEdge(X, u, v, q), addIn(Y, ty, q);
        found.push_back(q);
      } else if (zeros == 1) {
        int m = t[0] == 0 ? 0 : t[1] == 0 ? 1 : 2;
        uint32_t c = gy[m], d = gy[(m + 1) % 3], ey = Y.edgeOf(c, d);
        std::array<uint32_t, 3> key = xIsA ? std::array<uint32_t, 3>{EdgeEdge, ex, ey} : std::array<uint32_t, 3>{EdgeEdge, ey, ex};
        uint32_t q = point(key, [&] { return linesMeet(P[u], P[v], P[c], P[d]); });
        addOnEdge(X, u, v, q), addOnEdge(Y, c, d, q);
        found.push_back(q);
      } else {
        // Through a corner of Y: the corner itself, on X's edge.
        int m = 0;
        for (int i = 0; i < 3; i++)
          if (t[i] == 0 && t[(i + 2) % 3] == 0) m = i;
        addOnEdge(X, u, v, gy[m]);
        found.push_back(gy[m]);
      }
    }
  }

  // Two triangles in one plane: each one's corners inside the other, their edges' crossings, and each one's edges as cuts in
  // the other.
  void flatPair(uint32_t ta, uint32_t tb) {
    uint32_t ga[3], gb[3];
    V3 a[3], b[3];
    for (int k = 0; k < 3; k++) ga[k] = A.corner(ta, k), gb[k] = B.corner(tb, k), a[k] = P[ga[k]], b[k] = P[gb[k]];
    Flat f = flatOf(a[0], a[1], a[2]);
    // B's triangle counter-clockwise in A's view, for the inside tests.
    V3 bc[3] = {b[0], b[1], b[2]};
    uint32_t gbc[3] = {gb[0], gb[1], gb[2]};
    if (orientFlat(f, bc[0], bc[1], bc[2]) < 0) std::swap(bc[1], bc[2]), std::swap(gbc[1], gbc[2]);
    A.cut[ta].partners.push_back(tb);
    B.cut[tb].partners.push_back(ta);
    for (int k = 0; k < 3; k++) {
      int at = where(f, a, b[k]);
      if (at == 0) addIn(A, ta, gb[k]);
      else if (at >= 1 && at <= 3) addOnEdge(A, ga[at - 1], ga[at % 3], gb[k]);
      else if (at >= 4) unite(gb[k], ga[at - 4]);
      at = where(f, bc, a[k]);
      if (at == 0) addIn(B, tb, ga[k]);
      else if (at >= 1 && at <= 3) addOnEdge(B, gbc[at - 1], gbc[at % 3], ga[k]);
      else if (at >= 4) unite(ga[k], gbc[at - 4]);
    }
    // Proper crossings of the edges.
    std::vector<std::pair<uint32_t, uint32_t>> onA[3], onB[3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        uint32_t u = ga[i], v = ga[(i + 1) % 3], c = gb[j], d = gb[(j + 1) % 3];
        int o1 = orientFlat(f, P[u], P[v], P[c]), o2 = orientFlat(f, P[u], P[v], P[d]);
        int o3 = orientFlat(f, P[c], P[d], P[u]), o4 = orientFlat(f, P[c], P[d], P[v]);
        if (!(o1 * o2 < 0 && o3 * o4 < 0)) continue;
        uint32_t q = point({EdgeEdge, A.edgeOf(u, v), B.edgeOf(c, d)}, [&] {
          double ux, uy, vx, vy, cx, cy, dx, dy;
          f.at(P[u], ux, uy), f.at(P[v], vx, vy), f.at(P[c], cx, cy), f.at(P[d], dx, dy);
          double den = (vx - ux) * (dy - cy) - (vy - uy) * (dx - cx);
          double t = den != 0 ? std::clamp(((cx - ux) * (dy - cy) - (cy - uy) * (dx - cx)) / den, 0.0, 1.0) : 0.5;
          return P[u] + (P[v] - P[u]) * t;
        });
        addOnEdge(A, u, v, q), addOnEdge(B, c, d, q);
      }
    // Each one's edges inside the other become cuts there: the edge's points that lie in the other triangle, in order,
    // joined where the piece between them lies inside.
    auto edgesInto = [&](Part &X, uint32_t tx, const uint32_t gx[3], Part &Y, uint32_t ty, const V3 yt[3]) {
      for (int k = 0; k < 3; k++) {
        uint32_t u = gx[k], v = gx[(k + 1) % 3];
        std::vector<uint32_t> pts{u, v};
        for (uint32_t q : X.onEdge[X.edgeOf(u, v)]) pts.push_back(q);
        V3 dir = P[v] - P[u];
        std::sort(pts.begin(), pts.end(), [&](uint32_t p1, uint32_t p2) { return dot(P[p1] - P[u], dir) < dot(P[p2] - P[u], dir); });
        for (size_t i = 0; i + 1 < pts.size(); i++) {
          V3 mid = (P[pts[i]] + P[pts[i + 1]]) * 0.5;
          if (where(f, yt, mid) >= 0 && where(f, yt, P[pts[i]]) >= 0 && where(f, yt, P[pts[i + 1]]) >= 0) Y.cut[ty].segs.push_back({pts[i], pts[i + 1]});
        }
      }
      (void)tx;
    };
    edgesInto(B, tb, gb, A, ta, a);
    edgesInto(A, ta, ga, B, tb, bc);
  }

  void pairOf(uint32_t ta, uint32_t tb) {
    V3 a[3], b[3];
    for (int k = 0; k < 3; k++) a[k] = P[A.corner(ta, k)], b[k] = P[B.corner(tb, k)];
    int oB[3], oA[3];
    for (int k = 0; k < 3; k++) oB[k] = orient3d(a[0], a[1], a[2], b[k]);
    if ((oB[0] > 0 && oB[1] > 0 && oB[2] > 0) || (oB[0] < 0 && oB[1] < 0 && oB[2] < 0)) return;
    for (int k = 0; k < 3; k++) oA[k] = orient3d(b[0], b[1], b[2], a[k]);
    if ((oA[0] > 0 && oA[1] > 0 && oA[2] > 0) || (oA[0] < 0 && oA[1] < 0 && oA[2] < 0)) return;
    if (oB[0] == 0 && oB[1] == 0 && oB[2] == 0) {
      flatPair(ta, tb);
      return;
    }
    std::vector<uint32_t> &found = scratch;
    found.clear();
    edgesThrough(A, ta, B, tb, oA, true, found);
    edgesThrough(B, tb, A, ta, oB, false, found);
    for (auto &q : found) q = rep(q);
    std::sort(found.begin(), found.end());
    found.erase(std::unique(found.begin(), found.end()), found.end());
    if (found.size() < 2) return;
    // All on one line: in order along it, each next two joined.
    V3 dir = cross(cross(a[1] - a[0], a[2] - a[0]), cross(b[1] - b[0], b[2] - b[0]));
    std::sort(found.begin(), found.end(), [&](uint32_t p, uint32_t q) { return dot(P[p], dir) < dot(P[q], dir); });
    for (size_t i = 0; i + 1 < found.size(); i++) {
      A.cut[ta].segs.push_back({found[i], found[i + 1]});
      B.cut[tb].segs.push_back({found[i], found[i + 1]});
    }
  }

  // Each cut triangle of a shape re-made along its cuts: its pieces (global points) and which of their edges are cuts.
  struct Piece {
    uint32_t v[3];
    uint32_t from;  // the triangle it's a piece of
  };
  void remake(Part &X, std::vector<Piece> &out, std::unordered_set<uint64_t> &cuts) {
    size_t nt = X.w.count();
    out.reserve(nt * 2);
    for (uint32_t t = 0; t < nt; t++) {
      uint32_t g[3] = {X.corner(t, 0), X.corner(t, 1), X.corner(t, 2)};
      bool touched = X.cut[t].any();
      for (int k = 0; k < 3 && !touched; k++) touched = !X.onEdge[X.edgeOf(g[k], g[(k + 1) % 3])].empty();
      if (!touched) {
        out.push_back({{rep(g[0]), rep(g[1]), rep(g[2])}, t});
        continue;
      }
      Flat f = flatOf(P[g[0]], P[g[1]], P[g[2]]);
      double xy[3][2];
      for (int k = 0; k < 3; k++) f.at(P[g[k]], xy[k][0], xy[k][1]);
      Tri2 tri(xy[0][0], xy[0][1], xy[1][0], xy[1][1], xy[2][0], xy[2][1]);
      std::vector<uint32_t> global{rep(g[0]), rep(g[1]), rep(g[2])};
      local.begin(P.size());
      for (int k = 0; k < 3; k++) local.set(global[k], k);
      auto note = [&](int i, uint32_t q) {
        if (i >= (int)global.size()) global.resize(i + 1, q);
        else if (global[i] != q) unite(global[i], q), q = rep(q);
        local.set(q, i);
      };
      // Points on the sides, in order from each side's start.
      for (int k = 0; k < 3; k++) {
        uint32_t u = g[k], v = g[(k + 1) % 3];
        std::vector<uint32_t> pts;
        for (uint32_t q : X.onEdge[X.edgeOf(u, v)]) {
          q = rep(q);
          if (q != rep(u) && q != rep(v)) pts.push_back(q);
        }
        std::sort(pts.begin(), pts.end());
        pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
        V3 dir = P[v] - P[u];
        std::sort(pts.begin(), pts.end(), [&](uint32_t p1, uint32_t p2) { return dot(P[p1] - P[u], dir) < dot(P[p2] - P[u], dir); });
        int prev = local.get(rep(u));
        for (uint32_t q : pts) {
          if (local.has(q)) {
            prev = local.get(q);
            continue;
          }
          double x, y;
          f.at(P[q], x, y);
          int i = tri.insertOnEdge(prev, local.get(rep(v)), x, y);
          note(i, q);
          prev = i;
        }
      }
      {
        const Part::Cut &cutT = X.cut[t];
        for (uint32_t q : cutT.in) {
          q = rep(q);
          if (local.has(q)) continue;
          double x, y;
          f.at(P[q], x, y);
          // Known (exactly) to lie inside, though made in floating point it may have come out on a side.
          note(tri.insertWithin(x, y, (xy[0][0] + xy[1][0] + xy[2][0]) / 3, (xy[0][1] + xy[1][1] + xy[2][1]) / 3), q);
        }
        for (auto [p, q] : cutT.segs) {
          p = rep(p), q = rep(q);
          if (p == q) continue;
          if (!local.has(p) || !local.has(q)) {
            combineReport.segsDropped++;
            continue;
          }
          if (!tri.keep(local.get(p), local.get(q))) combineReport.keepsFailed++;
        }
      }
      // Points the triangulation made where two cuts crossed: put on the first cut in space too.
      for (auto &[i, m] : tri.made()) {
        if (i >= (int)global.size()) global.resize(i + 1, UINT32_MAX);
        (void)m;
      }
      for (int i = 0; i < (int)global.size(); i++) {
        auto it2 = tri.made().find(i);
        if (it2 == tri.made().end()) continue;
        V3 r = P[global[it2->second.r]], l = P[global[it2->second.l]];
        global[i] = (uint32_t)P.size();
        P.push_back(r + (l - r) * it2->second.s);
        parent.push_back(global[i]);
      }
      combineReport.crossingsMade += (long)tri.made().size();
      combineReport.detours += tri.detours, combineReport.edgeFallbacks += tri.fallbacks;
      std::vector<int> pieces = tri.triangles();
      for (size_t k = 0; k < pieces.size(); k += 3) {
        uint32_t v[3] = {rep(global[pieces[k]]), rep(global[pieces[k + 1]]), rep(global[pieces[k + 2]])};
        if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) continue;
        out.push_back({{v[0], v[1], v[2]}, t});
        for (int e = 0; e < 3; e++)
          if (tri.kept(pieces[k + e], pieces[k + (e + 1) % 3])) cuts.insert(pairKey(v[e], v[(e + 1) % 3]));
      }
    }
  }
};

double winding(const Welded &w, V3 q) {
  double sum = 0;
  for (size_t k = 0; k < w.tri.size(); k += 3) {
    V3 a = w.pts[w.tri[k]] - q, b = w.pts[w.tri[k + 1]] - q, c = w.pts[w.tri[k + 2]] - q;
    double la = norm(a), lb = norm(b), lc = norm(c);
    double det = dot(a, cross(b, c)), div = la * lb * lc + dot(a, b) * lc + dot(a, c) * lb + dot(b, c) * la;
    sum += 2 * std::atan2(det, div);
  }
  return sum / (4 * M_PI);
}

enum Side { Out, In, Same, Opposite };

// Each piece of X: outside or inside the other shape, or lying on it facing the same or the other way.
std::vector<Side> classify(Cutter &c, Part &X, const std::vector<Cutter::Piece> &pieces, const std::unordered_set<uint64_t> &cuts,
                           Part &Y, const Welded &other) {
  size_t n = pieces.size();
  std::vector<Side> side(n, Out);
  std::vector<int> fixed(n, 0);
  // Pieces lying on a triangle of the other shape.
  for (size_t i = 0; i < n; i++) {
    const Part::Cut &cutT = X.cut[pieces[i].from];
    if (cutT.partners.empty()) continue;
    uint32_t t = pieces[i].from;
    V3 xt[3] = {c.P[X.corner(t, 0)], c.P[X.corner(t, 1)], c.P[X.corner(t, 2)]};
    Flat f = flatOf(xt[0], xt[1], xt[2]);
    V3 nx = cross(xt[1] - xt[0], xt[2] - xt[0]);
    V3 mid = (c.P[pieces[i].v[0]] + c.P[pieces[i].v[1]] + c.P[pieces[i].v[2]]) / 3;
    for (uint32_t ty : cutT.partners) {
      V3 yt[3] = {c.P[Y.corner(ty, 0)], c.P[Y.corner(ty, 1)], c.P[Y.corner(ty, 2)]};
      V3 ny = cross(yt[1] - yt[0], yt[2] - yt[0]);
      if (orientFlat(f, yt[0], yt[1], yt[2]) < 0) std::swap(yt[1], yt[2]);
      if (where(f, yt, mid) == 0) {
        side[i] = dot(nx, ny) > 0 ? Same : Opposite;
        fixed[i] = 1;
        break;
      }
    }
  }
  // Regions between cuts (and between pieces on the other shape and off it), by flood.
  // Each piece's sides, sorted so the pieces sharing a side sit together; for each side, where its group starts and ends.
  std::vector<std::pair<uint64_t, uint32_t>> sides;
  sides.reserve(3 * n);
  for (size_t i = 0; i < n; i++)
    for (int e = 0; e < 3; e++) sides.push_back({pairKey(pieces[i].v[e], pieces[i].v[(e + 1) % 3]), (uint32_t)(3 * i + e)});
  std::sort(sides.begin(), sides.end());
  std::vector<uint32_t> groupOf(3 * n), groupEnd(sides.size());
  for (size_t k = 0, m; k < sides.size(); k = m) {
    for (m = k + 1; m < sides.size() && sides[m].first == sides[k].first;) m++;
    for (size_t q = k; q < m; q++) groupOf[sides[q].second] = (uint32_t)k, groupEnd[q] = (uint32_t)m;
  }
  std::vector<int> region(n, -1);
  std::vector<std::vector<uint32_t>> regions;
  for (size_t s = 0; s < n; s++) {
    if (region[s] >= 0) continue;
    int r = (int)regions.size();
    regions.push_back({(uint32_t)s});
    region[s] = r;
    for (size_t q = 0; q < regions[r].size(); q++) {
      uint32_t i = regions[r][q];
      for (int e = 0; e < 3; e++) {
        uint64_t key = pairKey(pieces[i].v[e], pieces[i].v[(e + 1) % 3]);
        if (cuts.count(key)) continue;
        uint32_t lo = groupOf[3 * i + e];
        for (uint32_t k = lo; k < groupEnd[lo]; k++) {
          uint32_t j = sides[k].second / 3;
          if (region[j] >= 0 || fixed[j] != fixed[i] || (fixed[i] && side[j] != side[i])) continue;
          region[j] = r;
          regions[r].push_back(j);
        }
      }
    }
  }
  for (auto &reg : regions) {
    if (fixed[reg[0]]) continue;
    // The largest pieces first: their middles are farthest from the other shape's surface.
    std::vector<std::pair<double, uint32_t>> bySize;
    for (uint32_t i : reg) {
      V3 a = c.P[pieces[i].v[0]], b = c.P[pieces[i].v[1]], d = c.P[pieces[i].v[2]];
      bySize.push_back({norm(cross(b - a, d - a)), i});
    }
    std::partial_sort(bySize.begin(), bySize.begin() + std::min<size_t>(4, bySize.size()), bySize.end(), std::greater<>());
    Side s = Out;
    bool sure = false;
    for (size_t k = 0; k < std::min<size_t>(4, bySize.size()) && !sure; k++) {
      uint32_t i = bySize[k].second;
      V3 mid = (c.P[pieces[i].v[0]] + c.P[pieces[i].v[1]] + c.P[pieces[i].v[2]]) / 3;
      double w = winding(other, mid);
      s = w > 0.5 ? In : Out;
      sure = std::fabs(w - 0.5) > 0.2;
    }
    if (!sure) combineReport.unsure++;
    for (uint32_t i : reg) side[i] = s;
  }
  return side;
}

// Points put on a grid finer than anything measured (about 10⁻¹² of the shapes' size), the same for both shapes: faces
// meant to lie on one another (a shape turned a quarter turn, whose sines and cosines come out a hair off) then do so
// exactly, and are merged by the rules for faces on faces rather than as two faces a hair apart.
Welded gridded(const Solid &s, double step) {
  Welded w = weld(s, step);
  // A triangle left with two corners in one place has no area: out (its sides cancel).
  size_t n = 0;
  for (size_t t = 0; t < w.count(); t++) {
    uint32_t *v = &w.tri[3 * t];
    if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) continue;
    for (int k = 0; k < 3; k++) w.tri[3 * n + k] = v[k], w.nrm[3 * n + k] = w.nrm[3 * t + k];
    if (!w.gap.empty())
      for (int k = 0; k < 6; k++) w.gap[6 * n + k] = w.gap[6 * t + k];
    w.face[n++] = w.face[t];
  }
  w.tri.resize(3 * n), w.nrm.resize(3 * n), w.face.resize(n);
  if (!w.gap.empty()) w.gap.resize(6 * n);
  return w;
}

}  // namespace

// Whether every side of every triangle is met by one running the other way (point numbers, not places).
static bool balanced(const Welded &w) {
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

Solid combine(const Solid &sa, const Solid &sb, int op, double merge, bool keepGrid) {
  combineReport.calls++;
  Cutter c;
  double step;
  {
    double scale = 1;
    for (const Solid *s : {&sa, &sb})
      for (V3 q : s->p) scale = std::max({scale, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
    step = std::ldexp(1.0, std::ilogb(scale) + 1 - 40);
    // A merge of a merge: on the grid it's on already (put on a coarser one, its points would round apart from where a
    // part made afresh rounds the same place).
    if (keepGrid)
      for (const Solid *s : {&sa, &sb})
        if (s->grid > 0) step = std::min(step, s->grid);
    c.A.w = gridded(sa, step), c.B.w = gridded(sb, step);
  }
  c.A.base = 0, c.B.base = (uint32_t)c.A.w.pts.size();
  c.P = c.A.w.pts;
  c.P.insert(c.P.end(), c.B.w.pts.begin(), c.B.w.pts.end());
  c.parent.resize(c.P.size());
  std::iota(c.parent.begin(), c.parent.end(), 0);
  for (Part *X : {&c.A, &c.B}) {
    X->edges.reserve(3 * X->w.count());
    for (uint32_t t = 0; t < X->w.count(); t++)
      for (int k = 0; k < 3; k++) X->edges.push_back(pairKey(X->corner(t, k), X->corner(t, (k + 1) % 3)));
    std::sort(X->edges.begin(), X->edges.end());
    X->edges.erase(std::unique(X->edges.begin(), X->edges.end()), X->edges.end());
    X->onEdge.resize(X->edges.size());
    X->cut.resize(X->w.count());
  }
  // Pairs of triangles whose boxes meet.
  std::vector<Box> boxesB(c.B.w.count());
  for (uint32_t t = 0; t < c.B.w.count(); t++) {
    V3 a = c.P[c.B.corner(t, 0)], b = c.P[c.B.corner(t, 1)], d = c.P[c.B.corner(t, 2)];
    boxesB[t] = {vmin(a, vmin(b, d)), vmax(a, vmax(b, d))};
  }
  Boxes tree(boxesB);
  for (uint32_t t = 0; t < c.A.w.count(); t++) {
    V3 a = c.P[c.A.corner(t, 0)], b = c.P[c.A.corner(t, 1)], d = c.P[c.A.corner(t, 2)];
    tree.overlapping({vmin(a, vmin(b, d)), vmax(a, vmax(b, d))}, [&](uint32_t u) { c.pairOf(t, u); });
  }
  // Crossing points made a hair apart by different roads (an edge through a triangle beside another edge through a
  // triangle, at what is one place) are one point: the triangulations would otherwise have to keep them apart at the
  // scale of rounding, which they can't. A hair is `merge` of the shapes' size: 10⁻¹¹ by default; where a rounding's
  // tool meets a face along the line it touches, its edges lie almost in the face, and where they cross comes out as
  // loosely as 10⁻⁹.
  {
    size_t made = c.A.w.pts.size() + c.B.w.pts.size();
    if (c.P.size() > made) {
      double scale = 0;
      for (V3 q : c.P) scale = std::max({scale, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
      double tol = merge * (1 + scale);
      using Cell = std::array<int64_t, 3>;
      auto cell = [&](V3 q) { return Cell{(int64_t)std::floor(q.x / tol), (int64_t)std::floor(q.y / tol), (int64_t)std::floor(q.z / tol)}; };
      std::vector<std::pair<Cell, uint32_t>> grid;
      grid.reserve(c.P.size());
      for (uint32_t i = 0; i < c.P.size(); i++)
        if (c.rep(i) == i) grid.push_back({cell(c.P[i]), i});
      std::sort(grid.begin(), grid.end());
      for (uint32_t i = (uint32_t)made; i < c.P.size(); i++) {
        if (c.rep(i) != i) continue;
        Cell k = cell(c.P[i]);
        for (int dx = -1; dx <= 1; dx++)
          for (int dy = -1; dy <= 1; dy++)
            for (int dz = -1; dz <= 1; dz++) {
              Cell near{k[0] + dx, k[1] + dy, k[2] + dz};
              auto it = std::lower_bound(grid.begin(), grid.end(), std::make_pair(near, (uint32_t)0));
              for (; it != grid.end() && it->first == near; ++it)
                if (c.rep(it->second) != c.rep(i) && norm2(c.P[i] - c.P[it->second]) <= tol * tol) c.unite(i, it->second);
            }
      }
    }
  }
  std::vector<Cutter::Piece> pa, pb;
  std::unordered_set<uint64_t> cuts;
  c.remake(c.A, pa, cuts);
  c.remake(c.B, pb, cuts);
  // Points found to be one after a piece was made: pieces left with two corners in one place have no area and go.
  for (auto *ps : {&pa, &pb}) {
    size_t n = 0;
    for (auto &x : *ps) {
      for (auto &v : x.v) v = c.rep(v);
      if (x.v[0] != x.v[1] && x.v[1] != x.v[2] && x.v[0] != x.v[2]) (*ps)[n++] = x;
    }
    ps->resize(n);
  }
  {
    std::unordered_set<uint64_t> fixedCuts;
    for (uint64_t k : cuts) fixedCuts.insert(pairKey(c.rep((uint32_t)(k >> 32)), c.rep((uint32_t)(k & 0xffffffffu))));
    cuts.swap(fixedCuts);
  }
  std::vector<Side> sideA = classify(c, c.A, pa, cuts, c.B, c.B.w), sideB = classify(c, c.B, pb, cuts, c.A, c.A.w);

  bool uni = op == BK_UNION, sub = op == BK_SUBTRACT, inter = op == BK_INTERSECT;
  auto keepA = [&](Side s) { return s == Out ? (uni || sub) : s == In ? inter : s == Same ? (uni || inter) : sub; };
  auto keepB = [&](Side s) { return s == Out ? uni : s == In ? (sub || inter) : false; };

  Welded out;
  out.pts = c.P;
  Solid result;
  size_t nfa = sa.faces.size();
  result.faces = sa.faces;
  result.faces.insert(result.faces.end(), sb.faces.begin(), sb.faces.end());
  if (sub)
    for (size_t f = nfa; f < result.faces.size(); f++) {
      auto &face = result.faces[f];
      face.normal = -face.normal, face.deficit = -face.deficit;
      if (face.geom.flat) face.geom.pn = -face.geom.pn, face.geom.pd = -face.geom.pd;
    }
  // A piece's corner normals, from its triangle's by where each corner lies in it.
  auto normals = [&](Part &X, uint32_t t, const uint32_t v[3], V3 out3[3]) {
    V3 a = c.P[X.corner(t, 0)], b = c.P[X.corner(t, 1)], d = c.P[X.corner(t, 2)];
    V3 n0 = X.w.nrm[3 * t], n1 = X.w.nrm[3 * t + 1], n2 = X.w.nrm[3 * t + 2];
    V3 nn = cross(b - a, d - a);
    double whole = dot(nn, nn);
    for (int k = 0; k < 3; k++) {
      V3 p = c.P[v[k]];
      double l0 = whole > 0 ? dot(cross(b - p, d - p), nn) / whole : 1.0 / 3, l1 = whole > 0 ? dot(cross(d - p, a - p), nn) / whole : 1.0 / 3;
      out3[k] = unit(n0 * l0 + n1 * l1 + n2 * (1 - l0 - l1));
    }
  };
  // A piece's share of its triangle's slivers.
  auto gapOf = [&](Part &X, uint32_t t, const uint32_t v[3], double g[6]) {
    if (X.w.gap.empty()) {
      std::fill(g, g + 6, 0.0);
      return;
    }
    V3 q[3] = {c.P[v[0]], c.P[v[1]], c.P[v[2]]};
    gapOfPiece(&X.w.gap[6 * t], c.P[X.corner(t, 0)], c.P[X.corner(t, 1)], c.P[X.corner(t, 2)], q, g);
  };
  for (size_t i = 0; i < pa.size(); i++) {
    if (!keepA(sideA[i])) continue;
    V3 nv[3];
    double g[6];
    normals(c.A, pa[i].from, pa[i].v, nv);
    gapOf(c.A, pa[i].from, pa[i].v, g);
    out.tri.insert(out.tri.end(), {pa[i].v[0], pa[i].v[1], pa[i].v[2]});
    out.nrm.insert(out.nrm.end(), {nv[0], nv[1], nv[2]});
    out.gap.insert(out.gap.end(), g, g + 6);
    out.face.push_back(c.A.w.face[pa[i].from]);
  }
  for (size_t i = 0; i < pb.size(); i++) {
    if (!keepB(sideB[i])) continue;
    V3 nv[3];
    double g[6];
    normals(c.B, pb[i].from, pb[i].v, nv);
    gapOf(c.B, pb[i].from, pb[i].v, g);
    uint32_t f = (uint32_t)nfa + c.B.w.face[pb[i].from];
    if (sub) {
      // Turned round, and what was beyond its surface now lies within the result: corners 0, 2, 1.
      out.tri.insert(out.tri.end(), {pb[i].v[0], pb[i].v[2], pb[i].v[1]});
      out.nrm.insert(out.nrm.end(), {-nv[0], -nv[2], -nv[1]});
      out.gap.insert(out.gap.end(), {-g[0], -g[2], -g[1], -g[5], -g[4], -g[3]});
    } else {
      out.tri.insert(out.tri.end(), {pb[i].v[0], pb[i].v[1], pb[i].v[2]});
      out.nrm.insert(out.nrm.end(), {nv[0], nv[1], nv[2]});
      out.gap.insert(out.gap.end(), g, g + 6);
    }
    out.face.push_back(f);
  }
  // Points a rounding apart made one, and pieces left with no area at all (points on one cut) swapped away, so the next
  // merge or cut never meets them.
  {
    double scale = 1;
    for (V3 q : c.P) scale = std::max({scale, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
    tidy(out, 1e-9 * scale);
    unneedle(out, 1e-9 * scale);
  }
  if (!balanced(out)) combineReport.open++;
  unweld(out, result);
  result.grid = step;
  return result;
}

}  // namespace bce
