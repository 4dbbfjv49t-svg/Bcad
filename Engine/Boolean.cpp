// Merge, subtract and intersect on meshes (two shapes, or one shape with many taken away and added at once). The shapes'
// points first go on one fine grid (so faces meant to lie on each other do, exactly; triangles left with no area there
// go), and their triangles are taken together. Every triangle is cut where any other crosses it — another shape's, or
// its own shape's where that meets or folds over itself — and every point made there is kept exactly, as the edge and
// plane (or three planes) it lies on, never rounded until the result is written out; every decision about the points
// (which side of a plane or a line, which comes first along an edge) is exact (Implicit.hpp). A point made once is used
// by every triangle it touches, and every point on a cut is put in both triangles the cut runs through, so the pieces
// meet edge to edge. Each region the cuts leave is then told inside or outside by counting, exactly, the triangles of
// each shape a ray from it crosses (its winding numbers), on either side of it; it's kept where the operation's answer
// differs from one side to the other, facing the way that's inside. So the result is closed by how it's made, whatever
// the shapes, and faces lying on each other follow from the same rule (a merge keeps one of two faces facing the same
// way and drops two facing each other; a subtract leaves no skin). Every piece kept takes its share of its triangle's
// slivers (Solid::gap).
//
// A result's triangles are marked as known not to cross (Solid::sound), so a shape merged with one part after another is
// looked at against itself only where it changed. Written out, points a rounding apart are made one and triangles thin
// past telling apart put away (clean, in Csg.cpp). Should the marks no longer hold (two triangles' corners rounded into
// each other since) or a shape meet itself farther off than was looked, the result shows it (not closed, or cuts
// crossing with no point made for them) and the merge is made again looking at every pair.
#include "Engine/Implicit.hpp"
#include "Engine/Model.hpp"
#include "Engine/Triangulate.hpp"
#include "Engine/Weld.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace bce {

thread_local CombineReport combineReport;

namespace {

uint64_t pairKey(uint32_t a, uint32_t b) { return (uint64_t)std::min(a, b) << 32 | std::max(a, b); }

// A triangle's plane seen flat: along the axis its normal lies most along, the other two coordinates in an order that
// keeps the triangle counter-clockwise.
struct Flat {
  int axis;
  bool swap;
};

// MARK: - the shapes' triangles

struct Soup {
  Welded w;  // each shape's triangles in turn; a point two have is one
  // Shape k's triangles are those from start[k] (one more, past the last, the count), and each triangle's shape.
  std::vector<uint32_t> start, owner;
  std::vector<uint64_t> edges;                // edges as (point, point), sorted: an edge's number is its place
  std::vector<std::vector<uint32_t>> onEdge;  // points to put on each edge
  // A cut in a triangle: from p to q, where triangle `twin` crosses it, or (twin none) along edge u–v lying in its plane.
  struct Seg {
    uint32_t p, q, twin, u, v;
  };
  struct Cut {
    std::vector<uint32_t> in;
    std::vector<Seg> segs;
    std::vector<uint32_t> partners;  // triangles lying in its plane over it
    bool any() const { return !in.empty() || !segs.empty() || !partners.empty(); }
  };
  std::vector<Cut> cut;  // per triangle

  int shapes() const { return (int)start.size() - 1; }
  int shape(uint32_t t) const { return (int)owner[t]; }
  uint32_t corner(uint32_t t, int k) const { return w.tri[3 * t + k]; }
  uint32_t edgeOf(uint32_t a, uint32_t b) const { return (uint32_t)(std::lower_bound(edges.begin(), edges.end(), pairKey(a, b)) - edges.begin()); }
};

struct Box {
  V3 lo, hi;
};

// A tree of boxes over the triangles, for finding those near a triangle, or along a ray.
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
      double cu = boxes[u].lo[axis] + boxes[u].hi[axis], cv = boxes[v].lo[axis] + boxes[v].hi[axis];
      return cu < cv || (cu == cv && u < v);
    });
    int l = make(first, half), r = make(first + half, count - half);
    nodes[at].left = l, nodes[at].right = r;
    return at;
  }
  // Every box `meets` says may hold what's looked for (a tree split in halves is never deeper than the stack).
  template <typename M, typename F> void each(M meets, F f) const {
    if (nodes.empty()) return;
    int stack[128], top = 0;
    stack[top++] = 0;
    while (top) {
      const Node &n = nodes[stack[--top]];
      if (!meets(n.b)) continue;
      if (n.left < 0) {
        for (int i = n.first; i < n.first + n.count; i++)
          if (meets(boxes[order[i]])) f(order[i]);
      } else {
        stack[top++] = n.left, stack[top++] = n.right;
      }
    }
  }
  template <typename F> void overlapping(const Box &q, F f) const {
    each([&](const Box &b) { return !(b.lo.x > q.hi.x || b.hi.x < q.lo.x || b.lo.y > q.hi.y || b.hi.y < q.lo.y || b.lo.z > q.hi.z || b.hi.z < q.lo.z); }, f);
  }
  // Boxes the ray from o along d passes through or within `margin` of.
  template <typename F> void along(V3 o, V3 d, double margin, F f) const {
    each(
        [&](const Box &b) {
          double t0 = 0, t1 = INFINITY;
          for (int k = 0; k < 3; k++) {
            double lo = b.lo[k] - margin, hi = b.hi[k] + margin;
            if (d[k] == 0) {
              if (o[k] < lo || o[k] > hi) return false;
              continue;
            }
            double a = (lo - o[k]) / d[k], c = (hi - o[k]) / d[k];
            t0 = std::max(t0, std::min(a, c)), t1 = std::min(t1, std::max(a, c));
          }
          return t0 <= t1;
        },
        f);
  }
};

struct KeyHash {
  size_t operator()(const std::array<uint32_t, 3> &k) const { return (size_t)k[0] * 0x9E3779B97F4A7C15ull ^ (size_t)k[1] * 0xC2B2AE3D27D4EB4Full ^ k[2]; }
};

struct Cutter {
  Soup S;
  std::vector<std::pair<uint32_t, uint32_t>> flats;  // pairs of triangles lying in one plane
  ExactPoints E;                 // every point: the shapes', then the ones made
  std::vector<V3> &P = E.at;     // (where each lies, to a rounding for made ones)
  std::vector<uint32_t> parent;  // points found to be one
  std::unordered_map<std::array<uint32_t, 3>, uint32_t, KeyHash> made;
  std::map<std::array<uint32_t, 3>, uint32_t> crossed;
  // Cuts lying flat already put in a triangle (by triangle, ends, and the edge they run along).
  struct SegHash {
    size_t operator()(const std::array<uint32_t, 4> &k) const {
      return (size_t)k[0] * 0x9E3779B97F4A7C15ull ^ (size_t)k[1] * 0xC2B2AE3D27D4EB4Full ^ (size_t)k[2] * 0x165667B19E3779F9ull ^ k[3];
    }
  };
  std::unordered_set<std::array<uint32_t, 4>, SegHash> flatSegs;
  // Per triangle, how many of its cuts were looked at for crossings, and what it held when its points were last spread.
  std::vector<uint32_t> crossChecked;
  std::vector<size_t> spreadSeen;
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
    bool has(uint32_t q) const { return q < at.size() && at[q] >= 0; }
    int get(uint32_t q) const { return at[q]; }
    void set(uint32_t q, int i) {
      if (q >= at.size()) at.resize(q + 1, -1);
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
    if (a == b) return;
    uint32_t lo = std::min(a, b), hi = std::max(a, b);
    parent[hi] = lo;
    // In the triangulation being made, the point keeps its number.
    if (!local.has(lo) && local.has(hi)) local.set(lo, local.get(hi));
  }
  template <typename F> uint32_t point(std::array<uint32_t, 3> key, const F &make) {
    auto it = made.find(key);
    if (it != made.end()) return it->second;
    uint32_t id = make();
    parent.push_back(id);
    made[key] = id;
    return id;
  }
  // Kinds of point made: an edge through a triangle, two edges across each other.
  enum { EdgeTri = 1, EdgeEdge = 2 };

  Flat flatOf(uint32_t t) const {
    Flat f;
    E.flat(S.corner(t, 0), S.corner(t, 1), S.corner(t, 2), f.axis, f.swap);
    return f;
  }
  int orientFlat(const Flat &f, uint32_t a, uint32_t b, uint32_t c) const { return E.orient2d(f.axis, f.swap, a, b, c); }
  // Where p (in the plane of triangle t, counter-clockwise in f) lies: -1 outside, 0 inside, 1 + m on side m (corner m
  // to m + 1), 4 + m on corner m.
  int where(const Flat &f, const uint32_t t[3], uint32_t p) const {
    // Outside the triangle's box by more than p may be off: outside it.
    V3 q = P[p];
    double e = 1.0001 * E.error(p);
    for (int k = 0; k < 3; k++) {
      double lo = std::min({P[t[0]][k], P[t[1]][k], P[t[2]][k]}), hi = std::max({P[t[0]][k], P[t[1]][k], P[t[2]][k]});
      if (q[k] < lo - e || q[k] > hi + e) return -1;
    }
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
  // Triangle t's corners counter-clockwise as f sees them.
  void cornersIn(const Flat &f, uint32_t t, uint32_t g[3]) const {
    for (int k = 0; k < 3; k++) g[k] = S.corner(t, k);
    if (orientFlat(f, g[0], g[1], g[2]) < 0) std::swap(g[1], g[2]);
  }
  // Points on one line, in order along it (one way or the other); one place twice, made by different roads, is made one
  // point.
  void inLine(std::vector<uint32_t> &pts) {
    for (auto &q : pts) q = rep(q);
    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    std::sort(pts.begin(), pts.end(), [&](uint32_t a, uint32_t b) {
      int c = E.lex(a, b);
      return c < 0 || (c == 0 && a < b);
    });
    size_t n = 0;
    for (uint32_t q : pts) {
      if (n && E.lex(pts[n - 1], q) == 0) {
        unite(pts[n - 1], q);
        pts[n - 1] = rep(q);
        continue;
      }
      pts[n++] = q;
    }
    pts.resize(n);
  }
  // A plane other than the triangle's own that a cut in it lies in: the twin's, or one standing on the edge it runs along.
  Plane3 carrier(const Soup::Seg &s, const Flat &f) const {
    if (s.twin != UINT32_MAX) return {S.corner(s.twin, 0), S.corner(s.twin, 1), S.corner(s.twin, 2), -1};
    return {s.u, s.v, s.u, f.axis};
  }
  // What a cut is made with, as one number.
  uint32_t carrierCode(const Soup::Seg &s) const { return s.twin != UINT32_MAX ? s.twin : 0x80000000u | S.edgeOf(s.u, s.v); }

  void addOnEdge(uint32_t a, uint32_t b, uint32_t p) { S.onEdge[S.edgeOf(a, b)].push_back(p); }
  void addIn(uint32_t t, uint32_t p) { S.cut[t].in.push_back(p); }
  // Point q put in triangle t (inside, on a side, or one with a corner), where it isn't already: whether it was new.
  bool putIn(uint32_t t, uint32_t q) {
    uint32_t g[3] = {S.corner(t, 0), S.corner(t, 1), S.corner(t, 2)};
    int at = where(flatOf(t), g, q);
    if (at < 0) return combineReport.misses++, false;
    if (at >= 4) {
      if (rep(q) == rep(g[at - 4])) return false;
      unite(q, g[at - 4]);
      return true;
    }
    auto &list = at == 0 ? S.cut[t].in : S.onEdge[S.edgeOf(g[at - 1], g[at % 3])];
    for (uint32_t r : list)
      if (rep(r) == rep(q)) return false;
    list.push_back(q);
    return true;
  }
  bool putOnEdge(uint32_t u, uint32_t v, uint32_t q) {
    if (rep(q) == rep(u) || rep(q) == rep(v)) return false;
    auto &list = S.onEdge[S.edgeOf(u, v)];
    for (uint32_t r : list)
      if (rep(r) == rep(q)) return false;
    list.push_back(q);
    return true;
  }

  // Triangle tx's edges through triangle ty (its corners on ty's plane included), oX its corners' sides of ty's plane.
  // Points found are added to `found`.
  void edgesThrough(uint32_t tx, uint32_t ty, const int oX[3], std::vector<uint32_t> &found) {
    uint32_t gy[3], gx[3];
    for (int k = 0; k < 3; k++) gy[k] = S.corner(ty, k), gx[k] = S.corner(tx, k);
    Flat fy = flatOf(ty);
    for (int k = 0; k < 3; k++) {
      if (oX[k] != 0) continue;
      int at = where(fy, gy, gx[k]);
      if (at < 0) continue;
      if (at == 0) addIn(ty, gx[k]);
      else if (at <= 3) addOnEdge(gy[at - 1], gy[at % 3], gx[k]);
      else unite(gx[k], gy[at - 4]);
      found.push_back(gx[k]);
    }
    for (int k = 0; k < 3; k++) {
      uint32_t u = gx[k], v = gx[(k + 1) % 3];
      int su = oX[k], sv = oX[(k + 1) % 3];
      if (su == 0 && sv == 0) {
        // The edge lies in ty's plane: where it crosses ty's edges, in that plane (on a plane standing on ty's edge).
        for (int m = 0; m < 3; m++) {
          uint32_t c = gy[m], d = gy[(m + 1) % 3];
          if (c == u || c == v || d == u || d == v) continue;
          int o1 = orientFlat(fy, u, v, c), o2 = orientFlat(fy, u, v, d);
          int o3 = orientFlat(fy, c, d, u), o4 = orientFlat(fy, c, d, v);
          if (o1 * o2 < 0 && o3 * o4 < 0) {
            uint32_t ex = S.edgeOf(u, v), ey = S.edgeOf(c, d);
            uint32_t q = point({EdgeEdge, std::min(ex, ey), std::max(ex, ey)}, [&] { return E.line(u, v, {c, d, c, fy.axis}); });
            addOnEdge(u, v, q), addOnEdge(c, d, q);
            found.push_back(q);
          }
        }
        continue;
      }
      if (su * sv >= 0) continue;
      // Through ty's plane: inside ty, across one of its edges, or through one of its corners.
      int t[3], zeros = 0, pos = 0, neg = 0;
      for (int m = 0; m < 3; m++) {
        t[m] = orient3d(P[u], P[v], P[gy[m]], P[gy[(m + 1) % 3]]);
        zeros += t[m] == 0, pos += t[m] > 0, neg += t[m] < 0;
      }
      if (pos && neg) continue;
      uint32_t ex = S.edgeOf(u, v);
      if (zeros == 0) {
        uint32_t q = point({EdgeTri, ex, ty}, [&] { return E.line(u, v, {gy[0], gy[1], gy[2]}); });
        addOnEdge(u, v, q), addIn(ty, q);
        found.push_back(q);
      } else if (zeros == 1) {
        int m = t[0] == 0 ? 0 : t[1] == 0 ? 1 : 2;
        uint32_t c = gy[m], d = gy[(m + 1) % 3], ey = S.edgeOf(c, d);
        // (On ty's edge where it crosses ty's plane.)
        uint32_t q = point({EdgeEdge, std::min(ex, ey), std::max(ex, ey)}, [&] { return E.line(u, v, {gy[0], gy[1], gy[2]}); });
        addOnEdge(u, v, q), addOnEdge(c, d, q);
        found.push_back(q);
      } else {
        // Through a corner of ty: the corner itself, on tx's edge.
        int m = 0;
        for (int i = 0; i < 3; i++)
          if (t[i] == 0 && t[(i + 2) % 3] == 0) m = i;
        addOnEdge(u, v, gy[m]);
        found.push_back(gy[m]);
      }
    }
  }

  // Two triangles in one plane: each one's corners inside the other, their edges' crossings, and each one's edges as cuts in
  // the other.
  void flatPair(uint32_t t1, uint32_t t2) {
    Flat f = flatOf(t1);
    uint32_t g1[3], g2[3];
    cornersIn(f, t1, g1), cornersIn(f, t2, g2);
    S.cut[t1].partners.push_back(t2);
    S.cut[t2].partners.push_back(t1);
    for (int k = 0; k < 3; k++) {
      int at = where(f, g1, g2[k]);
      if (at == 0) addIn(t1, g2[k]);
      else if (at >= 1 && at <= 3) addOnEdge(g1[at - 1], g1[at % 3], g2[k]);
      else if (at >= 4) unite(g2[k], g1[at - 4]);
      at = where(f, g2, g1[k]);
      if (at == 0) addIn(t2, g1[k]);
      else if (at >= 1 && at <= 3) addOnEdge(g2[at - 1], g2[at % 3], g1[k]);
      else if (at >= 4) unite(g1[k], g2[at - 4]);
    }
    // Proper crossings of the edges.
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        uint32_t u = g1[i], v = g1[(i + 1) % 3], c = g2[j], d = g2[(j + 1) % 3];
        if (c == u || c == v || d == u || d == v) continue;
        int o1 = orientFlat(f, u, v, c), o2 = orientFlat(f, u, v, d);
        int o3 = orientFlat(f, c, d, u), o4 = orientFlat(f, c, d, v);
        if (!(o1 * o2 < 0 && o3 * o4 < 0)) continue;
        uint32_t e1 = S.edgeOf(u, v), e2 = S.edgeOf(c, d);
        uint32_t q = point({EdgeEdge, std::min(e1, e2), std::max(e1, e2)}, [&] { return E.line(u, v, {c, d, c, f.axis}); });
        addOnEdge(u, v, q), addOnEdge(c, d, q);
      }
    // Each one's edges inside the other become cuts there: done once every pair has put its points on the edges.
    flats.push_back({t1, t2});
  }

  // Two triangles in one plane (after every pair is done, so each edge's points are all there): each one's edges inside the
  // other become cuts there, the edge's points in order, joined where the piece between them lies inside; every point of
  // such a cut is put in the other triangle too (inside, on a side, or one with a corner), so the cut is never lost there.
  // Whether any point was put anywhere new.
  bool flatEdges(uint32_t t1, uint32_t t2) {
    Flat f = flatOf(t1);
    uint32_t g1[3], g2[3];
    cornersIn(f, t1, g1), cornersIn(f, t2, g2);
    bool grew = false;
    auto edgesInto = [&](const uint32_t gx[3], uint32_t ty, const uint32_t gy[3]) {
      // A point of the cut put in ty (where it isn't already).
      auto put = [&](uint32_t q, int at) {
        if (at == 0) {
          for (uint32_t r : S.cut[ty].in)
            if (rep(r) == rep(q)) return;
          addIn(ty, q), grew = true;
        } else if (at <= 3) {
          grew = putOnEdge(gy[at - 1], gy[at % 3], q) || grew;
        } else if (rep(q) != rep(gy[at - 4])) {
          unite(q, gy[at - 4]), grew = true;
        }
      };
      for (int k = 0; k < 3; k++) {
        uint32_t u = gx[k], v = gx[(k + 1) % 3];
        std::vector<uint32_t> pts{u, v};
        for (uint32_t q : S.onEdge[S.edgeOf(u, v)]) pts.push_back(q);
        inLine(pts);
        for (size_t i = 0; i + 1 < pts.size(); i++) {
          // The piece between lies inside where both its ends do (the triangle is convex).
          int w0 = where(f, gy, pts[i]), w1 = where(f, gy, pts[i + 1]);
          if (w0 < 0 || w1 < 0) continue;
          put(pts[i], w0), put(pts[i + 1], w1);
          if (flatSegs.insert({ty, pts[i], pts[i + 1], S.edgeOf(u, v)}).second) S.cut[ty].segs.push_back({pts[i], pts[i + 1], UINT32_MAX, u, v});
        }
      }
    };
    edgesInto(g2, t1, g1);
    edgesInto(g1, t2, g2);
    return grew;
  }

  // Two triangles in one plane that don't overlap: only each one's corners on the other's sides (so sides running along
  // each other are cut at the same points).
  void touching(uint32_t t1, uint32_t t2) {
    Flat f = flatOf(t1);
    uint32_t g1[3], g2[3];
    cornersIn(f, t1, g1), cornersIn(f, t2, g2);
    for (int k = 0; k < 3; k++) {
      int at = where(f, g1, g2[k]);
      if (at >= 1 && at <= 3) addOnEdge(g1[at - 1], g1[at % 3], g2[k]);
      at = where(f, g2, g1[k]);
      if (at >= 1 && at <= 3) addOnEdge(g2[at - 1], g2[at % 3], g1[k]);
    }
  }
  // Whether two triangles in one plane overlap (more than along their sides): no side of either has the other wholly
  // beyond it or on it.
  bool overlap(uint32_t t1, uint32_t t2) {
    Flat f = flatOf(t1);
    uint32_t g[2][3];
    cornersIn(f, t1, g[0]), cornersIn(f, t2, g[1]);
    for (int i = 0; i < 2; i++)
      for (int k = 0; k < 3; k++) {
        bool apart = true;
        for (int m = 0; m < 3 && apart; m++) apart = orientFlat(f, g[i][k], g[i][(k + 1) % 3], g[1 - i][m]) <= 0;
        if (apart) return false;
      }
    return true;
  }

  // Two triangles (any two, of either shape) where they meet.
  void pairOf(uint32_t t1, uint32_t t2) {
    uint32_t g1[3], g2[3];
    for (int k = 0; k < 3; k++) g1[k] = S.corner(t1, k), g2[k] = S.corner(t2, k);
    int shared = 0;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) shared += g1[i] == g2[j];
    if (shared == 3) {
      // On the same three points: the same triangle, or one facing the other.
      flatPair(t1, t2);
      return;
    }
    if (shared == 2) {
      // Sharing a side, they meet along it alone, unless in one plane and folded back across it.
      uint32_t c1 = 0, c2 = 0;
      for (int k = 0; k < 3; k++) {
        if (g1[k] != g2[0] && g1[k] != g2[1] && g1[k] != g2[2]) c1 = g1[k];
        if (g2[k] != g1[0] && g2[k] != g1[1] && g2[k] != g1[2]) c2 = g2[k];
      }
      if (orient3d(P[g1[0]], P[g1[1]], P[g1[2]], P[c2]) != 0) return;
      Flat f = flatOf(t1);
      uint32_t u = 0, v = 0;
      for (int k = 0; k < 3; k++)
        if (g1[(k + 2) % 3] == c1) u = g1[k], v = g1[(k + 1) % 3];
      if (orientFlat(f, u, v, c1) * orientFlat(f, u, v, c2) < 0) return;
      flatPair(t1, t2);
      return;
    }
    int o2[3], o1[3];
    for (int k = 0; k < 3; k++) o2[k] = orient3d(P[g1[0]], P[g1[1]], P[g1[2]], P[g2[k]]);
    if ((o2[0] >= 0 && o2[1] >= 0 && o2[2] >= 0 && shared + (o2[0] > 0) + (o2[1] > 0) + (o2[2] > 0) == 3) ||
        (o2[0] <= 0 && o2[1] <= 0 && o2[2] <= 0 && shared + (o2[0] < 0) + (o2[1] < 0) + (o2[2] < 0) == 3))
      return;
    if (o2[0] == 0 && o2[1] == 0 && o2[2] == 0) {
      if (overlap(t1, t2)) flatPair(t1, t2);
      else touching(t1, t2);
      return;
    }
    for (int k = 0; k < 3; k++) o1[k] = orient3d(P[g2[0]], P[g2[1]], P[g2[2]], P[g1[k]]);
    if ((o1[0] >= 0 && o1[1] >= 0 && o1[2] >= 0 && shared + (o1[0] > 0) + (o1[1] > 0) + (o1[2] > 0) == 3) ||
        (o1[0] <= 0 && o1[1] <= 0 && o1[2] <= 0 && shared + (o1[0] < 0) + (o1[1] < 0) + (o1[2] < 0) == 3))
      return;
    std::vector<uint32_t> &found = scratch;
    found.clear();
    edgesThrough(t1, t2, o1, found);
    edgesThrough(t2, t1, o2, found);
    // All on the line where the planes meet: in order along it, each next two joined.
    inLine(found);
    for (size_t i = 0; i + 1 < found.size(); i++) {
      S.cut[t1].segs.push_back({found[i], found[i + 1], t2, 0, 0});
      S.cut[t2].segs.push_back({found[i], found[i + 1], t1, 0, 0});
    }
  }

  // Cuts crossing inside a triangle (where a shape meets itself, or folds over): the point where they cross, made once
  // (as where the three planes meet) and put in the triangle, so neither cut is broken there.
  bool crossings() {
    bool grew = false;
    crossChecked.resize(S.w.count(), 0);
    std::vector<Box> box;
    for (uint32_t t = 0; t < S.w.count(); t++) {
      const auto &segs = S.cut[t].segs;
      size_t from = crossChecked[t];
      crossChecked[t] = (uint32_t)segs.size();
      if (segs.size() < 2 || from == segs.size()) continue;
      Flat f = flatOf(t);
      // Each cut's box, a hair larger than its ends' places may be off by.
      box.resize(segs.size());
      for (size_t i = 0; i < segs.size(); i++) {
        V3 a = P[segs[i].p], b = P[segs[i].q];
        double e = 1e-12 * (1 + std::max({std::fabs(a.x), std::fabs(a.y), std::fabs(a.z), std::fabs(b.x), std::fabs(b.y), std::fabs(b.z)}));
        box[i] = {vmin(a, b) - V3{e, e, e}, vmax(a, b) + V3{e, e, e}};
      }
      for (size_t j = std::max<size_t>(from, 1); j < segs.size(); j++)
        for (size_t i = 0; i < j; i++) {
          const Box &a = box[i], &b = box[j];
          if (a.lo.x > b.hi.x || b.lo.x > a.hi.x || a.lo.y > b.hi.y || b.lo.y > a.hi.y || a.lo.z > b.hi.z || b.lo.z > a.hi.z) continue;
          uint32_t ci = carrierCode(segs[i]), cj = carrierCode(segs[j]);
          if (ci == cj) continue;
          uint32_t p1 = rep(segs[i].p), q1 = rep(segs[i].q), p2 = rep(segs[j].p), q2 = rep(segs[j].q);
          if (p1 == p2 || p1 == q2 || q1 == p2 || q1 == q2) continue;
          if (orientFlat(f, p1, q1, p2) * orientFlat(f, p1, q1, q2) >= 0 || orientFlat(f, p2, q2, p1) * orientFlat(f, p2, q2, q1) >= 0) continue;
          std::array<uint32_t, 3> key{t, std::min(ci, cj), std::max(ci, cj)};
          auto it = crossed.find(key);
          uint32_t q;
          if (it != crossed.end()) {
            q = it->second;
          } else {
            q = E.planes({S.corner(t, 0), S.corner(t, 1), S.corner(t, 2)}, carrier(segs[i], f), carrier(segs[j], f));
            parent.push_back(q);
            crossed[key] = q;
            combineReport.crossingsMade++;
          }
          bool known = false;
          for (uint32_t r : S.cut[t].in) known = known || rep(r) == rep(q);
          if (!known) S.cut[t].in.push_back(q), grew = true;
        }
    }
    return grew;
  }

  // Every point of a triangle lying on one of its cuts (strictly between the cut's ends) lies in the triangle the cut is
  // made with, or on the edge it runs along: put there too, so both are cut at the same points. (Only where cuts in one
  // triangle run along each other or cross: where a shape folds over or meets itself.)
  bool spread() {
    bool grew = false;
    std::vector<uint32_t> pts;
    spreadSeen.resize(S.w.count(), 0);
    for (uint32_t t = 0; t < S.w.count(); t++) {
      if (S.cut[t].segs.empty()) continue;
      uint32_t g[3] = {S.corner(t, 0), S.corner(t, 1), S.corner(t, 2)};
      // (Nothing new in it since last time: nothing new to spread.)
      size_t seen = S.cut[t].in.size() + S.cut[t].segs.size();
      for (int k = 0; k < 3; k++) seen += S.onEdge[S.edgeOf(g[k], g[(k + 1) % 3])].size();
      if (seen == spreadSeen[t]) continue;
      spreadSeen[t] = seen;
      pts.assign(S.cut[t].in.begin(), S.cut[t].in.end());
      for (int k = 0; k < 3; k++) {
        pts.push_back(g[k]);
        for (uint32_t q : S.onEdge[S.edgeOf(g[k], g[(k + 1) % 3])]) pts.push_back(q);
      }
      for (auto &q : pts) q = rep(q);
      std::sort(pts.begin(), pts.end());
      pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
      if (pts.size() <= 3) continue;
      Flat f = flatOf(t);
      for (size_t i = 0; i < S.cut[t].segs.size(); i++) {
        Soup::Seg s = S.cut[t].segs[i];
        uint32_t p = rep(s.p), q = rep(s.q);
        V3 lo = vmin(P[p], P[q]), hi = vmax(P[p], P[q]);
        double e = 1.0001 * (E.error(p) + E.error(q));
        for (uint32_t m : pts) {
          if (m == p || m == q) continue;
          // (Outside the cut's box by more than the points may be off: not on it.)
          double em = e + 1.0001 * E.error(m);
          V3 at = P[m];
          if (at.x < lo.x - em || at.x > hi.x + em || at.y < lo.y - em || at.y > hi.y + em || at.z < lo.z - em || at.z > hi.z + em) continue;
          if (orientFlat(f, p, q, m) != 0) continue;
          int s1 = E.lex(p, m), s2 = E.lex(m, q);
          if (s1 == 0 || s1 != s2) continue;
          grew = (s.twin != UINT32_MAX ? putIn(s.twin, m) : putOnEdge(s.u, s.v, m)) || grew;
        }
      }
    }
    return grew;
  }

  // Each cut triangle re-made along its cuts: its pieces (global points) and which of their edges are cuts.
  struct Piece {
    uint32_t v[3];
    uint32_t from;  // the triangle it's a piece of
  };
  // A triangle's points as the triangulation asks about them: each its global point, every answer exact.
  struct Held : Tri2::Points {
    Cutter &c;
    uint32_t t;
    Flat f;
    std::vector<uint32_t> &global;
    Held(Cutter &c, uint32_t t, Flat f, std::vector<uint32_t> &global) : c(c), t(t), f(f), global(global) {}
    int orient(int a, int b, int d) const override { return c.orientFlat(f, global[a], global[b], global[d]); }
    bool between(int a, int m, int b) const override {
      int s1 = c.E.lex(global[a], global[m]), s2 = c.E.lex(global[m], global[b]);
      return s1 != 0 && s1 == s2;
    }
    // Two cuts crossing inside the triangle where none was found before (never expected): where the three planes meet.
    int cross(int, int, int tagAB, int, int, int tagRL) override {
      const auto &segs = c.S.cut[t].segs;
      if (tagAB <= 0 || tagRL <= 0 || tagAB > (int)segs.size() || tagRL > (int)segs.size()) return -1;
      uint32_t id = c.E.planes({c.S.corner(t, 0), c.S.corner(t, 1), c.S.corner(t, 2)}, c.carrier(segs[tagAB - 1], f), c.carrier(segs[tagRL - 1], f));
      c.parent.push_back(id);
      global.push_back(id);
      c.local.set(id, (int)global.size() - 1);
      combineReport.misses++;
      return (int)global.size() - 1;
    }
  };
  std::vector<char> remade;  // per triangle: cut into pieces
  void remake(std::vector<Piece> &out, std::unordered_set<uint64_t> &cuts) {
    size_t nt = S.w.count();
    out.reserve(nt * 2);
    remade.assign(nt, 0);
    for (uint32_t t = 0; t < nt; t++) {
      uint32_t g[3] = {S.corner(t, 0), S.corner(t, 1), S.corner(t, 2)};
      bool touched = S.cut[t].any();
      for (int k = 0; k < 3 && !touched; k++) touched = !S.onEdge[S.edgeOf(g[k], g[(k + 1) % 3])].empty();
      remade[t] = touched;
      if (!touched) {
        out.push_back({{rep(g[0]), rep(g[1]), rep(g[2])}, t});
        continue;
      }
      Flat f = flatOf(t);
      std::vector<uint32_t> global{rep(g[0]), rep(g[1]), rep(g[2])};
      Held held(*this, t, f, global);
      Tri2 tri(held);
      local.begin(P.size());
      for (int k = 0; k < 3; k++) local.set(global[k], k);
      // Point q put in (by `put`, given its number): where it came out one with a point already there, made one with it.
      auto add = [&](uint32_t q, auto put) {
        int i = (int)global.size();
        global.push_back(q);
        int at = put(i);
        if (at == i) {
          local.set(q, i);
          return i;
        }
        global.pop_back();
        if (at < 0) return -1;
        unite(global[at], q);
        local.set(q, at), local.set(rep(q), at);
        return at;
      };
      // Points on the sides, in order from each side's start.
      for (int k = 0; k < 3; k++) {
        uint32_t u = g[k], v = g[(k + 1) % 3];
        std::vector<uint32_t> pts = S.onEdge[S.edgeOf(u, v)];
        pts.push_back(u), pts.push_back(v);
        inLine(pts);
        if (E.lex(u, v) > 0) std::reverse(pts.begin(), pts.end());
        int prev = local.get(rep(u)), end = local.get(rep(v));
        for (uint32_t q : pts) {
          if (local.has(q)) {
            prev = local.get(q);
            continue;
          }
          int i = add(q, [&](int p) { return tri.insertOnEdge(prev, end, p); });
          if (i >= 0) prev = i;
        }
      }
      const Soup::Cut &cutT = S.cut[t];
      for (uint32_t q : cutT.in) {
        q = rep(q);
        if (local.has(q)) continue;
        if (add(q, [&](int p) { return tri.insert(p); }) < 0) combineReport.misses++;
      }
      for (size_t k = 0; k < cutT.segs.size(); k++) {
        uint32_t p = rep(cutT.segs[k].p), q = rep(cutT.segs[k].q);
        if (p == q) continue;
        if (!local.has(p) || !local.has(q)) {
          combineReport.segsDropped++;
          continue;
        }
        if (!tri.keep(local.get(p), local.get(q), (int)k + 1)) combineReport.keepsFailed++;
      }
      combineReport.misses += tri.misses + tri.fallbacks;
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

  // Each shape's winding number just in front of piece (the side its triangle faces): the triangles of each a ray from its
  // middle crosses, each counted +1 where the ray leaves through it and −1 where it enters (those in the piece's own plane
  // left out). Exact; a ray that grazes an edge or a corner is given up for one along another way. False where every way
  // grazes something (never expected).
  bool windingFront(const Piece &pc, const Boxes &tree, double margin, std::vector<int> &w) {
    uint32_t t = pc.from, q = E.middle(pc.v[0], pc.v[1], pc.v[2]);
    parent.push_back(q);
    V3 t0 = P[S.corner(t, 0)], t1 = P[S.corner(t, 1)], t2 = P[S.corner(t, 2)];
    auto inPlane = [&](uint32_t u) {
      for (int k = 0; k < 3; k++)
        if (orient3d(t0, t1, t2, P[S.corner(u, k)]) != 0) return false;
      return true;
    };
    uint32_t seed = 0x2545F491u;
    // Out along the piece's own normal first (the side it faces is outside, so the way out is short and crosses little),
    // a little askew; then other ways at random.
    V3 nn = unit(cross(t1 - t0, t2 - t0));
    for (int attempt = 0; attempt < 48; attempt++) {
      int64_t d[3];
      for (int k = 0; k < 3; k++) {
        seed = seed * 1664525u + 1013904223u;
        int64_t r = (int64_t)(seed >> 11) - (1 << 20);
        d[k] = attempt < 8 ? (int64_t)std::llround(nn[k] * (1 << 20)) + r / 64 : r;
      }
      int s = E.facing(S.corner(t, 0), S.corner(t, 1), S.corner(t, 2), d);
      if (s == 0) continue;
      if (s < 0)
        for (auto &x : d) x = -x;
      std::fill(w.begin(), w.end(), 0);
      bool grazed = false;
      tree.along(P[q], {(double)d[0], (double)d[1], (double)d[2]}, margin, [&](uint32_t u) {
        if (grazed) return;
        uint32_t a = S.corner(u, 0), b = S.corner(u, 1), c = S.corner(u, 2);
        int nd = E.facing(a, b, c, d);
        // (In the piece's own plane: the ray leaves it at once; told from its corners alone.)
        if (inPlane(u)) return;
        int o = E.orient3d(a, b, c, q);
        // In its plane: the ray leaves it at once (or runs along it, which tells nothing).
        if (o == 0) {
          grazed = nd == 0;
          return;
        }
        // Crossing its plane ahead where the ray runs toward it from the side q is on.
        if (nd != o) return;
        int s1 = E.passes(q, d, a, b), s2 = E.passes(q, d, b, c), s3 = E.passes(q, d, c, a);
        if ((s1 > 0 || s2 > 0 || s3 > 0) && (s1 < 0 || s2 < 0 || s3 < 0)) return;
        if (s1 == 0 || s2 == 0 || s3 == 0) {
          grazed = true;
          return;
        }
        w[S.shape(u)] += nd;
      });
      if (!grazed) return true;
    }
    return false;
  }
};

double winding(const Welded &w, uint32_t from, uint32_t to, V3 q) {
  double sum = 0;
  for (size_t k = 3 * from; k < 3 * to; k += 3) {
    V3 a = w.pts[w.tri[k]] - q, b = w.pts[w.tri[k + 1]] - q, c = w.pts[w.tri[k + 2]] - q;
    double la = norm(a), lb = norm(b), lc = norm(c);
    double det = dot(a, cross(b, c)), div = la * lb * lc + dot(a, b) * lc + dot(a, c) * lb + dot(b, c) * la;
    sum += 2 * trig::atan2(det, div);
  }
  return sum / (4 * M_PI);
}

// What's inside the result, by each shape's winding number there: two shapes merged, subtracted or intersected (`op`), or
// (`takeEnd` set) the first shape less shapes 1 to takeEnd − 1 and with the rest added.
struct Rule {
  int op = BK_UNION;
  int takeEnd = 0;
  bool inside(const std::vector<int> &w) const {
    if (!takeEnd) {
      bool a = w[0] > 0, b = w[1] > 0;
      return op == BK_UNION ? a || b : op == BK_SUBTRACT ? a && !b : a && b;
    }
    bool in = w[0] > 0;
    for (int k = 1; k < takeEnd && in; k++) in = w[k] <= 0;
    for (int k = takeEnd; k < (int)w.size() && !in; k++) in = w[k] > 0;
    return in;
  }
  // Whether shape k's faces face the other way in the result (what it bounds is taken away).
  bool turned(int k) const { return takeEnd ? k >= 1 && k < takeEnd : op == BK_SUBTRACT && k == 1; }
};

// Which pieces are kept: 1 as they are, -1 turned round, 0 not. A piece is kept where the operation's answer differs
// from one side of it to the other, facing the way that's inside; where triangles lie on one another, from the first of
// them only. The winding numbers just in front of a piece are the same all over a region: pieces joined across sides
// that aren't cuts and that no other piece shares, and pieces of one triangle across cuts only where a triangle lying in
// its plane ends (nothing crossing there, so nothing in front changes). Each region is told by one ray from its largest
// piece; behind a piece, its own triangle and those lying over it count too.
std::vector<int> choose(Cutter &c, const std::vector<Cutter::Piece> &pieces, const std::unordered_set<uint64_t> &cuts, const Rule &rule,
                        const Boxes &tree, double margin) {
  size_t n = pieces.size();
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
  // Whether point x lies on the cut p–q (all three in the triangle's plane, seen flat by f).
  auto onSeg = [&](const Flat &f, uint32_t x, uint32_t p, uint32_t q) {
    if (x == p || x == q) return true;
    if (c.orientFlat(f, p, q, x) != 0) return false;
    int s1 = c.E.lex(p, x), s2 = c.E.lex(x, q);
    return s1 == 0 || s2 == 0 || s1 == s2;
  };
  // Whether the side a–b of a piece of triangle t lies along a cut where another triangle crosses t (rather than where
  // one lying in its plane ends).
  auto crossedAlong = [&](uint32_t t, uint32_t a, uint32_t b) {
    const auto &segs = c.S.cut[t].segs;
    Flat f = c.flatOf(t);
    for (const Soup::Seg &sg : segs) {
      if (sg.twin == UINT32_MAX) continue;
      uint32_t p = c.rep(sg.p), q = c.rep(sg.q);
      if (onSeg(f, a, p, q) && onSeg(f, b, p, q)) return true;
    }
    return false;
  };
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
        uint32_t lo = groupOf[3 * i + e], hi = groupEnd[lo];
        uint32_t join = UINT32_MAX;
        if (!cuts.count(sides[lo].first)) {
          if (hi - lo != 2) continue;
          join = sides[lo].second / 3 == i ? sides[lo + 1].second / 3 : sides[lo].second / 3;
        } else {
          int same = 0;
          for (uint32_t k = lo; k < hi; k++) {
            uint32_t j = sides[k].second / 3;
            if (j != i && pieces[j].from == pieces[i].from) join = j, same++;
          }
          if (same != 1 || region[join] >= 0 || crossedAlong(pieces[i].from, pieces[i].v[e], pieces[i].v[(e + 1) % 3])) continue;
        }
        if (region[join] >= 0) continue;
        region[join] = r;
        regions[r].push_back(join);
      }
    }
  }
  // The triangles lying in each piece's plane over it (all three of its corners in one or on its sides: the pieces never
  // cross their sides, which were cuts), and whether each faces the same way.
  std::vector<uint32_t> overAt(n + 1, 0);
  std::vector<std::pair<uint32_t, int>> overs;
  for (size_t i = 0; i < n; i++) {
    overAt[i] = (uint32_t)overs.size();
    uint32_t t = pieces[i].from;
    if (c.S.cut[t].partners.empty()) continue;
    Flat f = c.flatOf(t);
    for (uint32_t u : c.S.cut[t].partners) {
      uint32_t g[3] = {c.S.corner(u, 0), c.S.corner(u, 1), c.S.corner(u, 2)};
      int same = c.orientFlat(f, g[0], g[1], g[2]) > 0 ? 1 : -1;
      if (same < 0) std::swap(g[1], g[2]);
      bool inside = true;
      for (int k = 0; k < 3 && inside; k++) inside = c.where(f, g, pieces[i].v[k]) >= 0;
      if (inside) overs.push_back({u, same});
    }
  }
  overAt[n] = (uint32_t)overs.size();
  int shapes = c.S.shapes();
  std::vector<int> keep(n, 0), front(shapes), back(shapes);
  for (auto &reg : regions) {
    uint32_t best = reg[0];
    double area = -1;
    for (uint32_t i : reg) {
      V3 a = c.P[pieces[i].v[0]], b = c.P[pieces[i].v[1]], d = c.P[pieces[i].v[2]];
      double s = norm(cross(b - a, d - a));
      if (s > area) area = s, best = i;
    }
    if (!c.windingFront(pieces[best], tree, margin, front)) {
      // (Never expected.) By the winding numbers in floating point, a hair in front of the piece.
      combineReport.unsure++;
      const Cutter::Piece &pc = pieces[best];
      V3 a = c.P[pc.v[0]], b = c.P[pc.v[1]], d = c.P[pc.v[2]], nn = cross(b - a, d - a);
      V3 mid = (a + b + d) / 3 + unit(nn) * margin;
      for (int k = 0; k < shapes; k++) front[k] = (int)std::lround(winding(c.S.w, c.S.start[k], c.S.start[k + 1], mid));
    }
    bool f = rule.inside(front);
    for (uint32_t i : reg) {
      // Behind it: past its own triangle, and every one lying over it, each as it faces.
      back = front;
      back[c.S.shape(pieces[i].from)]++;
      for (uint32_t k = overAt[i]; k < overAt[i + 1]; k++) back[c.S.shape(overs[k].first)] += overs[k].second;
      bool b = rule.inside(back);
      keep[i] = f == b ? 0 : b ? 1 : -1;
      // Of the second of two shapes' own surface (out of it in front, into it behind), outside the first on both sides.
      if (shapes == 2 && c.S.shape(pieces[i].from) == 1 && front[1] <= 0 && back[1] > 0 && front[0] <= 0 && back[0] <= 0) combineReport.lastOutside++;
    }
  }
  // Where triangles lie on one another, the piece is kept from the first of them only.
  for (size_t i = 0; i < n; i++)
    for (uint32_t k = overAt[i]; k < overAt[i + 1] && keep[i]; k++)
      if (overs[k].first < pieces[i].from) keep[i] = 0;
  return keep;
}

// Points put on a grid finer than anything measured (about 10⁻¹² of the shapes' size), the same for both shapes: faces
// meant to lie on one another (a shape turned a quarter turn, whose sines and cosines come out a hair off) then do so
// exactly, and are merged by the rules for faces on faces rather than as two faces a hair apart.
// Triangles taken out (by mark), the rest kept in order.
void compact(Welded &w, const std::vector<char> &dead) {
  size_t n = 0;
  bool marks = w.sound.size() == w.count();
  for (size_t t = 0; t < w.count(); t++) {
    if (dead[t]) continue;
    for (int k = 0; k < 3; k++) w.tri[3 * n + k] = w.tri[3 * t + k], w.nrm[3 * n + k] = w.nrm[3 * t + k];
    if (!w.gap.empty())
      for (int k = 0; k < 6; k++) w.gap[6 * n + k] = w.gap[6 * t + k];
    if (marks) w.sound[n] = w.sound[t];
    w.face[n++] = w.face[t];
  }
  w.tri.resize(3 * n), w.nrm.resize(3 * n), w.face.resize(n);
  if (!w.gap.empty()) w.gap.resize(6 * n);
  if (marks) w.sound.resize(n);
}

Welded gridded(const Solid &s, double step) {
  Welded w = weld(s, step);
  // A triangle left with two corners in one place has no area: out (its sides cancel).
  {
    std::vector<char> dead(w.count(), 0);
    for (size_t t = 0; t < w.count(); t++) {
      const uint32_t *v = &w.tri[3 * t];
      dead[t] = v[0] == v[1] || v[1] == v[2] || v[0] == v[2];
    }
    compact(w, dead);
  }
  // Nor has one with its three corners on a line (as merges leave where a cut grazes a side): it would lie flat on every
  // triangle of the other shape there and be cut by nonsense. Its long side's neighbour is split at its middle corner
  // instead, and it goes (each side still met by one running the other way; two such facing each other both go).
  // (Again until none is left: one in a row of them along a line waits for its neighbour to go first, and splitting a
  // neighbour with its far corner on the line too makes two more, one step along.)
  bool gaps = !w.gap.empty();
  for (int round = 0; round < 256; round++) {
    size_t nt = w.count();
    auto onLine = [&](size_t t) {
      V3 a = w.pts[w.tri[3 * t]], b = w.pts[w.tri[3 * t + 1]], c = w.pts[w.tri[3 * t + 2]];
      return orient2d(a.x, a.y, b.x, b.y, c.x, c.y) == 0 && orient2d(a.y, a.z, b.y, b.z, c.y, c.z) == 0 && orient2d(a.z, a.x, b.z, b.x, c.z, c.x) == 0;
    };
    std::vector<size_t> flat;
    for (size_t t = 0; t < nt; t++)
      if (onLine(t)) flat.push_back(t);
    // (Any left after all that, never expected, counted: they'd be cut by nonsense.)
    if (flat.empty() || round == 255) {
      combineReport.misses += (long)flat.size();
      break;
    }
    std::unordered_map<uint64_t, uint32_t> side;
    side.reserve(3 * nt);
    for (size_t t = 0; t < nt; t++)
      for (int k = 0; k < 3; k++) side[(uint64_t)w.tri[3 * t + k] << 32 | w.tri[3 * t + (k + 1) % 3]] = (uint32_t)t;
    std::vector<char> dead(nt, 0), changed(nt, 0);
    for (size_t t : flat) {
      if (dead[t] || changed[t]) continue;
      // The long side a → b, and c on it between.
      int k = 0;
      double best = -1;
      for (int j = 0; j < 3; j++) {
        double l2 = norm2(w.pts[w.tri[3 * t + (j + 1) % 3]] - w.pts[w.tri[3 * t + j]]);
        if (l2 > best) best = l2, k = j;
      }
      uint32_t a = w.tri[3 * t + k], b = w.tri[3 * t + (k + 1) % 3], c = w.tri[3 * t + (k + 2) % 3];
      auto it = side.find((uint64_t)b << 32 | a);
      if (it == side.end() || it->second == t || dead[it->second] || changed[it->second]) continue;
      uint32_t n = it->second;
      int kn = 0;
      while (w.tri[3 * n + kn] != b) kn++;
      uint32_t d = w.tri[3 * n + (kn + 2) % 3];
      dead[t] = 1;
      if (d == c) {
        dead[n] = 1;
        continue;
      }
      // n = (b, a, d) becomes (b, c, d), and (c, a, d) is added; c's normal and the slivers from n's.
      V3 A = w.pts[a], B = w.pts[b], C = w.pts[c];
      double sAt = best > 0 ? dot(C - A, B - A) / best : 0.5;
      V3 nb = w.nrm[3 * n + kn], na = w.nrm[3 * n + (kn + 1) % 3], nd = w.nrm[3 * n + (kn + 2) % 3], nc = unit(na * (1 - sAt) + nb * sAt);
      double g[6] = {0, 0, 0, 0, 0, 0};
      V3 q[3] = {w.pts[w.tri[3 * n]], w.pts[w.tri[3 * n + 1]], w.pts[w.tri[3 * n + 2]]};
      if (gaps) std::copy(&w.gap[6 * n], &w.gap[6 * n] + 6, g);
      uint32_t f = w.face[n];
      auto put = [&](size_t at, uint32_t x, uint32_t y, uint32_t z, V3 nx, V3 ny, V3 nz) {
        if (at == w.count()) {
          if (w.sound.size() == w.count()) w.sound.push_back(0);
          w.tri.insert(w.tri.end(), {x, y, z}), w.nrm.insert(w.nrm.end(), {nx, ny, nz}), w.face.push_back(f);
          if (gaps) w.gap.insert(w.gap.end(), 6, 0.0);
          dead.push_back(0), changed.push_back(1);
        } else {
          if (w.sound.size() == w.count()) w.sound[at] = 0;
          w.tri[3 * at] = x, w.tri[3 * at + 1] = y, w.tri[3 * at + 2] = z;
          w.nrm[3 * at] = nx, w.nrm[3 * at + 1] = ny, w.nrm[3 * at + 2] = nz;
          changed[at] = 1;
        }
        if (gaps) {
          V3 piece[3] = {w.pts[x], w.pts[y], w.pts[z]};
          gapOfPiece(g, q[0], q[1], q[2], piece, &w.gap[6 * at]);
        }
      };
      put(n, b, c, d, nb, nc, nd);
      put(w.count(), c, a, d, nc, na, nd);
    }
    compact(w, dead);
  }
  return w;
}

}  // namespace

// One merge, looking for where each shape meets itself only where it isn't known not to (or, `everywhere`, all over);
// `open` says whether the result came out not closed.
static Solid combineOnce(const std::vector<const Solid *> &in, const Rule &rule, bool keepGrid, bool everywhere, bool &open) {
  long overflows = bigOverflowCount();
  Cutter c;
  double step;
  int shapes = (int)in.size();
  std::vector<Welded> ws(shapes);
  {
    double scale = 1;
    for (const Solid *s : in)
      for (V3 q : s->p) scale = std::max({scale, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
    step = std::ldexp(1.0, std::ilogb(scale) + 1 - 40);
    // A merge of a merge: on the grid it's on already (put on a coarser one, its points would round apart from where a
    // part made afresh rounds the same place).
    if (keepGrid)
      for (const Solid *s : in)
        if (s->grid > 0) step = std::min(step, s->grid);
    // No finer than 2^-60 of the size, so the exact tests' whole numbers stay in bounds (Implicit.hpp).
    step = std::max(step, std::ldexp(1.0, std::ilogb(scale) + 1 - 60));
    for (int k = 0; k < shapes; k++) ws[k] = gridded(*in[k], step);
  }
  // The shapes' triangles together, a point two have made one.
  Soup &S = c.S;
  std::vector<uint32_t> faceStart{0};
  for (const Solid *s : in) faceStart.push_back(faceStart.back() + (uint32_t)s->faces.size());
  {
    const Welded &wa = ws[0];
    S.w = wa;
    S.start = {0, (uint32_t)wa.count()};
    auto bits = [](V3 q) {
      uint64_t a, b, d;
      std::memcpy(&a, &q.x, 8), std::memcpy(&b, &q.y, 8), std::memcpy(&d, &q.z, 8);
      return std::array<uint64_t, 3>{a, b, d};
    };
    struct Hash {
      size_t operator()(const std::array<uint64_t, 3> &k) const { return (size_t)(k[0] * 0x9E3779B97F4A7C15ull ^ k[1] * 0xC2B2AE3D27D4EB4Full ^ k[2] * 0x165667B19E3779F9ull); }
    };
    std::unordered_map<std::array<uint64_t, 3>, uint32_t, Hash> at;
    size_t all = 0;
    for (const Welded &w : ws) all += w.pts.size();
    at.reserve(all);
    for (uint32_t i = 0; i < wa.pts.size(); i++) at.emplace(bits(wa.pts[i]), i);
    bool gaps = false;
    for (const Welded &w : ws) gaps = gaps || !w.gap.empty();
    if (gaps && S.w.gap.empty()) S.w.gap.assign(6 * wa.count(), 0.0);
    for (int k = 1; k < shapes; k++) {
      const Welded &wb = ws[k];
      std::vector<uint32_t> id(wb.pts.size());
      for (uint32_t i = 0; i < wb.pts.size(); i++) {
        auto [it, fresh] = at.emplace(bits(wb.pts[i]), (uint32_t)S.w.pts.size());
        if (fresh) S.w.pts.push_back(wb.pts[i]);
        id[i] = it->second;
      }
      for (uint32_t v : wb.tri) S.w.tri.push_back(id[v]);
      S.w.nrm.insert(S.w.nrm.end(), wb.nrm.begin(), wb.nrm.end());
      for (uint32_t f : wb.face) S.w.face.push_back(faceStart[k] + f);
      if (gaps) {
        if (wb.gap.empty()) S.w.gap.insert(S.w.gap.end(), 6 * wb.count(), 0.0);
        else S.w.gap.insert(S.w.gap.end(), wb.gap.begin(), wb.gap.end());
      }
      S.start.push_back((uint32_t)S.w.count());
    }
    for (int k = 0; k < shapes; k++) S.owner.insert(S.owner.end(), S.start[k + 1] - S.start[k], (uint32_t)k);
  }
  size_t nt = S.w.count();
  // Triangles known not to cross each other: each shape's own marks, while its points stayed on this grid.
  S.w.sound.assign(nt, 0);
  for (int k = 0; k < shapes; k++)
    if (in[k]->grid == step && ws[k].sound.size() == ws[k].count()) std::copy(ws[k].sound.begin(), ws[k].sound.end(), S.w.sound.begin() + S.start[k]);
  c.E = ExactPoints(step);
  for (V3 q : S.w.pts) c.E.grid(q);
  c.parent.resize(c.P.size());
  std::iota(c.parent.begin(), c.parent.end(), 0);
  S.edges.reserve(3 * nt);
  for (uint32_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) S.edges.push_back(pairKey(S.corner(t, k), S.corner(t, (k + 1) % 3)));
  std::sort(S.edges.begin(), S.edges.end());
  S.edges.erase(std::unique(S.edges.begin(), S.edges.end()), S.edges.end());
  S.onEdge.resize(S.edges.size());
  S.cut.resize(nt);
  // Pairs of triangles whose boxes meet, in order (the tree's own order is the standard library's to choose), so points
  // are numbered alike everywhere.
  std::vector<Box> boxes(nt);
  for (uint32_t t = 0; t < nt; t++) {
    V3 a = c.P[S.corner(t, 0)], b = c.P[S.corner(t, 1)], d = c.P[S.corner(t, 2)];
    boxes[t] = {vmin(a, vmin(b, d)), vmax(a, vmax(b, d))};
  }
  Boxes tree(boxes);
  // Pairs of triangles whose boxes meet: every pair from two shapes, and within each shape every pair but those of two
  // triangles already known not to cross (unless `everywhere`). So a shape merged with one small part after another is
  // looked at against itself only where it changed.
  std::vector<char> known(nt, 0);
  if (!everywhere)
    for (uint32_t t = 0; t < nt; t++) known[t] = S.w.sound[t];
  std::vector<uint64_t> pairs;
  // (Found from every shape but the one with most triangles.)
  int most = 0;
  for (int k = 1; k < shapes; k++)
    if (S.start[k + 1] - S.start[k] > S.start[most + 1] - S.start[most]) most = k;
  for (uint32_t t = 0; t < nt; t++) {
    bool across = S.shape(t) != most;
    if (known[t] && !across) continue;
    tree.overlapping(boxes[t], [&](uint32_t u) {
      if (u == t) return;
      if (S.shape(u) != S.shape(t)) {
        if (across) pairs.push_back((uint64_t)std::min(t, u) << 32 | std::max(t, u));
      } else if (!known[t] && (known[u] || u > t)) {
        pairs.push_back((uint64_t)std::min(t, u) << 32 | std::max(t, u));
      }
    });
  }
  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
  for (uint64_t k : pairs) c.pairOf((uint32_t)(k >> 32), (uint32_t)(k & 0xffffffffu));
  // Triangles lying in one plane: each one's edges as cuts in the other, their points put there too; cuts crossing in a
  // triangle crossed at one point; points on cuts put in what they're made with: again while any of that puts a point
  // anywhere new (another cut may then hold more of them).
  for (int round = 0; round < 16; round++) {
    bool grew = false;
    for (auto [t1, t2] : c.flats) grew = c.flatEdges(t1, t2) || grew;
    grew = c.crossings() || grew;
    grew = c.spread() || grew;
    if (!grew) break;
  }
  std::vector<Cutter::Piece> pieces;
  std::unordered_set<uint64_t> cuts;
  c.remake(pieces, cuts);
  // Points found to be one after a piece was made: pieces left with two corners in one place have no area and go.
  {
    size_t n = 0;
    for (auto &x : pieces) {
      for (auto &v : x.v) v = c.rep(v);
      if (x.v[0] != x.v[1] && x.v[1] != x.v[2] && x.v[0] != x.v[2]) pieces[n++] = x;
    }
    pieces.resize(n);
    std::unordered_set<uint64_t> fixedCuts;
    for (uint64_t k : cuts) fixedCuts.insert(pairKey(c.rep((uint32_t)(k >> 32)), c.rep((uint32_t)(k & 0xffffffffu))));
    cuts.swap(fixedCuts);
  }
  double scale = 1;
  for (V3 q : c.P) scale = std::max({scale, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
  std::vector<int> keep = choose(c, pieces, cuts, rule, tree, 1e-9 * scale);

  Welded out;
  std::vector<char> fresh;  // per piece kept: part of a triangle cut (only these can be thin or short)
  out.pts = c.P;
  Solid result;
  for (int k = 0; k < shapes; k++) {
    result.faces.insert(result.faces.end(), in[k]->faces.begin(), in[k]->faces.end());
    if (rule.turned(k))
      for (size_t f = faceStart[k]; f < faceStart[k + 1]; f++) {
        auto &face = result.faces[f];
        face.normal = -face.normal, face.deficit = -face.deficit;
        if (face.geom.flat) face.geom.pn = -face.geom.pn, face.geom.pd = -face.geom.pd;
      }
  }
  // A piece's corner normals, from its triangle's by where each corner lies in it.
  auto normals = [&](uint32_t t, const uint32_t v[3], V3 out3[3]) {
    V3 a = c.P[S.corner(t, 0)], b = c.P[S.corner(t, 1)], d = c.P[S.corner(t, 2)];
    V3 n0 = S.w.nrm[3 * t], n1 = S.w.nrm[3 * t + 1], n2 = S.w.nrm[3 * t + 2];
    V3 nn = cross(b - a, d - a);
    double whole = dot(nn, nn);
    for (int k = 0; k < 3; k++) {
      V3 p = c.P[v[k]];
      double l0 = whole > 0 ? dot(cross(b - p, d - p), nn) / whole : 1.0 / 3, l1 = whole > 0 ? dot(cross(d - p, a - p), nn) / whole : 1.0 / 3;
      out3[k] = unit(n0 * l0 + n1 * l1 + n2 * (1 - l0 - l1));
    }
  };
  // A piece's share of its triangle's slivers.
  auto gapOf = [&](uint32_t t, const uint32_t v[3], double g[6]) {
    if (S.w.gap.empty()) {
      std::fill(g, g + 6, 0.0);
      return;
    }
    V3 q[3] = {c.P[v[0]], c.P[v[1]], c.P[v[2]]};
    gapOfPiece(&S.w.gap[6 * t], c.P[S.corner(t, 0)], c.P[S.corner(t, 1)], c.P[S.corner(t, 2)], q, g);
  };
  for (size_t i = 0; i < pieces.size(); i++) {
    if (!keep[i]) continue;
    const Cutter::Piece &pc = pieces[i];
    V3 nv[3];
    double g[6];
    normals(pc.from, pc.v, nv);
    gapOf(pc.from, pc.v, g);
    if (keep[i] > 0) {
      out.tri.insert(out.tri.end(), {pc.v[0], pc.v[1], pc.v[2]});
      out.nrm.insert(out.nrm.end(), {nv[0], nv[1], nv[2]});
      out.gap.insert(out.gap.end(), g, g + 6);
    } else {
      // Turned round, and what was beyond its surface now lies within the result: corners 0, 2, 1.
      out.tri.insert(out.tri.end(), {pc.v[0], pc.v[2], pc.v[1]});
      out.nrm.insert(out.nrm.end(), {-nv[0], -nv[2], -nv[1]});
      out.gap.insert(out.gap.end(), {-g[0], -g[2], -g[1], -g[5], -g[4], -g[3]});
    }
    out.face.push_back(S.w.face[pc.from]);
    // Pieces of what's made meet only along their sides, each pair having been looked at here or known not to cross: so
    // marked (unless the clean-up below changes them: a corner rounded to the nearest double moves far less than any
    // triangle left is thin).
    out.sound.push_back(1);
    fresh.push_back(c.remade[pc.from]);
  }
  // Points a rounding apart made one, and pieces left with no area at all (points on one cut) swapped away, so the next
  // merge or cut never meets them.
  open = !balanced(out);
  combineReport.overflows += bigOverflowCount() - overflows;
  clean(out, 1e-9 * scale, &fresh);
  unweld(out, result);
  result.grid = step;
  return result;
}

// Where each shape meets itself is looked for only where it isn't known not to: not between triangles an earlier merge
// found not to cross. A result that isn't closed (a shape meeting itself farther off, so that a region there runs across
// where it does), or cuts found crossing with no point made where they cross (two triangles marked as not crossing that
// do, their corners rounded since), is made again looking at every pair.
static Solid merged(const std::vector<const Solid *> &in, const Rule &rule, bool keepGrid) {
  combineReport.calls++;
  bool open = false;
  CombineReport before = combineReport;
  combineReport.lastOutside = 0;
  Solid r = combineOnce(in, rule, keepGrid, false, open);
  if (open || combineReport.misses != before.misses || combineReport.keepsFailed != before.keepsFailed || combineReport.segsDropped != before.segsDropped) {
    combineReport = before;
    combineReport.again++, combineReport.lastOutside = 0;
    r = combineOnce(in, rule, keepGrid, true, open);
  }
  if (open) combineReport.open++;
  return r;
}

Solid combine(const Solid &sa, const Solid &sb, int op, double merge, bool keepGrid) {
  (void)merge;
  Rule rule;
  rule.op = op;
  return merged({&sa, &sb}, rule, keepGrid);
}

Solid resolved(const Solid &s) {
  Rule rule;
  rule.takeEnd = 1;
  return merged({&s}, rule, false);
}

Solid combine(const Solid &base, const std::vector<Solid> &take, const std::vector<Solid> &add) {
  if (take.empty() && add.empty()) return base;
  std::vector<const Solid *> in{&base};
  for (const Solid &s : take) in.push_back(&s);
  for (const Solid &s : add) in.push_back(&s);
  Rule rule;
  rule.takeEnd = 1 + (int)take.size();
  return merged(in, rule, false);
}

}  // namespace bce
