// The shortest distance between picked elements: the closest triangles (or segments, or points) found quickly through
// a tree of boxes, then the two points moved onto the exact surfaces, each in turn to the other's nearest, until they
// rest.
#include "Engine/Distance.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace bce {

namespace {

// MARK: - closest points between simple pieces

V3 closestOnSegment(V3 p, V3 a, V3 b) {
  V3 ab = b - a;
  double l2 = norm2(ab);
  if (l2 <= 0) return a;
  return a + ab * std::clamp(dot(p - a, ab) / l2, 0.0, 1.0);
}

// The point of triangle abc nearest p, by which corner, edge or the inside p faces.
V3 closestOnTriangle(V3 p, V3 a, V3 b, V3 c) {
  V3 ab = b - a, ac = c - a, ap = p - a;
  double d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return a;
  V3 bp = p - b;
  double d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return b;
  double vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
  V3 cp = p - c;
  double d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return c;
  double vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
  double va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
  double den = 1 / (va + vb + vc);
  return a + ab * (vb * den) + ac * (vc * den);
}

// The nearest points of segments p1q1 and p2q2.
void closestSegments(V3 p1, V3 q1, V3 p2, V3 q2, V3 &c1, V3 &c2) {
  V3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
  double a = norm2(d1), e = norm2(d2), f = dot(d2, r), s, t;
  const double tiny = 1e-300;
  if (a <= tiny && e <= tiny) {
    s = t = 0;
  } else if (a <= tiny) {
    s = 0, t = std::clamp(f / e, 0.0, 1.0);
  } else {
    double c = dot(d1, r);
    if (e <= tiny) {
      t = 0, s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      double b = dot(d1, d2), den = a * e - b * b;
      s = den > 0 ? std::clamp((b * f - c * e) / den, 0.0, 1.0) : 0;
      t = (b * s + f) / e;
      if (t < 0) t = 0, s = std::clamp(-c / a, 0.0, 1.0);
      else if (t > 1) t = 1, s = std::clamp((b - c) / a, 0.0, 1.0);
    }
  }
  c1 = p1 + d1 * s, c2 = p2 + d2 * t;
}

// Segment pq against triangle abc: where it passes through, or the nearest points.
void closestSegmentTriangle(V3 p, V3 q, V3 a, V3 b, V3 c, V3 &cs, V3 &ct) {
  V3 nrm = cross(b - a, c - a);
  double dp = dot(p - a, nrm), dq = dot(q - a, nrm);
  if ((dp <= 0 && dq >= 0) || (dp >= 0 && dq <= 0)) {
    if (dp != dq) {
      V3 x = p + (q - p) * (dp / (dp - dq));
      V3 y = closestOnTriangle(x, a, b, c);
      if (norm2(x - y) <= 1e-24 * (1 + norm2(x))) {
        cs = ct = x;
        return;
      }
    }
  }
  double best = INFINITY;
  auto take = [&](V3 u, V3 v) {
    double d = norm2(u - v);
    if (d < best) best = d, cs = u, ct = v;
  };
  take(p, closestOnTriangle(p, a, b, c));
  take(q, closestOnTriangle(q, a, b, c));
  V3 u, v;
  closestSegments(p, q, a, b, u, v), take(u, v);
  closestSegments(p, q, b, c, u, v), take(u, v);
  closestSegments(p, q, c, a, u, v), take(u, v);
}

// MARK: - sets of pieces

// What is measured from: points, segments or triangles, and how to put a point onto it exactly.
struct Set {
  int dim = 0;
  std::vector<V3> v;  // dim + 1 per piece
  enum Exact { Mesh, Turned, Circle, Profile } exact = Mesh;
  Elem elem{};
  double r = 0, z = 0;
  Affine toLocal, toWorld;
  // How far its exact form may lie from its mesh. A merged or split shape's faces and edges are only parts of their exact
  // forms: a point put on the form farther than this from the mesh is past where the part ends.
  double trim = 0;
  int count() const { return (int)v.size() / (dim + 1); }
  const V3 *piece(int i) const { return &v[(size_t)i * (dim + 1)]; }
};

// The nearest points of piece i of a and piece j of b; returns their squared distance.
double closestPieces(const Set &a, int i, const Set &b, int j, V3 &pa, V3 &pb) {
  const V3 *x = a.piece(i), *y = b.piece(j);
  if (a.dim < b.dim) return closestPieces(b, j, a, i, pb, pa);
  if (a.dim == 0) {
    pa = x[0], pb = y[0];
  } else if (a.dim == 1) {
    if (b.dim == 0) pa = closestOnSegment(y[0], x[0], x[1]), pb = y[0];
    else closestSegments(x[0], x[1], y[0], y[1], pa, pb);
  } else if (b.dim == 0) {
    pa = closestOnTriangle(y[0], x[0], x[1], x[2]), pb = y[0];
  } else if (b.dim == 1) {
    closestSegmentTriangle(y[0], y[1], x[0], x[1], x[2], pb, pa);
  } else {
    double best = INFINITY;
    V3 u, v;
    for (int k = 0; k < 3; k++) {
      closestSegmentTriangle(x[k], x[(k + 1) % 3], y[0], y[1], y[2], u, v);
      if (norm2(u - v) < best) best = norm2(u - v), pa = u, pb = v;
      closestSegmentTriangle(y[k], y[(k + 1) % 3], x[0], x[1], x[2], v, u);
      if (norm2(u - v) < best) best = norm2(u - v), pa = u, pb = v;
    }
  }
  return norm2(pa - pb);
}

// A tree of boxes over a set's pieces.
struct Tree {
  struct Node {
    V3 lo, hi;
    int left = -1, right = -1, first = 0, count = 0;
  };
  std::vector<Node> nodes;
  std::vector<int> order;

  explicit Tree(const Set &s) {
    int n = s.count();
    std::vector<V3> lo(n), hi(n), mid(n);
    for (int i = 0; i < n; i++) {
      const V3 *p = s.piece(i);
      lo[i] = hi[i] = p[0];
      for (int k = 1; k <= s.dim; k++) lo[i] = vmin(lo[i], p[k]), hi[i] = vmax(hi[i], p[k]);
      mid[i] = (lo[i] + hi[i]) * 0.5;
    }
    order.resize(n);
    for (int i = 0; i < n; i++) order[i] = i;
    nodes.reserve(2 * n / 3 + 2);
    if (n) make(0, n, lo, hi, mid);
  }

  int make(int first, int count, const std::vector<V3> &lo, const std::vector<V3> &hi, const std::vector<V3> &mid) {
    int at = (int)nodes.size();
    nodes.push_back({});
    V3 a = lo[order[first]], b = hi[order[first]], ca = mid[order[first]], cb = ca;
    for (int i = first + 1; i < first + count; i++) {
      a = vmin(a, lo[order[i]]), b = vmax(b, hi[order[i]]);
      ca = vmin(ca, mid[order[i]]), cb = vmax(cb, mid[order[i]]);
    }
    nodes[at].lo = a, nodes[at].hi = b;
    if (count <= 4) {
      nodes[at].first = first, nodes[at].count = count;
      return at;
    }
    V3 ext = cb - ca;
    int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : ext.y >= ext.z ? 1 : 2;
    int half = count / 2;
    std::nth_element(order.begin() + first, order.begin() + first + half, order.begin() + first + count,
                     [&](int u, int v) { return mid[u][axis] < mid[v][axis]; });
    int l = make(first, half, lo, hi, mid);
    int r = make(first + half, count - half, lo, hi, mid);
    nodes[at].left = l, nodes[at].right = r;
    return at;
  }
};

double boxGap2(const Tree::Node &a, const Tree::Node &b) {
  double s = 0;
  for (int k = 0; k < 3; k++) {
    double g = std::max({0.0, b.lo[k] - a.hi[k], a.lo[k] - b.hi[k]});
    s += g * g;
  }
  return s;
}

// The nearest points between two sets: pairs of boxes taken nearest first, until none can come closer than the best.
double nearest(const Set &a, const Tree &ta, const Set &b, const Tree &tb, V3 &pa, V3 &pb) {
  struct Item {
    double gap;
    int i, j;
    bool operator<(const Item &o) const { return gap > o.gap; }
  };
  double best = INFINITY;
  std::priority_queue<Item> queue;
  queue.push({boxGap2(ta.nodes[0], tb.nodes[0]), 0, 0});
  while (!queue.empty()) {
    Item it = queue.top();
    queue.pop();
    if (it.gap >= best) break;
    const auto &na = ta.nodes[it.i], &nb = tb.nodes[it.j];
    bool leafA = na.left < 0, leafB = nb.left < 0;
    if (leafA && leafB) {
      for (int x = na.first; x < na.first + na.count; x++)
        for (int y = nb.first; y < nb.first + nb.count; y++) {
          V3 u, v;
          double d = closestPieces(a, ta.order[x], b, tb.order[y], u, v);
          if (d < best) best = d, pa = u, pb = v;
        }
      continue;
    }
    // Open the larger box (or the one that isn't a leaf).
    bool openA = !leafA && (leafB || norm2(na.hi - na.lo) >= norm2(nb.hi - nb.lo));
    if (openA) {
      for (int c : {na.left, na.right}) {
        double g = boxGap2(ta.nodes[c], nb);
        if (g < best) queue.push({g, c, it.j});
      }
    } else {
      for (int c : {nb.left, nb.right}) {
        double g = boxGap2(na, tb.nodes[c]);
        if (g < best) queue.push({g, it.i, c});
      }
    }
  }
  return best;
}

// The point of a set nearest x: exactly on its surface or curve where the set knows it, otherwise on its mesh.
V3 project(const Set &s, const Tree &t, V3 x) {
  if (s.exact == Set::Mesh) {
    Set one;
    one.v = {x};
    Tree t1(one);
    V3 p, q;
    nearest(s, t, one, t1, p, q);
    return p;
  }
  auto onto = [&](V3 x) {
    V3 y = s.toLocal.point(x);
    double rho = trig::hypot(y.x, y.y), c = rho > 0 ? y.x / rho : 1, sn = rho > 0 ? y.y / rho : 0, r, z;
    if (s.exact == Set::Circle) {
      r = s.r, z = s.z;
    } else if (s.exact == Set::Turned) {
      s.elem.at(s.elem.nearest(rho, y.z), r, z);
    } else {
      // A profile piece in the half-plane y = 0 (its seam).
      s.elem.at(s.elem.nearest(y.x, y.z), r, z);
      return s.toWorld.point({r, 0, z});
    }
    return s.toWorld.point({r * c, r * sn, z});
  };
  V3 y = onto(x);
  if (s.trim > 0) {
    // Past where the part ends: its nearest point on the mesh, put onto the form there.
    Set one;
    one.v = {y};
    Tree t1(one);
    V3 p, q;
    if (nearest(s, t, one, t1, p, q) > s.trim * s.trim) {
      one.v = {x};
      Tree t2(one);
      nearest(s, t, one, t2, p, q);
      return onto(p);
    }
  }
  return y;
}

// The set an end stands for; a curved face or edge comes as a finer mesh where it can't be put exactly.
bool gather(const End &e, Set &s, std::string &why) {
  if (e.kind == End::Point) {
    s.dim = 0, s.v = {e.point};
    return true;
  }
  if (!e.shape || !e.shape->node) {
    why = "distance: no shape to measure";
    return false;
  }
  Shape placed{e.shape->node, e.shape->place.then(e.place)};
  V3 lo, hi;
  bounds(placed, lo, hi);
  double diag = norm(hi - lo);
  for (int pass = 0; pass < 2; pass++) {
    Solid solid;
    double d = pass == 0 ? std::clamp(diag * 2e-3, 0.01, 0.2) : std::clamp(diag * 1e-5, 0.0002, 0.005);
    mesh(placed, d, solid);
    s = Set();
    bool exact = false;
    if (e.kind == End::Face) {
      if (e.index < 0 || e.index >= (int)solid.faces.size()) break;
      s.dim = 2;
      for (size_t k = 0; k < solid.triFace.size(); k++)
        if ((int)solid.triFace[k] == e.index)
          for (int c = 0; c < 3; c++) s.v.push_back(solid.p[solid.tri[3 * k + c]]);
      const FaceGeom &g = solid.faces[e.index].geom;
      if (g.kind == FaceGeom::Turned && g.exact) s.exact = Set::Turned, s.elem = g.elem, s.toWorld = g.place, exact = true;
      // A flat face is exact as meshed.
      if (g.kind == FaceGeom::Flat) exact = true;
    } else {
      if (e.index < 0 || e.index >= (int)solid.edges.size() || solid.edges[e.index].pts.size() < 2) break;
      const auto &edge = solid.edges[e.index];
      s.dim = 1;
      for (size_t k = 0; k + 1 < edge.pts.size(); k++) s.v.push_back(edge.pts[k]), s.v.push_back(edge.pts[k + 1]);
      const EdgeGeom &g = edge.geom;
      if (g.exact && g.kind == EdgeGeom::Circle) s.exact = Set::Circle, s.r = g.r, s.z = g.z, s.toWorld = g.place, exact = true;
      if (g.exact && g.kind == EdgeGeom::Profile) s.exact = Set::Profile, s.elem = g.elem, s.toWorld = g.place, exact = true;
      if (g.kind == EdgeGeom::Line) exact = true;
    }
    if (s.v.empty()) break;
    if (exact) {
      if (s.exact != Set::Mesh) s.toLocal = s.toWorld.inverse();
      if (placed.node->kind != Node::Prim) s.trim = 2 * d + 1e-9 * (1 + diag);
      return true;
    }
    if (pass == 1) return true;
  }
  why = "distance: no such edge or face";
  return false;
}

}  // namespace

bool distance(const End &a, const End &b, double &d, V3 &pa, V3 &pb, std::string &why) {
  Set sa, sb;
  if (!gather(a, sa, why) || !gather(b, sb, why)) return false;
  Tree ta(sa), tb(sb);
  double best = nearest(sa, ta, sb, tb, pa, pb);
  // Onto the exact surfaces: each point in turn to the nearest of its own set to the other, from the meshes' nearest
  // points to where they settle (farther than the meshes' where a surface curves in, as round a hole).
  if (sa.exact != Set::Mesh || sb.exact != Set::Mesh) {
    V3 p = pa, q = pb;
    for (int it = 0; it < 200; it++) {
      V3 np = sa.dim == 0 ? p : project(sa, ta, q);
      V3 nq = sb.dim == 0 ? q : project(sb, tb, np);
      double moved = norm(np - p) + norm(nq - q);
      p = np, q = nq;
      if (moved <= 1e-13 * (1 + norm(p))) break;
    }
    best = norm2(p - q), pa = p, pb = q;
  }
  d = std::sqrt(best);
  return std::isfinite(d);
}

}  // namespace bce
