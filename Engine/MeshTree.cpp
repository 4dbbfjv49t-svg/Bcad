// A tree of boxes over a mesh's triangles (MeshTree.hpp). Each box is halved at the middle triangle along its longest
// side (the halves fixed by a full order, so the tree comes out the same everywhere); far groups of triangles count in
// the winding number as one (Barill et al., "Fast winding numbers for soups and clouds", 2018, to the first order: the
// next order is no closer over a curved patch, where the order after it is as large).
#include "Engine/MeshTree.hpp"

#include "Engine/Sculpt.hpp"
#include "Engine/Trig.hpp"

#include <algorithm>

namespace bce {

namespace {
constexpr double far = 2;      // a group counts as one past this many times its reach
constexpr uint32_t leaf = 8;  // triangles at most in a leaf
}

MeshTree::MeshTree(const std::vector<V3> &pts, const std::vector<uint32_t> &tris) : P(pts), T(tris) {
  uint32_t n = (uint32_t)(tris.size() / 3);
  order.resize(n);
  std::vector<V3> mids(n);
  for (uint32_t t = 0; t < n; t++) order[t] = t, mids[t] = (P[T[3 * t]] + P[T[3 * t + 1]] + P[T[3 * t + 2]]) / 3;
  nodes.reserve(n / 2 + 1);
  build(0, n, mids);
}

uint32_t MeshTree::build(uint32_t first, uint32_t count, std::vector<V3> &mids) {
  uint32_t at = (uint32_t)nodes.size();
  nodes.emplace_back();
  Node node;
  node.first = first, node.count = count;
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY}, mlo = lo, mhi = hi;
  for (uint32_t i = first; i < first + count; i++) {
    uint32_t t = order[i];
    for (int k = 0; k < 3; k++) lo = vmin(lo, P[T[3 * t + k]]), hi = vmax(hi, P[T[3 * t + k]]);
    mlo = vmin(mlo, mids[t]), mhi = vmax(mhi, mids[t]);
  }
  node.lo = lo, node.hi = hi;
  V3 ext = mhi - mlo;
  int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : ext.y >= ext.z ? 1 : 2;
  uint32_t *o = order.data() + first;
  if (count <= leaf || !(ext[axis] > 0)) {
    // A leaf: its triangles in their own order. (All their middles in one place: halved in that order, while many.)
    std::sort(o, o + count);
    if (count > leaf) {
      uint32_t half = count / 2;
      node.count = 0;
      node.left = build(first, half, mids);
      node.right = build(first + half, count - half, mids);
    }
  } else {
    uint32_t half = count / 2;
    std::nth_element(o, o + half, o + count, [&](uint32_t a, uint32_t b) { return mids[a][axis] != mids[b][axis] ? mids[a][axis] < mids[b][axis] : a < b; });
    node.count = 0;
    node.left = build(first, half, mids);
    node.right = build(first + half, count - half, mids);
  }
  if (node.count || !count) {
    V3 sum{0, 0, 0};
    for (uint32_t i = first; i < first + count; i++) {
      uint32_t t = order[i];
      V3 a = P[T[3 * t]], b = P[T[3 * t + 1]], c = P[T[3 * t + 2]], v = cross(b - a, c - a) * 0.5;
      double m = norm(v);
      node.area += v, node.mass += m, sum += mids[t] * m;
    }
    node.centre = node.mass > 0 ? sum / node.mass : (lo + hi) * 0.5;
  } else {
    const Node &l = nodes[node.left], &r = nodes[node.right];
    node.area = l.area + r.area, node.mass = l.mass + r.mass;
    node.centre = node.mass > 0 ? (l.centre * l.mass + r.centre * r.mass) / node.mass : (lo + hi) * 0.5;
  }
  for (int k = 0; k < 8 && count; k++) {
    V3 corner{k & 1 ? hi.x : lo.x, k & 2 ? hi.y : lo.y, k & 4 ? hi.z : lo.z};
    node.radius = std::max(node.radius, norm(corner - node.centre));
  }
  nodes[at] = node;
  return at;
}

MeshTree::Near MeshTree::nearest(V3 q, double within2) const {
  Near best;
  best.d2 = within2;
  if (order.empty()) return best;
  auto boxD2 = [&](const Node &n) {
    double d = 0;
    for (int k = 0; k < 3; k++) {
      double e = std::max({n.lo[k] - q[k], 0.0, q[k] - n.hi[k]});
      d += e * e;
    }
    return d;
  };
  uint32_t stack[128];
  int top = 0;
  stack[top++] = 0;
  while (top) {
    const Node &n = nodes[stack[--top]];
    if (boxD2(n) > best.d2) continue;
    if (n.count) {
      for (uint32_t i = n.first; i < n.first + n.count; i++) {
        uint32_t t = order[i];
        V3 p = nearestOnTriangle(q, P[T[3 * t]], P[T[3 * t + 1]], P[T[3 * t + 2]]);
        double d2 = norm2(p - q);
        if (d2 < best.d2 || (d2 == best.d2 && t < best.tri)) best.d2 = d2, best.at = p, best.tri = t;
      }
      continue;
    }
    // The nearer half looked at first (pushed last).
    double dl = boxD2(nodes[n.left]), dr = boxD2(nodes[n.right]);
    if (dl <= dr) stack[top++] = n.right, stack[top++] = n.left;
    else stack[top++] = n.left, stack[top++] = n.right;
  }
  return best;
}

// The solid angle triangle t fills seen from q, as a part of the whole sphere (van Oosterom and Strackee).
double MeshTree::solidAngle(uint32_t t, V3 q) const {
  V3 a = P[T[3 * t]] - q, b = P[T[3 * t + 1]] - q, c = P[T[3 * t + 2]] - q;
  double la = norm(a), lb = norm(b), lc = norm(c);
  double num = dot(a, cross(b, c)), den = la * lb * lc + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb;
  return trig::atan2(num, den) / (2 * M_PI);
}

double MeshTree::winding(V3 q) const {
  if (order.empty()) return 0;
  double w = 0;
  uint32_t stack[128];
  int top = 0;
  stack[top++] = 0;
  while (top) {
    const Node &n = nodes[stack[--top]];
    V3 d = n.centre - q;
    double r2 = norm2(d);
    if (r2 > far * far * n.radius * n.radius) {
      w += dot(n.area, d) / (4 * M_PI * r2 * std::sqrt(r2));
    } else if (n.count) {
      for (uint32_t i = n.first; i < n.first + n.count; i++) w += solidAngle(order[i], q);
    } else {
      stack[top++] = n.right, stack[top++] = n.left;
    }
  }
  return w;
}

double MeshTree::windingExactly(V3 q) const {
  double w = 0;
  for (uint32_t t = 0; t < T.size() / 3; t++) w += solidAngle(t, q);
  return w;
}

}  // namespace bce
