// Meshes made lighter (Scan.hpp): edges collapsed one by one, the one that moves the surface least first (Garland and
// Heckbert's quadrics), never one that would tear the mesh, turn a triangle over or leave a point with too few
// triangles round it; a mesh too large to work on as it is first made coarser on a grid.
#include "Engine/MeshTree.hpp"
#include "Engine/Scan.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace bce {

namespace {

double fl(double x) { return (double)(float)x; }

// The squared distance to a set of planes, each weighted by its triangle's area: x Q x for x = (p, 1).
struct Quadric {
  double q[10] = {};  // aa ab ac ad bb bc bd cc cd dd
  void addPlane(V3 n, double d, double w) {
    double v[4] = {n.x, n.y, n.z, d};
    int k = 0;
    for (int i = 0; i < 4; i++)
      for (int j = i; j < 4; j++) q[k++] += w * v[i] * v[j];
  }
  Quadric &operator+=(const Quadric &o) {
    for (int k = 0; k < 10; k++) q[k] += o.q[k];
    return *this;
  }
  double at(V3 p) const {
    double x = p.x, y = p.y, z = p.z;
    return q[0] * x * x + 2 * q[1] * x * y + 2 * q[2] * x * z + 2 * q[3] * x + q[4] * y * y + 2 * q[5] * y * z + 2 * q[6] * y + q[7] * z * z +
           2 * q[8] * z + q[9];
  }
  // The point where it's least, if there's one place it is.
  bool least(V3 &p) const {
    double a = q[0], b = q[1], c = q[2], d = q[4], e = q[5], f = q[7];
    double det = a * (d * f - e * e) - b * (b * f - e * c) + c * (b * e - d * c), scale = a + d + f;
    if (!(std::fabs(det) > 1e-9 * scale * scale * scale)) return false;
    double r0 = -q[3], r1 = -q[6], r2 = -q[8];
    p.x = (r0 * (d * f - e * e) - b * (r1 * f - e * r2) + c * (r1 * e - d * r2)) / det;
    p.y = (a * (r1 * f - e * r2) - r0 * (b * f - e * c) + c * (b * r2 - r1 * c)) / det;
    p.z = (a * (d * r2 - r1 * e) - b * (b * r2 - r1 * c) + r0 * (b * e - d * c)) / det;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }
};

struct Collapse {
  double cost;
  uint32_t a, b;          // a < b
  uint32_t stampA, stampB;
  bool operator<(const Collapse &o) const {  // (the heap's top: the least cost, then the lowest points)
    if (cost != o.cost) return cost > o.cost;
    if (a != o.a) return a > o.a;
    return b > o.b;
  }
};

struct Decimator {
  std::vector<V3> &P;
  std::vector<uint32_t> &T;
  std::vector<Quadric> Q;
  std::vector<std::vector<uint32_t>> around;  // per point: its triangles
  std::vector<uint32_t> stamp;
  std::vector<uint8_t> deadPoint, deadTri;
  std::priority_queue<Collapse> heap;
  double tiny;  // the least a triangle's doubled area may be
  size_t live;
  Decimator(std::vector<V3> &P, std::vector<uint32_t> &T) : P(P), T(T) {
    size_t np = P.size(), nt = T.size() / 3;
    Q.assign(np, Quadric()), around.assign(np, {}), stamp.assign(np, 0), deadPoint.assign(np, 0), deadTri.assign(nt, 0);
    V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
    for (const V3 &p : P) lo = vmin(lo, p), hi = vmax(hi, p);
    double size = std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z, 1e-6});
    tiny = 1e-12 * size * size;
    for (uint32_t t = 0; t < nt; t++) {
      V3 a = P[T[3 * t]], b = P[T[3 * t + 1]], c = P[T[3 * t + 2]], n = cross(b - a, c - a);
      double area = norm(n);
      if (area > 0) {
        V3 u = n / area;
        Quadric k;
        k.addPlane(u, -dot(u, a), area / 2);
        for (int i = 0; i < 3; i++) Q[T[3 * t + i]] += k;
      }
      for (int i = 0; i < 3; i++) around[T[3 * t + i]].push_back(t);
    }
    live = nt;
  }
  bool has(uint32_t t, uint32_t v) const { return T[3 * t] == v || T[3 * t + 1] == v || T[3 * t + 2] == v; }
  void neighbours(uint32_t v, std::vector<uint32_t> &out) const {
    out.clear();
    for (uint32_t t : around[v])
      for (int i = 0; i < 3; i++)
        if (T[3 * t + i] != v) out.push_back(T[3 * t + i]);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
  }
  // Where the edge's two points would go, and what moving there costs.
  V3 target(uint32_t a, uint32_t b, double &cost) const {
    Quadric k = Q[a];
    k += Q[b];
    V3 mid = (P[a] + P[b]) * 0.5, p;
    double len2 = norm2(P[b] - P[a]);
    if (k.least(p) && norm2(p - mid) <= 4 * len2) p = V3{fl(p.x), fl(p.y), fl(p.z)};
    else {
      p = P[a], cost = k.at(P[a]);
      double cb = k.at(P[b]), cm = k.at(V3{fl(mid.x), fl(mid.y), fl(mid.z)});
      if (cb < cost) p = P[b], cost = cb;
      if (cm < cost) p = V3{fl(mid.x), fl(mid.y), fl(mid.z)}, cost = cm;
      return p;
    }
    cost = k.at(p);
    return p;
  }
  void push(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    double cost;
    target(a, b, cost);
    heap.push({std::max(cost, 0.0), a, b, stamp[a], stamp[b]});
  }
  std::vector<uint32_t> na, nb;
  // Whether a and b can become one at p: the mesh stays a surface (the two only share the two points across their edge,
  // neither of which is left with fewer than three triangles), no triangle turns over or goes flat.
  bool can(uint32_t a, uint32_t b, V3 p) {
    neighbours(a, na), neighbours(b, nb);
    size_t common = 0, both = 0;
    for (size_t i = 0, j = 0; i < na.size() && j < nb.size();) {
      if (na[i] < nb[j]) i++;
      else if (nb[j] < na[i]) j++;
      else common++, i++, j++;
    }
    for (uint32_t t : around[a]) both += has(t, b);
    if (both != 2 || common != 2 || na.size() + nb.size() - common - 2 < 3) return false;
    for (uint32_t t : around[a])
      if (has(t, b))
        for (int i = 0; i < 3; i++) {
          uint32_t c = T[3 * t + i];
          if (c != a && c != b && around[c].size() < 4) return false;
        }
    for (uint32_t v : {a, b})
      for (uint32_t t : around[v]) {
        if (has(t, a) && has(t, b)) continue;
        V3 c[3];
        for (int i = 0; i < 3; i++) c[i] = P[T[3 * t + i]];
        V3 before = cross(c[1] - c[0], c[2] - c[0]);
        for (int i = 0; i < 3; i++)
          if (T[3 * t + i] == v) c[i] = p;
        V3 after = cross(c[1] - c[0], c[2] - c[0]);
        double la = norm(after);
        if (!(la > tiny) || dot(after, before) <= 0.2 * la * norm(before)) return false;
      }
    return true;
  }
  void collapse(uint32_t a, uint32_t b, V3 p) {
    P[a] = p, Q[a] += Q[b], deadPoint[b] = 1, stamp[a]++;
    for (uint32_t t : around[b]) {
      if (has(t, a)) {
        deadTri[t] = 1, live--;
        for (int i = 0; i < 3; i++) {
          uint32_t c = T[3 * t + i];
          if (c == b) continue;
          auto &list = around[c];
          list.erase(std::find(list.begin(), list.end(), t));
        }
      } else {
        for (int i = 0; i < 3; i++)
          if (T[3 * t + i] == b) T[3 * t + i] = a;
        around[a].push_back(t);
      }
    }
    around[b].clear(), around[b].shrink_to_fit();
    neighbours(a, na);
    for (uint32_t u : na) push(a, u);
  }
};

}  // namespace

bool simplify(std::vector<V3> &P, std::vector<uint32_t> &T, size_t most, const std::function<bool(double)> &going) {
  if (T.size() / 3 <= most) return true;
  Decimator d(P, T);
  {
    std::vector<uint64_t> edges;
    edges.reserve(T.size());
    for (size_t s = 0; s < T.size(); s++) {
      uint32_t a = T[s], b = T[s / 3 * 3 + (s % 3 + 1) % 3];
      if (a < b) edges.push_back((uint64_t)a << 32 | b);
    }
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    std::vector<Collapse> all;
    all.reserve(edges.size());
    for (uint64_t e : edges) {
      uint32_t a = (uint32_t)(e >> 32), b = (uint32_t)e;
      double cost;
      d.target(a, b, cost);
      all.push_back({std::max(cost, 0.0), a, b, 0, 0});
    }
    d.heap = std::priority_queue<Collapse>(std::less<Collapse>(), std::move(all));
  }
  size_t start = d.live, done = 0;
  while (d.live > most && !d.heap.empty()) {
    Collapse c = d.heap.top();
    d.heap.pop();
    if (d.deadPoint[c.a] || d.deadPoint[c.b] || c.stampA != d.stamp[c.a] || c.stampB != d.stamp[c.b]) continue;
    double cost;
    V3 p = d.target(c.a, c.b, cost);
    if (!d.can(c.a, c.b, p)) continue;
    d.collapse(c.a, c.b, p);
    if (++done % 20000 == 0 && going && !going((double)(start - d.live) / (start - most))) return false;
  }
  // The points and triangles left, in order.
  std::vector<uint32_t> id(P.size(), UINT32_MAX);
  std::vector<V3> np;
  std::vector<uint32_t> nt;
  for (size_t t = 0; t < T.size() / 3; t++) {
    if (d.deadTri[t]) continue;
    for (int i = 0; i < 3; i++) {
      uint32_t v = T[3 * t + i];
      if (id[v] == UINT32_MAX) id[v] = (uint32_t)np.size(), np.push_back(P[v]);
      nt.push_back(id[v]);
    }
  }
  P = std::move(np), T = std::move(nt);
  return true;
}

void cluster(std::vector<V3> &P, std::vector<uint32_t> &T, double cell) {
  std::unordered_map<uint64_t, uint32_t> cellOf;  // (only looked up, never walked)
  std::vector<uint32_t> id(P.size(), UINT32_MAX);
  std::vector<V3> sum;
  std::vector<double> count;
  for (size_t c = 0; c < T.size(); c++) {
    uint32_t v = T[c];
    if (id[v] != UINT32_MAX) continue;
    V3 q = P[v];
    uint64_t key = (uint64_t)(std::floor(q.x / cell) + (1 << 20)) << 42 | (uint64_t)(std::floor(q.y / cell) + (1 << 20)) << 21 |
                   (uint64_t)(std::floor(q.z / cell) + (1 << 20));
    auto it = cellOf.find(key);
    if (it == cellOf.end()) it = cellOf.emplace(key, (uint32_t)sum.size()).first, sum.push_back(V3{0, 0, 0}), count.push_back(0);
    id[v] = it->second, sum[it->second] += q, count[it->second]++;
  }
  P.resize(sum.size());
  for (size_t i = 0; i < sum.size(); i++) P[i] = sum[i] / count[i], P[i] = V3{fl(P[i].x), fl(P[i].y), fl(P[i].z)};
  size_t n = 0;
  for (size_t t = 0; t < T.size() / 3; t++) {
    uint32_t a = id[T[3 * t]], b = id[T[3 * t + 1]], c = id[T[3 * t + 2]];
    if (a == b || b == c || c == a) continue;
    T[3 * n] = a, T[3 * n + 1] = b, T[3 * n + 2] = c, n++;
  }
  T.resize(3 * n);
}

double deviation(const std::vector<V3> &from, const std::vector<V3> &P, const std::vector<uint32_t> &T) {
  MeshTree tree(P, T);
  double worst = 0;
  for (const V3 &q : from) worst = std::max(worst, tree.nearest(q).d2);
  return std::sqrt(worst);
}

}  // namespace bce
