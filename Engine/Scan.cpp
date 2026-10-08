// Meshes from scans and other apps made into closed bodies (Scan.hpp). Each step keeps what's sound as it is and mends
// only what isn't; every point made is put on a float at once, so what's checked is what bk_mesh_shape will be given.
#include "Engine/MeshTree.hpp"
#include "Engine/Model.hpp"
#include "Engine/Print.hpp"
#include "Engine/Scan.hpp"
#include "Engine/Sculpt.hpp"
#include "Engine/Treat.hpp"
#include "Engine/Triangulate.hpp"
#include "Engine/Weld.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace bce {

namespace {

constexpr uint32_t none = UINT32_MAX;
double fl(double x) { return (double)(float)x; }
V3 fl(V3 q) { return {fl(q.x), fl(q.y), fl(q.z)}; }
uint64_t edgeKey(uint32_t a, uint32_t b) { return a < b ? (uint64_t)a << 32 | b : (uint64_t)b << 32 | a; }
uint32_t nextSide(uint32_t s) { return s / 3 * 3 + (s % 3 + 1) % 3; }
uint32_t lastSide(uint32_t s) { return s / 3 * 3 + (s % 3 + 2) % 3; }

struct Repair {
  const ScanOptions &o;
  ScanReport &r;
  std::string &why;
  std::vector<V3> P;
  std::vector<uint32_t> T;
  double size = 0;
  std::vector<uint32_t> pieceOf;  // per triangle: the piece it was turned one way with
  std::vector<uint8_t> twisted;   // per piece: one that can't be turned one way throughout (a Möbius strip)
  std::vector<uint32_t> mate;     // per side: the side it's joined to, running back along it
  Repair(const ScanOptions &o, ScanReport &r, std::string &why) : o(o), r(r), why(why) {}

  uint32_t triangles() const { return (uint32_t)(T.size() / 3); }
  bool going(double at) {
    if (o.progress && !o.progress(at)) return why = "scan: stopped", false;
    return true;
  }
  // Every side (3t + k: corner k to the next) by its edge, the sides of each edge together, in order: sorted once, then
  // kept up to date as triangles turn round or go (sorted again only where points are made one).
  std::vector<std::pair<uint64_t, uint32_t>> edges;
  bool edgesStale = true;
  const std::vector<std::pair<uint64_t, uint32_t>> &sidesByEdge() {
    if (!edgesStale) return edges;
    edges.resize(T.size());
    for (uint32_t s = 0; s < T.size(); s++) edges[s] = {edgeKey(T[s], T[nextSide(s)]), s};
    std::sort(edges.begin(), edges.end());
    edgesStale = false;
    return edges;
  }
  // Triangle t turned round (corners 1 and 2 swapped: side k becomes side 2 − k, running back).
  void turnRound(const std::vector<uint8_t> &turn) {
    for (uint32_t t = 0; t < triangles(); t++)
      if (turn[t]) std::swap(T[3 * t + 1], T[3 * t + 2]);
    if (edgesStale) return;
    for (auto &e : edges)
      if (turn[e.second / 3]) e.second = e.second / 3 * 3 + 2 - e.second % 3;
    eachEdge(edges, [&](size_t i, size_t j) {
      if (j - i > 1) std::sort(edges.begin() + i, edges.begin() + j);
    });
  }
  template <class F> static void eachEdge(const std::vector<std::pair<uint64_t, uint32_t>> &e, F f) {
    for (size_t i = 0, j; i < e.size(); i = j) {
      for (j = i + 1; j < e.size() && e[j].first == e[i].first;) j++;
      f(i, j);
    }
  }
  // Triangles kept where keep[t], in order (and their pieces, and their sides by edge).
  void keepOnly(const std::vector<uint8_t> &keep) {
    std::vector<uint32_t> to(triangles(), none);
    uint32_t n = 0;
    for (uint32_t t = 0; t < triangles(); t++)
      if (keep[t]) {
        for (int k = 0; k < 3; k++) T[3 * n + k] = T[3 * t + k];
        if (!pieceOf.empty()) pieceOf[n] = pieceOf[t];
        to[t] = n++;
      }
    T.resize(3 * n);
    if (!pieceOf.empty()) pieceOf.resize(n);
    if (edgesStale) return;
    size_t m = 0;
    for (auto &e : edges)
      if (to[e.second / 3] != none) edges[m++] = {e.first, to[e.second / 3] * 3 + e.second % 3};
    edges.resize(m);
  }

  // 1, 2. The triangles whose corners are all numbers and all there, each corner once; their points about the middle
  // of their box (on floats), those in one place made one.
  bool take(const float *pos, size_t points, const uint32_t *tri, size_t count) {
    auto number = [&](uint32_t i) {
      return i < points && std::isfinite(pos[3 * (size_t)i]) && std::isfinite(pos[3 * (size_t)i + 1]) && std::isfinite(pos[3 * (size_t)i + 2]);
    };
    V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
    for (size_t t = 0; t < count; t++)
      if (number(tri[3 * t]) && number(tri[3 * t + 1]) && number(tri[3 * t + 2]))
        for (int k = 0; k < 3; k++) {
          const float *q = pos + 3 * (size_t)tri[3 * t + k];
          lo = vmin(lo, V3{q[0], q[1], q[2]}), hi = vmax(hi, V3{q[0], q[1], q[2]});
        }
    if (!(lo.x <= hi.x)) return why = "scan: no triangles", false;
    V3 ext = hi - lo;
    size = std::max({ext.x, ext.y, ext.z});
    if (size > 10000) return why = "scan: larger than 10 m", false;
    r.offset = middleOf(lo, hi);
    std::vector<uint32_t> id(points, none);
    std::vector<float> fp;
    PointWeld weld(fp);
    T.reserve(3 * count);
    for (size_t t = 0; t < count; t++) {
      uint32_t v[3];
      bool ok = true;
      for (int k = 0; k < 3 && ok; k++) {
        uint32_t i = tri[3 * t + k];
        ok = number(i);
        if (!ok) break;
        if (id[i] == none) {
          const float *q = pos + 3 * (size_t)i;
          id[i] = weld.add((float)(q[0] - r.offset.x), (float)(q[1] - r.offset.y), (float)(q[2] - r.offset.z));
        }
        v[k] = id[i];
      }
      if (!ok || v[0] == v[1] || v[1] == v[2] || v[2] == v[0]) {
        r.dropped++;
        continue;
      }
      T.insert(T.end(), v, v + 3);
    }
    P.resize(fp.size() / 3);
    for (size_t i = 0; i < P.size(); i++) P[i] = {fp[3 * i], fp[3 * i + 1], fp[3 * i + 2]};
    if (T.empty()) return why = "scan: no triangles", false;
    return true;
  }

  // 3. Points a hair apart at open edges made one: each (in order) with the first before it that near (where a file's
  // pieces didn't quite meet). Triangles left with a corner twice go.
  void weldCracks() {
    std::vector<uint8_t> open(P.size(), 0);
    const auto &e = sidesByEdge();
    eachEdge(e, [&](size_t i, size_t j) {
      if (j - i == 1) open[T[e[i].second]] = open[T[nextSide(e[i].second)]] = 1;
    });
    double tol = std::max(1e-6 * size, std::ldexp(size, -21));
    if (!(tol > 0)) return;
    auto cell = [&](double x) { return (int64_t)std::floor(x / tol) + (1 << 20); };
    auto pack = [](int64_t x, int64_t y, int64_t z) { return (uint64_t)x << 42 | (uint64_t)y << 21 | (uint64_t)z; };
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells;  // (only looked up, never walked)
    std::vector<uint32_t> to(P.size());
    uint64_t welded = 0;
    for (uint32_t i = 0; i < P.size(); i++) {
      to[i] = i;
      if (!open[i]) continue;
      int64_t c[3] = {cell(P[i].x), cell(P[i].y), cell(P[i].z)};
      uint32_t best = none;
      for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
          for (int dz = -1; dz <= 1; dz++) {
            auto it = cells.find(pack(c[0] + dx, c[1] + dy, c[2] + dz));
            if (it == cells.end()) continue;
            for (uint32_t j : it->second)
              if (j < best && norm2(P[j] - P[i]) <= tol * tol) best = j;
          }
      if (best != none) to[i] = best, welded++;
      else cells[pack(c[0], c[1], c[2])].push_back(i);
    }
    if (!welded) return;
    r.welded += welded, edgesStale = true;
    std::vector<uint8_t> keep(triangles(), 1);
    for (uint32_t t = 0; t < triangles(); t++) {
      uint32_t *v = &T[3 * t];
      for (int k = 0; k < 3; k++) v[k] = to[v[k]];
      if (v[0] == v[1] || v[1] == v[2] || v[2] == v[0]) keep[t] = 0, r.dropped++;
    }
    keepOnly(keep);
  }

  // 4. A triangle there more than once kept once; one there both ways round (a wall of no thickness) gone both ways.
  void dropDuplicates() {
    struct Key {
      uint32_t v[3];
      bool odd;  // its corners not in their sorted order turned round
      uint32_t t;
    };
    std::vector<Key> keys(triangles());
    for (uint32_t t = 0; t < triangles(); t++) {
      const uint32_t *v = &T[3 * t];
      int m = v[0] < v[1] ? (v[0] < v[2] ? 0 : 2) : (v[1] < v[2] ? 1 : 2);
      uint32_t a = v[m], b = v[(m + 1) % 3], c = v[(m + 2) % 3];
      keys[t] = {{a, std::min(b, c), std::max(b, c)}, b > c, t};
    }
    std::sort(keys.begin(), keys.end(), [](const Key &x, const Key &y) {
      return std::tie(x.v[0], x.v[1], x.v[2], x.t) < std::tie(y.v[0], y.v[1], y.v[2], y.t);
    });
    std::vector<uint8_t> keep(triangles(), 1);
    bool any = false;
    for (size_t i = 0, j; i < keys.size(); i = j) {
      size_t odd = 0;
      for (j = i; j < keys.size() && std::equal(keys[j].v, keys[j].v + 3, keys[i].v); j++) odd += keys[j].odd;
      if (j - i == 1) continue;
      any = true;
      size_t even = j - i - odd;
      bool kept = false;
      for (size_t k = i; k < j; k++) {
        bool keepThis = !kept && even != odd && keys[k].odd == (odd > even);
        if (keepThis) kept = true;
        else keep[keys[k].t] = 0, r.duplicates++;
      }
    }
    if (any) edgesStale = true, keepOnly(keep);
  }

  // 5, 6. Each piece (triangles joined by edges two of them meet at) turned one way throughout, as few turned as can be.
  void orient() {
    std::vector<uint32_t> other(T.size(), none);
    const auto &e = sidesByEdge();
    eachEdge(e, [&](size_t i, size_t j) {
      if (j - i == 2) other[e[i].second] = e[i + 1].second, other[e[i + 1].second] = e[i].second;
    });
    uint32_t nt = triangles();
    pieceOf.assign(nt, none), twisted.clear();
    std::vector<uint8_t> flip(nt, 0);
    std::vector<uint32_t> queue;
    queue.reserve(nt);
    for (uint32_t seed = 0; seed < nt; seed++) {
      if (pieceOf[seed] != none) continue;
      uint32_t id = (uint32_t)twisted.size();
      twisted.push_back(0);
      size_t from = queue.size();
      queue.push_back(seed), pieceOf[seed] = id;
      for (size_t q = from; q < queue.size(); q++) {
        uint32_t t = queue[q];
        for (int k = 0; k < 3; k++) {
          uint32_t s = 3 * t + k, m = other[s];
          if (m == none) continue;
          uint32_t u = m / 3;
          uint8_t need = flip[t] ^ (T[s] == T[m]);
          if (pieceOf[u] == none) pieceOf[u] = id, flip[u] = need, queue.push_back(u);
          else if (flip[u] != need) twisted[id] = 1;
        }
      }
      size_t turned = 0;
      for (size_t q = from; q < queue.size(); q++) turned += flip[queue[q]];
      if (2 * turned > queue.size() - from)
        for (size_t q = from; q < queue.size(); q++) flip[queue[q]] ^= 1;
    }
    for (uint32_t t = 0; t < nt; t++) r.flipped += flip[t];
    turnRound(flip);
  }

  // 7. Loose open bits taken off: under 8 triangles, or under the given share of them and small beside the whole. The
  // largest part stays, as do closed ones however small.
  void dropIslands() {
    if (!o.removeIslands) return;
    uint32_t nt = triangles();
    Find part(nt);
    std::vector<uint8_t> openAt(nt, 0);
    const auto &e = sidesByEdge();
    eachEdge(e, [&](size_t i, size_t j) {
      if (j - i == 1) openAt[e[i].second / 3] = 1;
      for (size_t k = i + 1; k < j; k++) part.join(e[i].second / 3, e[k].second / 3);
    });
    std::vector<uint32_t> count(nt, 0);
    std::vector<uint8_t> open(nt, 0);
    std::vector<V3> lo(nt, V3{INFINITY, INFINITY, INFINITY}), hi(nt, V3{-INFINITY, -INFINITY, -INFINITY});
    V3 allLo{INFINITY, INFINITY, INFINITY}, allHi{-INFINITY, -INFINITY, -INFINITY};
    for (uint32_t t = 0; t < nt; t++) {
      uint32_t p = part(t);
      count[p]++, open[p] |= openAt[t];
      for (int k = 0; k < 3; k++) lo[p] = vmin(lo[p], P[T[3 * t + k]]), hi[p] = vmax(hi[p], P[T[3 * t + k]]);
      allLo = vmin(allLo, lo[p]), allHi = vmax(allHi, hi[p]);
    }
    uint32_t largest = 0;
    for (uint32_t p = 0; p < nt; p++)
      if (count[p] > count[largest]) largest = p;
    double whole = norm(allHi - allLo);
    std::vector<uint8_t> drop(nt, 0);
    bool any = false;
    for (uint32_t p = 0; p < nt; p++)
      if (count[p] && open[p] && p != largest &&
          (count[p] < 8 || (count[p] < o.islandShare * nt && norm(hi[p] - lo[p]) < 0.2 * whole)))
        drop[p] = 1, any = true;
    if (!any) return;
    std::vector<uint8_t> keep(nt, 1);
    for (uint32_t t = 0; t < nt; t++)
      if (drop[part(t)]) keep[t] = 0, r.islands++;
    keepOnly(keep);
  }

  // 8. Each piece turned to face out: an open one so that, closed by caps over its holes, it encloses more than
  // nothing; a closed one inside out likewise, unless it lies inside one facing out (a void).
  void outward() {
    size_t pieces = twisted.size();
    std::vector<V3> sum(pieces, V3{0, 0, 0}), lo(pieces, V3{INFINITY, INFINITY, INFINITY}), hi(pieces, V3{-INFINITY, -INFINITY, -INFINITY});
    std::vector<double> rim(pieces, 0), vol(pieces, 0);
    std::vector<uint8_t> closed(pieces, 1);
    const auto &e = sidesByEdge();
    eachEdge(e, [&](size_t i, size_t j) {
      if (j - i == 2 && T[e[i].second] != T[e[i + 1].second]) return;
      for (size_t k = i; k < j; k++) {
        uint32_t s = e[k].second, p = pieceOf[s / 3];
        closed[p] = 0, sum[p] += P[T[s]], rim[p]++;
      }
    });
    for (uint32_t t = 0; t < triangles(); t++)
      for (int k = 0; k < 3; k++) lo[pieceOf[t]] = vmin(lo[pieceOf[t]], P[T[3 * t + k]]), hi[pieceOf[t]] = vmax(hi[pieceOf[t]], P[T[3 * t + k]]);
    // (Measured from a point on its rim, the caps over its holes counted without being made.)
    std::vector<V3> from(pieces);
    for (size_t p = 0; p < pieces; p++) from[p] = rim[p] ? sum[p] / rim[p] : (lo[p] + hi[p]) * 0.5;
    for (uint32_t t = 0; t < triangles(); t++) {
      V3 base = from[pieceOf[t]], a = P[T[3 * t]] - base, b = P[T[3 * t + 1]] - base, c = P[T[3 * t + 2]] - base;
      vol[pieceOf[t]] += dot(a, cross(b, c));
    }
    std::vector<uint8_t> turn(pieces, 0);
    bool voids = false;
    for (size_t p = 0; p < pieces; p++) {
      if (!(vol[p] < 0)) continue;
      if (closed[p] && !twisted[p]) voids = true;
      else turn[p] = 1;
    }
    if (voids) {
      // Inside out and closed: a void where it lies inside the pieces facing out and closed.
      std::vector<uint32_t> outer;
      std::vector<uint32_t> firstOf(pieces, none);
      for (uint32_t t = 0; t < triangles(); t++) {
        uint32_t p = pieceOf[t];
        if (firstOf[p] == none) firstOf[p] = t;
        if (closed[p] && !twisted[p] && vol[p] > 0) outer.insert(outer.end(), {T[3 * t], T[3 * t + 1], T[3 * t + 2]});
      }
      MeshTree tree(P, outer);
      for (size_t p = 0; p < pieces; p++) {
        if (!(vol[p] < 0) || !closed[p] || twisted[p]) continue;
        uint32_t t = firstOf[p];
        V3 q = (P[T[3 * t]] + P[T[3 * t + 1]] + P[T[3 * t + 2]]) / 3;
        if (!(tree.winding(q) > 0.5)) turn[p] = 1;
      }
    }
    std::vector<uint8_t> turned(triangles(), 0);
    bool any = false;
    for (uint32_t t = 0; t < triangles(); t++)
      if (turn[pieceOf[t]]) turned[t] = 1, r.flipped++, any = true;
    if (any) turnRound(turned);
  }

  // 9. Each side joined to one running back along it: at an edge more than two triangles meet at, paired so each pair
  // bounds one wedge of material.
  void pairSides() {
    mate.assign(T.size(), none);
    const auto &e = sidesByEdge();
    std::vector<uint32_t> fwd, back;
    std::vector<std::pair<uint32_t, uint32_t>> pairs;
    eachEdge(e, [&](size_t i, size_t j) {
      if (j - i == 1) return;
      uint32_t u = (uint32_t)(e[i].first >> 32), v = (uint32_t)e[i].first;
      fwd.clear(), back.clear(), pairs.clear();
      for (size_t k = i; k < j; k++) (T[e[k].second] == u ? fwd : back).push_back(e[k].second);
      if (j - i > 2) r.crowded++;
      pairAround(P[u], P[v], fwd, back, [&](uint32_t s) { return P[T[lastSide(s)]]; }, pairs);
      for (auto &pr : pairs) mate[pr.first] = pr.second, mate[pr.second] = pr.first;
    });
  }

  // 10. A point a part touches itself at (separate fans of triangles round it) made one point per fan, so each edge
  // is between two triangles and each hole's rim passes each point once.
  void splitFans() {
    Find corner(T.size());
    for (uint32_t s = 0; s < T.size(); s++) {
      uint32_t m = mate[s];
      if (m == none || m < s) continue;
      corner.join(s, nextSide(m)), corner.join(nextSide(s), m);
    }
    std::vector<uint32_t> vertexOf(T.size(), none);
    std::vector<uint8_t> given(P.size(), 0);
    for (uint32_t c = 0; c < T.size(); c++) {
      uint32_t root = corner(c);
      if (vertexOf[root] == none) {
        uint32_t p = T[c];
        if (!given[p]) given[p] = 1, vertexOf[root] = p;
        else vertexOf[root] = (uint32_t)P.size(), P.push_back(P[p]);
      }
      T[c] = vertexOf[root];
    }
    edgesStale = true, edges = {};
  }

  // 11. Each hole closed: a triangle as it is; up to 300 sides by the triangles of least area (and shortest sides),
  // no edge made that's there already; more, in the plane it most nearly lies in; where neither will do, a fan from its
  // middle.
  std::vector<uint32_t> start, around;  // the triangles at each point
  std::vector<int> at;                  // a point's place round the hole being filled
  bool fillHoles() {
    std::vector<uint32_t> out(P.size(), none);
    size_t open = 0;
    for (uint32_t s = 0; s < T.size(); s++)
      if (mate[s] == none) {
        open++;
        if (out[T[s]] != none) return why = "its holes are tangled", false;
        out[T[s]] = s;
      }
    if (!open) return true;
    if (!o.fillHoles) return why = "it has holes", false;
    size_t before = T.size();
    start.assign(P.size() + 1, 0), around.resize(T.size());
    for (uint32_t v : T) start[v + 1]++;
    for (size_t i = 0; i < P.size(); i++) start[i + 1] += start[i];
    {
      std::vector<uint32_t> fill(start.begin(), start.end() - 1);
      for (uint32_t c = 0; c < T.size(); c++) around[fill[T[c]]++] = c / 3;
    }
    at.assign(P.size(), -1);
    std::vector<uint8_t> done(before, 0);
    std::vector<uint32_t> loop;
    for (uint32_t s = 0; s < before; s++) {
      if (mate[s] != none || done[s]) continue;
      loop.clear();
      uint32_t c = s;
      while (c != none && !done[c]) done[c] = 1, loop.push_back(T[c]), c = out[T[nextSide(c)]];
      if (c != s) return why = "its holes are tangled", false;
      // (The rim runs one way; the patch the other.)
      std::reverse(loop.begin(), loop.end());
      fill(loop);
      r.holes++, r.largestHole = std::max<uint64_t>(r.largestHole, loop.size());
      if ((r.holes & 255) == 0 && !going(0.6)) return false;
    }
    return true;
  }
  void add(uint32_t a, uint32_t b, uint32_t c) { T.push_back(a), T.push_back(b), T.push_back(c); }
  void fill(const std::vector<uint32_t> &w) {
    size_t n = w.size();
    if (n == 3) return add(w[0], w[1], w[2]);
    for (size_t i = 0; i < n; i++) at[w[i]] = (int)i;
    // The edges already there between the hole's points.
    std::vector<uint64_t> there;
    for (size_t i = 0; i < n; i++)
      for (uint32_t k = start[w[i]]; k < start[w[i] + 1]; k++)
        for (int c = 0; c < 3; c++) {
          int j = at[T[3 * around[k] + c]];
          if (j > (int)i) there.push_back((uint64_t)i << 32 | (uint32_t)j);
        }
    std::sort(there.begin(), there.end());
    auto exists = [&](size_t i, size_t j) {
      if (i > j) std::swap(i, j);
      return std::binary_search(there.begin(), there.end(), (uint64_t)i << 32 | j);
    };
    bool ok = n <= 300 ? leastArea(w, exists) : flat(w, exists);
    if (!ok) fan(w);
    for (size_t i = 0; i < n; i++) at[w[i]] = -1;
  }
  template <class Exists> bool leastArea(const std::vector<uint32_t> &w, Exists exists) {
    size_t n = w.size();
    std::vector<double> cost(n * n, INFINITY);
    std::vector<uint16_t> pick(n * n, 0);
    for (size_t i = 0; i + 1 < n; i++) cost[i * n + i + 1] = 0;
    for (size_t len = 2; len < n; len++)
      for (size_t i = 0, j = len; j < n; i++, j++) {
        if (!(i == 0 && j == n - 1) && exists(i, j)) continue;
        V3 a = P[w[i]], c = P[w[j]];
        double best = INFINITY;
        for (size_t k = i + 1; k < j; k++) {
          double sub = cost[i * n + k] + cost[k * n + j];
          if (!(sub < INFINITY)) continue;
          V3 b = P[w[k]];
          double weight = norm(cross(b - a, c - a)) + 0.1 * (norm2(b - a) + norm2(c - b) + norm2(a - c)) + sub;
          if (weight < best) best = weight, pick[i * n + j] = (uint16_t)k;
        }
        cost[i * n + j] = best;
      }
    if (!(cost[n - 1] < INFINITY)) return false;
    std::vector<std::pair<size_t, size_t>> stack{{0, n - 1}};
    while (!stack.empty()) {
      auto [i, j] = stack.back();
      stack.pop_back();
      if (j - i < 2) continue;
      size_t k = pick[i * n + j];
      add(w[i], w[k], w[j]);
      stack.push_back({k, j}), stack.push_back({i, k});
    }
    return true;
  }
  template <class Exists> bool flat(const std::vector<uint32_t> &w, Exists exists) {
    size_t n = w.size();
    V3 N{0, 0, 0}, c{0, 0, 0};
    for (size_t i = 0; i < n; i++) N += cross(P[w[i]], P[w[(i + 1) % n]]), c += P[w[i]];
    c = c / (double)n;
    if (!(norm(N) > 0)) return false;
    V3 z = unit(N), x = unit(std::fabs(z.x) < 0.9 ? cross(z, V3{1, 0, 0}) : cross(z, V3{0, 1, 0})), y = cross(z, x);
    std::vector<double> u(n), v(n);
    double reach = 0;
    for (size_t i = 0; i < n; i++) {
      V3 d = P[w[i]] - c;
      u[i] = dot(d, x), v[i] = dot(d, y), reach = std::max({reach, std::fabs(u[i]), std::fabs(v[i])});
    }
    double R = 10 * reach + 1;
    Tri2 tri(-2 * R, -R, 2 * R, -R, 0, 2 * R);
    for (size_t i = 0; i < n; i++)
      if (tri.insert(u[i], v[i]) != (int)(3 + i)) return false;
    for (size_t i = 0; i < n; i++)
      if (!tri.keep((int)(3 + i), (int)(3 + (i + 1) % n))) return false;
    if (tri.fallbacks || tri.detours || !tri.made().empty() || tri.count() != (int)(3 + n)) return false;
    std::vector<int> in = tri.insideKept();
    if (in.size() != 3 * (n - 2)) return false;
    for (size_t k = 0; k < in.size(); k += 3)
      for (int e = 0; e < 3; e++) {
        size_t a = in[k + e] - 3, b = in[k + (e + 1) % 3] - 3;
        if ((a + 1) % n != b && (b + 1) % n != a && exists(a, b)) return false;
      }
    for (size_t k = 0; k < in.size(); k += 3) add(w[in[k] - 3], w[in[k + 1] - 3], w[in[k + 2] - 3]);
    return true;
  }
  void fan(const std::vector<uint32_t> &w) {
    V3 c{0, 0, 0};
    for (uint32_t v : w) c += P[v];
    uint32_t mid = (uint32_t)P.size();
    P.push_back(fl(c / (double)w.size()));
    for (size_t i = 0; i < w.size(); i++) add(w[i], w[(i + 1) % w.size()], mid);
  }

  // 12. Triangles of no area (corners in a line, as a hole's patch over a straight rim makes) done away with.
  void cleanFlat() {
    bool any = false;
    for (uint32_t t = 0; t < triangles() && !any; t++) any = norm2(cross(P[T[3 * t + 1]] - P[T[3 * t]], P[T[3 * t + 2]] - P[T[3 * t]])) == 0;
    if (!any) return;
    Welded w;
    w.pts = std::move(P), w.tri = std::move(T);
    w.face.assign(w.tri.size() / 3, 0), w.nrm.assign(w.tri.size(), V3{0, 0, 1});
    clean(w, 1e-7 * size);
    P = std::move(w.pts), T = std::move(w.tri);
    for (V3 &q : P) q = fl(q);
  }

  // 13. As bk_mesh_shape will check it: closed, facing out, voids inside; and not passing through itself (unless it's
  // to be simplified, and checked for that after).
  bool sound(std::string &w, std::vector<uint8_t> &crossed, bool crossings = true) {
    if (!meshModel(P, T, w, true)) return false;
    if (crossings && selfCrossings(P, T, crossed)) return w = "it passes through itself", false;
    return true;
  }

  // The points in use, in order, as floats; what it came to.
  void finish(std::vector<float> &outPos, std::vector<uint32_t> &outTri) {
    std::vector<uint32_t> id(P.size(), none);
    outPos.clear(), outTri.resize(T.size());
    V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
    for (size_t c = 0; c < T.size(); c++) {
      uint32_t v = T[c];
      if (id[v] == none) {
        id[v] = (uint32_t)(outPos.size() / 3);
        outPos.push_back((float)P[v].x), outPos.push_back((float)P[v].y), outPos.push_back((float)P[v].z);
        lo = vmin(lo, P[v]), hi = vmax(hi, P[v]);
      }
      outTri[c] = id[v];
    }
    r.trianglesOut = T.size() / 3, r.pointsOut = outPos.size() / 3, r.size = hi - lo;
    double vol = 0, area = 0;
    for (uint32_t t = 0; t < triangles(); t++) {
      V3 a = P[T[3 * t]], b = P[T[3 * t + 1]], c = P[T[3 * t + 2]];
      vol += dot(a, cross(b, c)), area += norm(cross(b - a, c - a)) / 2;
    }
    r.volume = vol / 6;
    // The detail to sculpt at: its triangles' middle side, within reason, and coarse enough for the sculptor's most.
    std::vector<double> len;
    size_t step = std::max<size_t>(1, T.size() / 200000);
    for (size_t s = 0; s < T.size(); s += step) len.push_back(norm(P[T[nextSide((uint32_t)s)]] - P[T[s]]));
    std::nth_element(len.begin(), len.begin() + len.size() / 2, len.end());
    r.detail = std::max(std::min(std::max(len[len.size() / 2], 0.05), 20.0), std::sqrt(6 * area / 1.4e6));
  }
};

}  // namespace

namespace {

enum class Ending { sound, crossed, open, stopped };

// Steps 3 to 13 (where they come out sound, with the triangles that pass through others where that's why not).
Ending mend(Repair &f, std::string &w, std::vector<uint8_t> &crossed, bool crossings = true) {
  f.weldCracks();
  f.dropDuplicates();
  if (!f.triangles()) return w = "nothing's left", Ending::open;
  if (!f.going(0.2)) return Ending::stopped;
  f.orient();
  f.dropIslands();
  f.outward();
  if (!f.going(0.3)) return Ending::stopped;
  f.pairSides();
  f.splitFans();
  if (!f.going(0.4)) return Ending::stopped;
  if (!f.fillHoles()) return f.why == "scan: stopped" ? Ending::stopped : (w = f.why, Ending::open);
  f.cleanFlat();
  if (!f.going(0.5)) return Ending::stopped;
  if (f.sound(w, crossed, crossings)) return Ending::sound;
  return w == "it passes through itself" ? Ending::crossed : Ending::open;
}

// A mesh made again (about the first one's middle) put through the steps afresh, mended no further: sound, with what
// it came to.
bool afresh(const std::vector<V3> &pts, const std::vector<uint32_t> &tris, const ScanOptions &options, double from, double to,
            std::vector<float> &outPos, std::vector<uint32_t> &outTri, ScanReport &got, std::string &why) {
  std::vector<float> p(3 * pts.size());
  for (size_t i = 0; i < pts.size(); i++) p[3 * i] = (float)pts[i].x, p[3 * i + 1] = (float)pts[i].y, p[3 * i + 2] = (float)pts[i].z;
  ScanOptions o = options;
  if (options.progress) o.progress = [&](double x) { return options.progress(from + (to - from) * x); };
  got = ScanReport();
  Repair g(o, got, why);
  if (!g.take(p.data(), pts.size(), tris.data(), tris.size() / 3)) return false;
  std::string w;
  std::vector<uint8_t> crossed;
  if (mend(g, w, crossed) != Ending::sound) return false;
  g.finish(outPos, outTri);
  return true;
}

// The spacing of a grid to make a mesh again on: about its triangles' size, coarse enough for the sculptor's most and
// for a grid of at most 300 million points.
double gridDetail(const Repair &f) {
  double area = 0;
  std::vector<double> len;
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (uint32_t t = 0; t < f.triangles(); t++) {
    V3 a = f.P[f.T[3 * t]], b = f.P[f.T[3 * t + 1]], c = f.P[f.T[3 * t + 2]];
    area += norm(cross(b - a, c - a)) / 2;
    if (t % 7 == 0) len.push_back(norm(b - a));
    lo = vmin(vmin(lo, a), vmin(b, c)), hi = vmax(vmax(hi, a), vmax(b, c));
  }
  std::nth_element(len.begin(), len.begin() + len.size() / 2, len.end());
  V3 ext = hi - lo;
  return std::max({len[len.size() / 2], std::sqrt(6 * area / 1.4e6), std::cbrt((ext.x + 1e-9) * (ext.y + 1e-9) * (ext.z + 1e-9) / 3e8), 1e-3});
}

}  // namespace

bool solidify(const std::vector<V3> &P, const std::vector<uint32_t> &T, double h, std::vector<V3> &pts, std::vector<uint32_t> &tris, std::string &why) {
  pts.clear(), tris.clear();
  if (T.empty() || !(h > 0)) return why = "nothing inside", false;
  MeshTree tree(P, T);
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (uint32_t v : T) lo = vmin(lo, P[v]), hi = vmax(hi, P[v]);
  int n[3];
  for (int k = 0; k < 3; k++) n[k] = (int)std::ceil((hi[k] - lo[k]) / h) + 6;
  double band = 2 * h;
  auto field = [&](V3 q) {
    MeshTree::Near near = tree.nearest(q, band * band);
    double d = near.tri == UINT32_MAX ? band : std::sqrt(near.d2);
    return tree.winding(q) > 0.5 ? -d : d;
  };
  // (A block nowhere near the surface, well inside or well outside: all one way.)
  auto block = [&](V3 blo, V3 bhi) {
    V3 c = (blo + bhi) * 0.5;
    double r = norm(bhi - blo) * 0.5;
    if (tree.nearest(c, r * r).tri != UINT32_MAX) return 0;
    double w = tree.winding(c);
    return w > 0.9 ? -1 : w < 0.1 ? 1 : 0;
  };
  bool uncrossed = false;
  return isoSurface((lo + hi) * 0.5, h, n, field, block, 2, pts, tris, why, 1500000, 1, nullptr, &uncrossed);
}

namespace {

// The triangles passing through others cut out with `rings` rings of triangles round them.
void cutOut(const Repair &f, const std::vector<uint8_t> &crossed, int rings, std::vector<uint32_t> &tris) {
  std::vector<uint8_t> cut(crossed), at(f.P.size(), 0);
  for (int ring = 0; ring < rings; ring++) {
    for (uint32_t t = 0; t < f.triangles(); t++)
      if (cut[t]) at[f.T[3 * t]] = at[f.T[3 * t + 1]] = at[f.T[3 * t + 2]] = 1;
    for (uint32_t t = 0; t < f.triangles(); t++)
      if (at[f.T[3 * t]] || at[f.T[3 * t + 1]] || at[f.T[3 * t + 2]]) cut[t] = 1;
  }
  tris.clear();
  for (uint32_t t = 0; t < f.triangles(); t++)
    if (!cut[t]) tris.insert(tris.end(), {f.T[3 * t], f.T[3 * t + 1], f.T[3 * t + 2]});
}

}  // namespace

namespace {

// Simplified where it's more than the options allow (the mesh sound, about the origin), put through the steps again; where
// it then isn't sound, made again on a grid fine enough to be within the most.
bool lighter(std::vector<float> &outPos, std::vector<uint32_t> &outTri, const ScanOptions &options, ScanReport &report, std::string &why) {
  size_t nt = outTri.size() / 3;
  if (!options.maxTriangles || nt <= options.maxTriangles) return true;
  std::vector<V3> P(outPos.size() / 3), full;
  for (size_t i = 0; i < P.size(); i++) P[i] = {outPos[3 * i], outPos[3 * i + 1], outPos[3 * i + 2]};
  std::vector<uint32_t> T = outTri;
  // (How far it strays: measured at up to 100 000 of the full surface's points.)
  for (size_t i = 0, step = std::max<size_t>(1, P.size() / 100000); i < P.size(); i += step) full.push_back(P[i]);
  auto going = [&](double x) { return !options.progress || options.progress(0.9 + 0.05 * x); };
  if (!simplify(P, T, (size_t)options.maxTriangles, going)) return why = "scan: stopped", false;
  ScanOptions o = options;
  o.maxTriangles = 0;
  ScanReport got;
  std::string tried;
  std::vector<float> pos;
  std::vector<uint32_t> tri;
  // (Collapsing edges keeps it closed and facing out: what's left to check is whether it passes through itself.)
  bool ok;
  {
    Repair h(o, got, tried);
    h.P = std::move(P), h.T = std::move(T);
    std::vector<uint8_t> crossed;
    ok = h.sound(tried, crossed);
    if (ok) h.finish(pos, tri);
  }
  if (!ok) {
    // (Made again on a grid: its triangles about as many as the most, at about 1.4 per square of the spacing.)
    double area = 0;
    std::vector<V3> FP(outPos.size() / 3);
    for (size_t i = 0; i < FP.size(); i++) FP[i] = {outPos[3 * i], outPos[3 * i + 1], outPos[3 * i + 2]};
    for (size_t t = 0; t < nt; t++) {
      V3 a = FP[outTri[3 * t]], b = FP[outTri[3 * t + 1]], c = FP[outTri[3 * t + 2]];
      area += norm(cross(b - a, c - a)) / 2;
    }
    double d = std::sqrt(2.2 * area / (double)options.maxTriangles);
    std::vector<V3> pts;
    std::vector<uint32_t> tris;
    ok = remesh(FP, outTri, d, pts, tris, tried, (size_t)options.maxTriangles) && afresh(pts, tris, o, 0.95, 0.99, pos, tri, got, tried);
    if (!ok) return why = tried == "scan: stopped" ? tried : "scan: can't be made lighter: " + tried, false;
    report.remade = 3, report.remadeDetail = d;
    P.clear();
    for (size_t i = 0; i < pos.size(); i += 3) P.push_back({pos[i], pos[i + 1], pos[i + 2]});
  }
  if (!report.simplifiedFrom) report.simplifiedFrom = nt;
  report.offset = report.offset + got.offset;
  report.trianglesOut = got.trianglesOut, report.pointsOut = got.pointsOut, report.size = got.size, report.volume = got.volume;
  report.detail = got.detail;
  // (Its points as afresh put them: about its own middle, a little off the full surface's.)
  std::vector<V3> moved(pos.size() / 3);
  for (size_t i = 0; i < moved.size(); i++) moved[i] = V3{pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]} + got.offset;
  report.deviation += deviation(full, moved, tri);
  outPos = std::move(pos), outTri = std::move(tri);
  return true;
}

}  // namespace

bool repairScan(const float *pos, size_t points, const uint32_t *tri, size_t triangles, const ScanOptions &options, std::vector<float> &outPos,
                std::vector<uint32_t> &outTri, ScanReport &report, std::string &why) {
  outPos.clear(), outTri.clear();
  report = ScanReport();
  report.trianglesIn = triangles, report.pointsIn = points;
  Repair f(options, report, why);
  if (!f.take(pos, points, tri, triangles) || !f.going(0.1)) return false;
  // Too large to work on as it is (and more than may be kept): made coarser at once, on a grid that leaves about 4
  // million triangles.
  if (options.maxTriangles && f.triangles() > std::max<uint64_t>(8000000, options.maxTriangles)) {
    double area = 0;
    for (uint32_t t = 0; t < f.triangles(); t++) {
      V3 a = f.P[f.T[3 * t]], b = f.P[f.T[3 * t + 1]], c = f.P[f.T[3 * t + 2]];
      area += norm(cross(b - a, c - a)) / 2;
    }
    double cell = std::sqrt(area / 2e6);
    cluster(f.P, f.T, cell);
    report.simplifiedFrom = f.triangles() ? triangles : 0, report.deviation = cell * std::sqrt(3.0) / 2;
    if (!f.triangles()) return why = "scan: no triangles", false;
  }
  std::string w;
  std::vector<uint8_t> crossed;
  // (To be simplified: whether it passes through itself is checked after, and mended then.)
  bool heavy = options.maxTriangles && f.triangles() > options.maxTriangles;
  Ending end = mend(f, w, crossed, !heavy);
  if (end == Ending::stopped) return false;
  if (end == Ending::sound) {
    f.finish(outPos, outTri);
    return lighter(outPos, outTri, options, report, why) && f.going(1);
  }
  if (!options.remake) return why = "scan: can't be closed: " + w, false;
  // Mended further, the first way that comes out sound: what it came to taken from there.
  ScanReport got;
  std::string tried;
  auto use = [&](int remade, double detail) {
    report.remade = remade, report.remadeDetail = detail;
    report.offset = report.offset + got.offset;
    report.trianglesOut = got.trianglesOut, report.pointsOut = got.pointsOut, report.size = got.size, report.volume = got.volume;
    report.detail = got.detail;
    return lighter(outPos, outTri, options, report, why) && f.going(1);
  };
  std::vector<V3> pts;
  std::vector<uint32_t> tris;
  if (end == Ending::crossed) {
    // A fold: what passes through the rest cut out with a ring or more round it, and the holes filled again.
    size_t marked = 0;
    for (uint8_t c : crossed) marked += c;
    // (Only where that leaves about the volume it had: parts overlapping each lose a slice where they meet, and
    // whatever's left of them is no fold mended.)
    double before = 0;
    for (uint32_t t = 0; t < f.triangles(); t++) before += dot(f.P[f.T[3 * t]], cross(f.P[f.T[3 * t + 1]], f.P[f.T[3 * t + 2]])) / 6;
    if (marked <= std::max<size_t>(2000, f.triangles() / 100))
      for (int rings = 1; rings <= 3; rings++) {
        cutOut(f, crossed, rings, tris);
        if (afresh(f.P, tris, options, 0.5, 0.6, outPos, outTri, got, tried) && std::fabs(got.volume - before) <= 0.05 * std::fabs(before))
          return use(1, 0);
        if (tried == "scan: stopped") return why = tried, false;
      }
    // Parts overlapping (or a fold too large): what they enclose together.
    if (f.triangles() <= 1000000) {
      std::string no;
      auto m = meshModel(f.P, f.T, no, true);
      if (m) {
        Solid u = resolved(*m->mesh);
        if (!u.tri.empty() && afresh(u.p, u.tri, options, 0.6, 0.75, outPos, outTri, got, tried)) return use(2, 0);
        if (tried == "scan: stopped") return why = tried, false;
      }
    }
    // Made again on a grid: what it encloses.
    double d = gridDetail(f);
    if (remesh(f.P, f.T, d, pts, tris, tried) && afresh(pts, tris, options, 0.75, 0.85, outPos, outTri, got, tried)) return use(3, d);
    if (tried == "scan: stopped") return why = tried, false;
  }
  // Made solid on a grid: inside where it wraps round more than halfway.
  double d = gridDetail(f);
  if (solidify(f.P, f.T, d, pts, tris, tried) && afresh(pts, tris, options, 0.85, 0.99, outPos, outTri, got, tried)) return use(4, d);
  if (tried == "scan: stopped") return why = tried, false;
  outPos.clear(), outTri.clear();
  return why = "scan: can't be closed: " + w, false;
}

}  // namespace bce
