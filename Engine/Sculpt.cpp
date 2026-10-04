// Sculpted bodies: a closed mesh made again at a detail. What it encloses is found on a grid by counting, along each of
// the grid's lines, where the line crosses the mesh (exactly, so a line grazing a side or a corner is counted right); the
// new surface is marching cubes on that, its points where the lines cross the mesh, then evened out.
#include "Engine/Sculpt.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_map>

namespace bce {

namespace {

// The sign of the turn a → b → q, with q moved off every line through two points by an infinitely small step along +u
// and then a smaller one along +v: never 0 for a ≠ b, so a point is inside or outside every triangle, never on its side,
// and a line through a closed mesh crosses it as often as it should wherever it grazes a side or a corner.
int turn(double au, double av, double bu, double bv, double qu, double qv) {
  int s = orient2d(au, av, bu, bv, qu, qv);
  if (s) return s;
  // (The turn to q + (e, e²) is (bu − au)·e² − (bv − av)·e.)
  if (bv != av) return bv > av ? -1 : 1;
  return bu > au ? 1 : bu < au ? -1 : 0;
}

struct Crossing {
  double at;
  int sign;  // +1 where a line running up its axis goes in (the mesh facing down the axis there), -1 where it comes out
};

struct Grid {
  V3 o;
  double h;
  int n[3];
  size_t index(int i, int j, int k) const { return (size_t)i + (size_t)n[0] * ((size_t)j + (size_t)n[1] * (size_t)k); }
  double at(int axis, int i) const { return o[axis] + i * h; }
};

// The grid's lines along axis a (one through each grid point of the plane across it, by its steps j along b = a + 1 and
// k along c = a + 2, round): where each crosses the mesh, in order, and the winding number past each crossing.
struct Lines {
  int a = 0, b = 1, c = 2;
  std::vector<uint32_t> start;  // per line, into list (one more at the end)
  std::vector<Crossing> list;
  std::vector<int> wind;
  size_t line(int j, int k, const Grid &g) const { return (size_t)j + (size_t)g.n[b] * (size_t)k; }
};

Lines cast(const std::vector<V3> &P, const std::vector<uint32_t> &T, const Grid &g, int a) {
  Lines L;
  L.a = a, L.b = (a + 1) % 3, L.c = (a + 2) % 3;
  int b = L.b, c = L.c;
  size_t lines = (size_t)g.n[b] * g.n[c];
  struct Hit {
    uint32_t line;
    Crossing x;
  };
  std::vector<Hit> hits;
  for (size_t t = 0; t + 2 < T.size(); t += 3) {
    V3 A = P[T[t]], B = P[T[t + 1]], C = P[T[t + 2]];
    int s = orient2d(A[b], A[c], B[b], B[c], C[b], C[c]);
    // Edge-on to the lines: crossed by none (every line moved off it as above).
    if (!s) continue;
    // Facing up the axis (its normal's part along a has s's sign) a line comes out there; facing down it, it goes in.
    int sign = -s;
    if (s < 0) std::swap(B, C);
    V3 nrm = cross(B - A, C - A);
    double lb = std::min({A[b], B[b], C[b]}), hb = std::max({A[b], B[b], C[b]}), lc = std::min({A[c], B[c], C[c]}), hc = std::max({A[c], B[c], C[c]});
    // (A step wider each way than its box: a line just past it by rounding is still tested.)
    int j0 = std::max(0, (int)std::floor((lb - g.o[b]) / g.h) - 1), j1 = std::min(g.n[b] - 1, (int)std::ceil((hb - g.o[b]) / g.h) + 1);
    int k0 = std::max(0, (int)std::floor((lc - g.o[c]) / g.h) - 1), k1 = std::min(g.n[c] - 1, (int)std::ceil((hc - g.o[c]) / g.h) + 1);
    for (int k = k0; k <= k1; k++)
      for (int j = j0; j <= j1; j++) {
        double qb = g.at(b, j), qc = g.at(c, k);
        if (turn(A[b], A[c], B[b], B[c], qb, qc) <= 0 || turn(B[b], B[c], C[b], C[c], qb, qc) <= 0 || turn(C[b], C[c], A[b], A[c], qb, qc) <= 0) continue;
        double at = A[a] - (nrm[b] * (qb - A[b]) + nrm[c] * (qc - A[c])) / nrm[a];
        hits.push_back({(uint32_t)L.line(j, k, g), {at, sign}});
      }
  }
  L.start.assign(lines + 1, 0);
  for (const Hit &x : hits) L.start[x.line + 1]++;
  for (size_t i = 0; i < lines; i++) L.start[i + 1] += L.start[i];
  L.list.resize(hits.size());
  {
    std::vector<uint32_t> fill(L.start.begin(), L.start.end() - 1);
    for (const Hit &x : hits) L.list[fill[x.line]++] = x.x;
  }
  L.wind.resize(L.list.size());
  for (size_t i = 0; i < lines; i++) {
    std::sort(L.list.begin() + L.start[i], L.list.begin() + L.start[i + 1],
              [](const Crossing &x, const Crossing &y) { return x.at != y.at ? x.at < y.at : x.sign < y.sign; });
    int w = 0;
    for (uint32_t p = L.start[i]; p < L.start[i + 1]; p++) L.wind[p] = w += L.list[p].sign;
  }
  return L;
}

// Where the grid's edge from point (i, j, k) one step up the lines' axis crosses the surface, as a fraction of the step:
// the first crossing on it where the inside changes the way it does from end to end (to `farIn` at the far end); the
// middle if none does (the lines along other axes may differ by a rounding where the surface meets the edge's end).
// Kept a little off the ends, so no triangle comes out with no area.
double crossingOn(const Lines &L, const Grid &g, int i, int j, int k, bool farIn) {
  int at[3] = {i, j, k};
  size_t line = L.line(at[L.b], at[L.c], g);
  double x0 = g.at(L.a, at[L.a]), x1 = x0 + g.h;
  uint32_t p0 = L.start[line], p1 = L.start[line + 1];
  uint32_t p = (uint32_t)(std::lower_bound(L.list.begin() + p0, L.list.begin() + p1, x0, [](const Crossing &x, double v) { return x.at < v; }) - L.list.begin());
  double f = 0.5;
  for (; p < p1 && L.list[p].at < x1; p++) {
    bool before = (p > p0 ? L.wind[p - 1] : 0) > 0, after = L.wind[p] > 0;
    if (before != after && after == farIn) {
      f = (L.list[p].at - x0) / g.h;
      break;
    }
  }
  return std::min(0.98, std::max(0.02, f));
}

// A cube's corners (bit 0: one step along x, bit 1 along y, bit 2 along z), its 12 edges (by axis and lower corner) and
// its 6 faces, each with its corners counter-clockwise seen from outside the cube and the edges between them in turn.
struct Cube {
  int edgeAxis[12], edgeLow[12], edgeOf[8][3];
  struct Face {
    int corner[4], edge[4];
  } face[6];
  Cube() {
    int n = 0;
    for (int c = 0; c < 8; c++)
      for (int a = 0; a < 3; a++) {
        edgeOf[c][a] = -1;
        if (!(c >> a & 1)) edgeAxis[n] = a, edgeLow[n] = c, edgeOf[c][a] = n++;
      }
    for (int a = 0, f = 0; a < 3; a++)
      for (int s = 0; s < 2; s++, f++) {
        int u = (a + 1) % 3, v = (a + 2) % 3;
        if (!s) std::swap(u, v);  // (facing down the axis: the other way round)
        const int du[4] = {0, 1, 1, 0}, dv[4] = {0, 0, 1, 1};
        for (int m = 0; m < 4; m++) face[f].corner[m] = s << a | du[m] << u | dv[m] << v;
        for (int m = 0; m < 4; m++) {
          int c0 = face[f].corner[m], c1 = face[f].corner[(m + 1) % 4], diff = c0 ^ c1, axis = diff == 1 ? 0 : diff == 2 ? 1 : 2;
          face[f].edge[m] = edgeOf[std::min(c0, c1)][axis];
        }
      }
  }
};

// Inside or not at each grid point, by the lines along x (bit per point, in the grid's order): how many are.
size_t label(const Grid &g, const Lines &L, std::vector<uint64_t> &in) {
  in.assign(((size_t)g.n[0] * g.n[1] * g.n[2] + 63) / 64, 0);
  size_t inner = 0;
  for (int k = 0; k < g.n[2]; k++)
    for (int j = 0; j < g.n[1]; j++) {
      size_t line = L.line(j, k, g);
      uint32_t p = L.start[line], end = L.start[line + 1];
      int w = 0;
      for (int i = 0; i < g.n[0]; i++) {
        double x = g.at(0, i);
        while (p < end && L.list[p].at < x) w = L.wind[p++];
        if (w > 0) {
          size_t id = g.index(i, j, k);
          in[id >> 6] |= 1ull << (id & 63), inner++;
        }
      }
    }
  return inner;
}

// Marching cubes over grid points labelled inside or not (inside(i, j, k)), a surface of triangles facing out of the
// inside, its points where fraction(i, j, k, axis, farIn) puts them along each grid edge it crosses (from the edge's
// lower end, as a part of the step). On each face of a cube, the surface cuts off each run of inside corners (taken
// counter-clockwise seen from outside the cube) by a side from where it goes in to where it comes out; inside corners
// diagonal on a face stay apart. A face's choice is the same seen from both cubes, so the pieces meet, and each crossing
// point (one per grid edge) is where one side ends and the next begins: they close into loops, each a polygon of the
// surface (always closed).
template <class Inside, class Fraction>
void march(const Grid &g, Inside inside, Fraction fraction, size_t reserve, std::vector<V3> &outPts, std::vector<uint32_t> &outTris) {
  static const Cube cube;
  auto corners = [&](int i, int j, int k) {
    int m = 0;
    for (int c = 0; c < 8; c++) m |= (int)inside(i + (c & 1), j + (c >> 1 & 1), k + (c >> 2 & 1)) << c;
    return m;
  };
  std::unordered_map<uint64_t, uint32_t> pointOf;  // per grid edge crossed (its axis and lower end), its point
  pointOf.reserve(reserve);
  auto point = [&](int i, int j, int k, int e) {
    int a = cube.edgeAxis[e], c = cube.edgeLow[e];
    int at[3] = {i + (c & 1), j + (c >> 1 & 1), k + (c >> 2 & 1)};
    uint64_t key = (uint64_t)a << 62 | g.index(at[0], at[1], at[2]);
    auto found = pointOf.find(key);
    if (found != pointOf.end()) return found->second;
    int up[3] = {at[0], at[1], at[2]};
    up[a]++;
    double f = fraction(at[0], at[1], at[2], a, inside(up[0], up[1], up[2]));
    V3 q{g.at(0, at[0]), g.at(1, at[1]), g.at(2, at[2])};
    q[a] += f * g.h;
    uint32_t id = (uint32_t)outPts.size();
    outPts.push_back(q);
    pointOf.emplace(key, id);
    return id;
  };
  for (int k = 0; k + 1 < g.n[2]; k++)
    for (int j = 0; j + 1 < g.n[1]; j++)
      for (int i = 0; i + 1 < g.n[0]; i++) {
        int m = corners(i, j, k);
        if (m == 0 || m == 255) continue;
        int next[12];
        std::fill(next, next + 12, -1);
        for (const auto &f : cube.face) {
          bool c[4];
          for (int q = 0; q < 4; q++) c[q] = m >> f.corner[q] & 1;
          for (int q = 0; q < 4; q++) {
            // Going in across edge q (corner q out, the next in): the run ends where it next comes out.
            if (c[q] || !c[(q + 1) % 4]) continue;
            int r = (q + 1) % 4;
            while (!(c[r] && !c[(r + 1) % 4])) r = (r + 1) % 4;
            next[f.edge[q]] = f.edge[r];
          }
        }
        bool done[12] = {false};
        for (int e = 0; e < 12; e++) {
          if (next[e] < 0 || done[e]) continue;
          uint32_t loop[12];
          int n = 0;
          for (int x = e; !done[x]; x = next[x]) done[x] = true, loop[n++] = point(i, j, k, x);
          if (n == 3) {
            outTris.insert(outTris.end(), {loop[0], loop[1], loop[2]});
            continue;
          }
          // A polygon of more sides: a point at its middle, a triangle to each side (no two polygons ever share a
          // diagonal this way).
          V3 mid{0, 0, 0};
          for (int q = 0; q < n; q++) mid += outPts[loop[q]];
          uint32_t centre = (uint32_t)outPts.size();
          outPts.push_back(mid / n);
          for (int q = 0; q < n; q++) outTris.insert(outTris.end(), {centre, loop[q], loop[(q + 1) % n]});
        }
      }
}

// Evened out: each point moved halfway to the middle of its neighbours, along the surface (not across it), `passes` times.
void relax(std::vector<V3> &outPts, const std::vector<uint32_t> &outTris, int passes) {
  size_t np = outPts.size();
  std::vector<uint32_t> start(np + 1, 0), nb;
  for (size_t t = 0; t < outTris.size(); t += 3)
    for (int q = 0; q < 3; q++) start[outTris[t + q] + 1]++;
  for (size_t i = 0; i < np; i++) start[i + 1] += start[i];
  nb.resize(start[np]);
  {
    // (Each side of a closed mesh runs one way once: a point's neighbours are where its sides go.)
    std::vector<uint32_t> fill(start.begin(), start.end() - 1);
    for (size_t t = 0; t < outTris.size(); t += 3)
      for (int q = 0; q < 3; q++) nb[fill[outTris[t + q]]++] = outTris[t + (q + 1) % 3];
  }
  std::vector<V3> moved(np), normal(np);
  for (int pass = 0; pass < passes; pass++) {
    std::fill(normal.begin(), normal.end(), V3{0, 0, 0});
    for (size_t t = 0; t < outTris.size(); t += 3) {
      V3 w = cross(outPts[outTris[t + 1]] - outPts[outTris[t]], outPts[outTris[t + 2]] - outPts[outTris[t]]);
      for (int q = 0; q < 3; q++) normal[outTris[t + q]] += w;
    }
    for (size_t i = 0; i < np; i++) {
      V3 mid{0, 0, 0};
      for (uint32_t s = start[i]; s < start[i + 1]; s++) mid += outPts[nb[s]];
      V3 d = start[i + 1] > start[i] ? mid / (double)(start[i + 1] - start[i]) - outPts[i] : V3{0, 0, 0};
      V3 n = norm(normal[i]) > 0 ? unit(normal[i]) : V3{0, 0, 0};
      moved[i] = outPts[i] + (d - n * dot(d, n)) * 0.5;
    }
    outPts.swap(moved);
  }
}

// The nearest point of triangle abc to q (Ericson's regions: a corner, a side or the inside).
V3 nearestOnTriangle(V3 q, V3 a, V3 b, V3 c) {
  V3 ab = b - a, ac = c - a, aq = q - a;
  double d1 = dot(ab, aq), d2 = dot(ac, aq);
  if (d1 <= 0 && d2 <= 0) return a;
  V3 bq = q - b;
  double d3 = dot(ab, bq), d4 = dot(ac, bq);
  if (d3 >= 0 && d4 <= d3) return b;
  double vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
  V3 cq = q - c;
  double d5 = dot(ab, cq), d6 = dot(ac, cq);
  if (d6 >= 0 && d5 <= d6) return c;
  double vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
  double va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
  double den = 1 / (va + vb + vc);
  return a + ab * (vb * den) + ac * (vc * den);
}

}  // namespace

bool remesh(const std::vector<V3> &pts, const std::vector<uint32_t> &tris, double detail, std::vector<V3> &outPts, std::vector<uint32_t> &outTris,
            std::string &why, size_t most) {
  outPts.clear(), outTris.clear();
  if (!(detail > 0) || !std::isfinite(detail)) return why = "the detail must be a size", false;
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (uint32_t i : tris) lo = vmin(lo, pts[i]), hi = vmax(hi, pts[i]);
  if (tris.empty() || !std::isfinite(lo.x + lo.y + lo.z + hi.x + hi.y + hi.z)) return why = "nothing to remesh", false;
  // The grid: a step of the detail, over the mesh's box and a step and a half round it (so its edge is all outside).
  Grid g;
  g.h = detail;
  double cells = 1;
  for (int a = 0; a < 3; a++) {
    g.o[a] = lo[a] - 1.5 * detail;
    double n = std::ceil((hi[a] - lo[a]) / detail) + 4;
    cells *= n;
    if (n > 2e6) return why = "too fine for its size", false;
    g.n[a] = (int)n;
  }
  if (cells > 4e8) return why = "too fine for its size", false;
  Lines lines[3];
  lines[0] = cast(pts, tris, g, 0);
  std::vector<uint64_t> in;
  size_t inner = label(g, lines[0], in);
  if (!inner) return why = "nothing inside", false;
  auto inside = [&](int i, int j, int k) {
    size_t id = g.index(i, j, k);
    return (in[id >> 6] >> (id & 63) & 1) != 0;
  };
  // The cubes the surface passes through (some corners in, some out): about four triangles each.
  size_t mixed = 0;
  for (int k = 0; k + 1 < g.n[2]; k++)
    for (int j = 0; j + 1 < g.n[1]; j++)
      for (int i = 0; i + 1 < g.n[0]; i++) {
        int m = 0;
        for (int c = 0; c < 8; c++) m |= (int)inside(i + (c & 1), j + (c >> 1 & 1), k + (c >> 2 & 1)) << c;
        mixed += m != 0 && m != 255;
      }
  if (4 * mixed > most) return why = "too fine: about " + std::to_string(4 * mixed) + " triangles", false;
  lines[1] = cast(pts, tris, g, 1);
  lines[2] = cast(pts, tris, g, 2);
  // The surface where the grid's lines cross the mesh's.
  march(g, inside, [&](int i, int j, int k, int a, bool farIn) { return crossingOn(lines[a], g, i, j, k, farIn); }, 4 * mixed, outPts, outTris);
  relax(outPts, outTris, 2);
  return true;
}

namespace {

// Whether segment pq passes through triangle abc: each end strictly on its own side of the triangle's plane, and the
// line within the triangle (on a side or a corner too: there the triangle beside it is passed through as well).
bool through(V3 p, V3 q, V3 a, V3 b, V3 c) {
  int sp = orient3d(a, b, c, p), sq = orient3d(a, b, c, q);
  if (!sp || !sq || sp == sq) return false;
  int s1 = orient3d(p, q, a, b), s2 = orient3d(p, q, b, c), s3 = orient3d(p, q, c, a);
  return (s1 >= 0 && s2 >= 0 && s3 >= 0) || (s1 <= 0 && s2 <= 0 && s3 <= 0);
}

// Whether two triangles in one plane (that across axis `drop`) overlap more than along a side or at a corner: no side of
// either has all of the other on its outside (or on it).
bool overlapFlat(const V3 *t, const V3 *u, int drop) {
  int x = (drop + 1) % 3, y = (drop + 2) % 3;
  auto apart = [&](const V3 *a, const V3 *b) {
    int o = orient2d(a[0][x], a[0][y], a[1][x], a[1][y], a[2][x], a[2][y]);
    if (!o) return true;
    for (int e = 0; e < 3; e++) {
      const V3 &p = a[e], &q = a[(e + 1) % 3];
      bool out = true;
      for (int k = 0; k < 3 && out; k++) out = orient2d(p[x], p[y], q[x], q[y], b[k][x], b[k][y]) * o <= 0;
      if (out) return true;
    }
    return false;
  };
  return !apart(t, u) && !apart(u, t);
}

}  // namespace

bool selfCrossing(const std::vector<V3> &P, const std::vector<uint32_t> &T) {
  size_t nt = T.size() / 3;
  if (nt < 2) return false;
  // A box tree over the triangles (halved by their middles, down to four).
  struct Box {
    V3 lo, hi;
  };
  std::vector<Box> box(nt);
  std::vector<V3> mid(nt);
  for (size_t t = 0; t < nt; t++) {
    V3 a = P[T[3 * t]], b = P[T[3 * t + 1]], c = P[T[3 * t + 2]];
    box[t] = {vmin(vmin(a, b), c), vmax(vmax(a, b), c)};
    mid[t] = (a + b + c) / 3;
  }
  struct Node {
    Box b;
    int left = -1, right = -1, first = 0, count = 0;
  };
  std::vector<Node> nodes;
  std::vector<uint32_t> order(nt);
  for (size_t t = 0; t < nt; t++) order[t] = (uint32_t)t;
  std::function<int(int, int)> build = [&](int first, int count) {
    int id = (int)nodes.size();
    nodes.push_back({});
    Box b = box[order[first]];
    V3 mlo = mid[order[first]], mhi = mlo;
    for (int k = first; k < first + count; k++) {
      b.lo = vmin(b.lo, box[order[k]].lo), b.hi = vmax(b.hi, box[order[k]].hi);
      mlo = vmin(mlo, mid[order[k]]), mhi = vmax(mhi, mid[order[k]]);
    }
    nodes[id].b = b;
    if (count <= 4) {
      nodes[id].first = first, nodes[id].count = count;
      return id;
    }
    V3 span = mhi - mlo;
    int axis = span.x >= span.y && span.x >= span.z ? 0 : span.y >= span.z ? 1 : 2;
    int half = count / 2;
    std::nth_element(order.begin() + first, order.begin() + first + half, order.begin() + first + count, [&](uint32_t x, uint32_t y) {
      return mid[x][axis] < mid[y][axis] || (mid[x][axis] == mid[y][axis] && x < y);
    });
    int l = build(first, half), r = build(first + half, count - half);
    nodes[id].left = l, nodes[id].right = r;
    return id;
  };
  build(0, (int)nt);
  auto overlap = [](const Box &x, const Box &y) {
    return x.lo.x <= y.hi.x && y.lo.x <= x.hi.x && x.lo.y <= y.hi.y && y.lo.y <= x.hi.y && x.lo.z <= y.hi.z && y.lo.z <= x.hi.z;
  };
  std::vector<int> stack;
  for (size_t t = 0; t < nt; t++) {
    const uint32_t *x = &T[3 * t];
    V3 a = P[x[0]], b = P[x[1]], c = P[x[2]];
    stack.assign(1, 0);
    while (!stack.empty()) {
      const Node &nd = nodes[stack.back()];
      stack.pop_back();
      if (!overlap(nd.b, box[t])) continue;
      if (nd.left >= 0) {
        stack.push_back(nd.left), stack.push_back(nd.right);
        continue;
      }
      for (int k = nd.first; k < nd.first + nd.count; k++) {
        uint32_t u = order[k];
        if (u <= t || !overlap(box[u], box[t])) continue;
        const uint32_t *y = &T[3 * u];
        // (Triangles meeting at a corner or a side are neighbours, not a crossing.)
        bool shared = false;
        for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++) shared = shared || x[i] == y[j];
        if (shared) continue;
        V3 d = P[y[0]], e = P[y[1]], f = P[y[2]];
        if (through(d, e, a, b, c) || through(e, f, a, b, c) || through(f, d, a, b, c) || through(a, b, d, e, f) || through(b, c, d, e, f) ||
            through(c, a, d, e, f))
          return true;
        // In one plane: overlapping (as pieces laid over each other are).
        if (!orient3d(a, b, c, d) && !orient3d(a, b, c, e) && !orient3d(a, b, c, f)) {
          V3 nrm = cross(b - a, c - a);
          int drop = std::fabs(nrm.x) >= std::fabs(nrm.y) && std::fabs(nrm.x) >= std::fabs(nrm.z) ? 0 : std::fabs(nrm.y) >= std::fabs(nrm.z) ? 1 : 2;
          const V3 t3[3] = {a, b, c}, u3[3] = {d, e, f};
          if (overlapFlat(t3, u3, drop)) return true;
        }
      }
    }
  }
  return false;
}

bool hollowByGrid(const std::vector<V3> &pts, const std::vector<uint32_t> &tris, double t, std::vector<V3> &outPts,
                  std::vector<uint32_t> &outTris, std::string &why) {
  outPts.clear(), outTris.clear();
  if (!(t > 0) || !std::isfinite(t)) return why = "the walls must be a size", false;
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (uint32_t i : tris) lo = vmin(lo, pts[i]), hi = vmax(hi, pts[i]);
  if (tris.empty() || !std::isfinite(lo.x + lo.y + lo.z + hi.x + hi.y + hi.z)) return why = "nothing to hollow", false;
  // A grid half the wall apart (coarser where that would take more than 20 million points), over the box and a step
  // round it.
  V3 span = hi - lo;
  double h = std::max(t / 2, std::cbrt((span.x + t) * (span.y + t) * (span.z + t) / 2e7));
  Grid g;
  g.h = h;
  for (int a = 0; a < 3; a++) g.o[a] = lo[a] - h, g.n[a] = (int)std::ceil(span[a] / h) + 3;
  size_t N = (size_t)g.n[0] * g.n[1] * g.n[2];
  std::vector<uint64_t> in;
  if (!label(g, cast(pts, tris, g, 0), in)) return why = "nothing inside", false;
  auto inside = [&](int i, int j, int k) {
    size_t id = g.index(i, j, k);
    return (in[id >> 6] >> (id & 63) & 1) != 0;
  };
  // How far each point inside lies from the surface, wherever that's less than the wall and a step (further: the void).
  // (The void's surface is put a little further in than the wall, so that between the grid's points, where it's only
  // as true as a straight line between them, the wall is still never thinner than asked.)
  double wall = t + 0.15 * h, reach = wall + h;
  std::vector<float> dist(N, INFINITY);
  for (size_t q = 0; q + 2 < tris.size(); q += 3) {
    V3 a = pts[tris[q]], b = pts[tris[q + 1]], c = pts[tris[q + 2]];
    V3 bl = vmin(vmin(a, b), c), bh = vmax(vmax(a, b), c);
    int r0[3], r1[3];
    for (int x = 0; x < 3; x++) {
      r0[x] = std::max(0, (int)std::floor((bl[x] - reach - g.o[x]) / h));
      r1[x] = std::min(g.n[x] - 1, (int)std::ceil((bh[x] + reach - g.o[x]) / h));
    }
    for (int k = r0[2]; k <= r1[2]; k++)
      for (int j = r0[1]; j <= r1[1]; j++)
        for (int i = r0[0]; i <= r1[0]; i++) {
          if (!inside(i, j, k)) continue;
          V3 p{g.at(0, i), g.at(1, j), g.at(2, k)};
          double d = norm(p - nearestOnTriangle(p, a, b, c));
          float &at = dist[g.index(i, j, k)];
          if (d < at) at = (float)d;
        }
  }
  // The void: inside, and further than the wall from the surface; its surface where that distance is the wall's.
  std::vector<uint64_t> hollow(in.size(), 0);
  size_t voided = 0;
  for (size_t id = 0; id < N; id++)
    if ((in[id >> 6] >> (id & 63) & 1) && dist[id] > wall) hollow[id >> 6] |= 1ull << (id & 63), voided++;
  if (!voided) return why = "the walls fill it", false;
  auto empty = [&](int i, int j, int k) {
    size_t id = g.index(i, j, k);
    return (hollow[id >> 6] >> (id & 63) & 1) != 0;
  };
  // (Outside counts as on the surface; further than the wall and a step, as just that.)
  auto beyond = [&](int i, int j, int k) {
    size_t id = g.index(i, j, k);
    if (!(in[id >> 6] >> (id & 63) & 1)) return -wall;
    return std::min((double)dist[id], reach) - wall;
  };
  march(g, empty,
        [&](int i, int j, int k, int a, bool) {
          int up[3] = {i, j, k};
          up[a]++;
          double f0 = beyond(i, j, k), f1 = beyond(up[0], up[1], up[2]);
          double f = f0 != f1 ? f0 / (f0 - f1) : 0.5;
          return std::min(0.98, std::max(0.02, f));
        },
        4 * voided, outPts, outTris);
  relax(outPts, outTris, 2);
  // Facing into the void (away from the walls round it).
  for (size_t q = 0; q + 2 < outTris.size(); q += 3) std::swap(outTris[q + 1], outTris[q + 2]);
  return true;
}


// Sculpting.

Sculptor::Sculptor(std::vector<V3> pts, std::vector<uint32_t> tris) : p(std::move(pts)), tri(std::move(tris)) {
  size_t np = p.size(), nt = tri.size() / 3;
  tri.resize(nt * 3);
  // Per point: its triangles, and its neighbours, each in order.
  vtStart.assign(np + 1, 0);
  for (uint32_t v : tri) vtStart[v + 1]++;
  for (size_t i = 0; i < np; i++) vtStart[i + 1] += vtStart[i];
  vtList.resize(tri.size());
  {
    std::vector<uint32_t> fill(vtStart.begin(), vtStart.end() - 1);
    for (size_t t = 0; t < nt; t++)
      for (int q = 0; q < 3; q++) vtList[fill[tri[3 * t + q]]++] = (uint32_t)t;
  }
  nbStart.assign(np + 1, 0);
  std::vector<uint32_t> near;
  for (size_t i = 0; i < np; i++) {
    near.clear();
    for (uint32_t s = vtStart[i]; s < vtStart[i + 1]; s++)
      for (int q = 0; q < 3; q++)
        if (tri[3 * vtList[s] + q] != i) near.push_back(tri[3 * vtList[s] + q]);
    std::sort(near.begin(), near.end());
    near.erase(std::unique(near.begin(), near.end()), near.end());
    nbList.insert(nbList.end(), near.begin(), near.end());
    nbStart[i + 1] = (uint32_t)nbList.size();
  }
  n.resize(np);
  for (size_t i = 0; i < np; i++) n[i] = normalAt((uint32_t)i);
  if (nt) {
    order.resize(nt);
    leafOf.assign(nt, 0);
    std::vector<V3> mid(nt);
    for (size_t t = 0; t < nt; t++) order[t] = (uint32_t)t, mid[t] = (p[tri[3 * t]] + p[tri[3 * t + 1]] + p[tri[3 * t + 2]]) / 3;
    nodes.reserve(nt / 2 + 1), parentOf.reserve(nt / 2 + 1);
    build(0, (int)nt, mid, -1);
  }
  dirtyNode.assign(nodes.size(), 0);
  seen.assign(np, 0);
  touchedAt.assign(np, 0);
  changedFlag.assign(np, 0);
}

void Sculptor::boxOf(uint32_t t, V3 &lo, V3 &hi) const {
  V3 a = p[tri[3 * t]], b = p[tri[3 * t + 1]], c = p[tri[3 * t + 2]];
  lo = vmin(vmin(a, b), c), hi = vmax(vmax(a, b), c);
}

V3 Sculptor::normalAt(uint32_t i) const {
  V3 sum{0, 0, 0};
  for (uint32_t s = vtStart[i]; s < vtStart[i + 1]; s++) {
    const uint32_t *t = &tri[3 * vtList[s]];
    sum += cross(p[t[1]] - p[t[0]], p[t[2]] - p[t[0]]);
  }
  return norm2(sum) > 0 ? unit(sum) : V3{0, 0, 0};
}

// Halved by the triangles' middles along the box's longest way (the same halves however they're sorted), down to four.
int Sculptor::build(int first, int count, const std::vector<V3> &mid, int parent) {
  int id = (int)nodes.size();
  nodes.push_back({});
  parentOf.push_back(parent);
  const double big = std::numeric_limits<double>::infinity();
  V3 lo{big, big, big}, hi{-big, -big, -big}, mlo = lo, mhi = hi;
  for (int k = first; k < first + count; k++) {
    V3 a, b;
    boxOf(order[k], a, b);
    lo = vmin(lo, a), hi = vmax(hi, b);
    mlo = vmin(mlo, mid[order[k]]), mhi = vmax(mhi, mid[order[k]]);
  }
  nodes[id].lo = lo, nodes[id].hi = hi;
  if (count <= 4) {
    nodes[id].first = first, nodes[id].count = count;
    for (int k = first; k < first + count; k++) leafOf[order[k]] = (uint32_t)id;
    return id;
  }
  V3 span = mhi - mlo;
  int axis = span.x >= span.y && span.x >= span.z ? 0 : span.y >= span.z ? 1 : 2;
  int half = count / 2;
  std::nth_element(order.begin() + first, order.begin() + first + half, order.begin() + first + count, [&](uint32_t a, uint32_t b) {
    double x = mid[a][axis], y = mid[b][axis];
    return x < y || (x == y && a < b);
  });
  int left = build(first, half, mid, id);
  int right = build(first + half, count - half, mid, id);
  nodes[id].left = left, nodes[id].right = right;
  return id;
}

bool Sculptor::ray(V3 o, V3 d, V3 &at, V3 &normal) const {
  if (nodes.empty() || !(norm2(d) > 0)) return false;
  double best = std::numeric_limits<double>::infinity(), bu = 0, bv = 0;
  int64_t hit = -1;
  int stack[128], top = 0;
  stack[top++] = 0;
  while (top) {
    const TreeNode &nd = nodes[stack[--top]];
    double t0 = 0, t1 = best;
    bool miss = false;
    for (int a = 0; a < 3 && !miss; a++) {
      if (d[a] == 0) {
        miss = o[a] < nd.lo[a] || o[a] > nd.hi[a];
        continue;
      }
      double ta = (nd.lo[a] - o[a]) / d[a], tb = (nd.hi[a] - o[a]) / d[a];
      if (ta > tb) std::swap(ta, tb);
      t0 = std::max(t0, ta), t1 = std::min(t1, tb);
      miss = t0 > t1;
    }
    if (miss) continue;
    if (nd.left >= 0) {
      stack[top++] = nd.right, stack[top++] = nd.left;
      continue;
    }
    for (int k = nd.first; k < nd.first + nd.count; k++) {
      uint32_t t = order[k];
      V3 A = p[tri[3 * t]], e1 = p[tri[3 * t + 1]] - A, e2 = p[tri[3 * t + 2]] - A;
      V3 pv = cross(d, e2);
      double det = dot(e1, pv);
      if (det == 0) continue;
      V3 tv = o - A, qv = cross(tv, e1);
      double u = dot(tv, pv) / det, v = dot(d, qv) / det, s = dot(e2, qv) / det;
      if (!(u >= 0 && v >= 0 && u + v <= 1 && s >= 0)) continue;
      if (s < best || (s == best && (int64_t)t < hit)) best = s, hit = t, bu = u, bv = v;
    }
  }
  if (hit < 0) return false;
  const uint32_t *t = &tri[3 * hit];
  at = o + d * best;
  V3 m = n[t[0]] * (1 - bu - bv) + n[t[1]] * bu + n[t[2]] * bv;
  normal = norm2(m) > 0 ? unit(m) : unit(cross(p[t[1]] - p[t[0]], p[t[2]] - p[t[0]]));
  return true;
}

void Sculptor::within(V3 c, double r, std::vector<uint32_t> &out, std::vector<double> &weight) const {
  out.clear(), weight.clear();
  if (nodes.empty() || !(r > 0)) return;
  if (++seenStamp == 0) std::fill(seen.begin(), seen.end(), 0), seenStamp = 1;
  double r2 = r * r;
  int stack[128], top = 0;
  stack[top++] = 0;
  while (top) {
    const TreeNode &nd = nodes[stack[--top]];
    double d2 = 0;
    for (int a = 0; a < 3; a++) {
      double e = std::max({nd.lo[a] - c[a], 0.0, c[a] - nd.hi[a]});
      d2 += e * e;
    }
    if (d2 >= r2) continue;
    if (nd.left >= 0) {
      stack[top++] = nd.right, stack[top++] = nd.left;
      continue;
    }
    for (int k = nd.first; k < nd.first + nd.count; k++)
      for (int q = 0; q < 3; q++) {
        uint32_t v = tri[3 * order[k] + q];
        if (seen[v] == seenStamp) continue;
        seen[v] = seenStamp;
        if (norm2(p[v] - c) < r2) out.push_back(v);
      }
  }
  std::sort(out.begin(), out.end());
  weight.resize(out.size());
  for (size_t k = 0; k < out.size(); k++) {
    double f = 1 - norm2(p[out[k]] - c) / r2;
    weight[k] = f * f;
  }
}

void Sculptor::begin(int b, V3 at, double r, double s, bool mir, bool inv) {
  if (stroking) end();
  stroking = true, dabbed = false;
  brush = b >= Grab && b <= Crease ? b : Draw;
  radius = r > 1e-9 ? r : 1e-9, strength = std::min(1.0, std::max(0.0, s));
  mirror = mir, invert = inv;
  start = last = at;
  if (++strokeId == 0) std::fill(touchedAt.begin(), touchedAt.end(), 0), strokeId = 1;
  step = Step();
  grabbed.clear(), grabWeight[0].clear(), grabWeight[1].clear(), grabFrom.clear();
  if (brush != Grab) return;
  within(at, radius, idx[0], wt[0]);
  if (mirror) within(V3{-at.x, at.y, at.z}, radius, idx[1], wt[1]);
  else idx[1].clear(), wt[1].clear();
  size_t a = 0, c = 0;
  while (a < idx[0].size() || c < idx[1].size()) {
    bool fromA = c == idx[1].size() || (a < idx[0].size() && idx[0][a] <= idx[1][c]);
    bool fromC = a == idx[0].size() || (c < idx[1].size() && idx[1][c] <= idx[0][a]);
    grabbed.push_back(fromA ? idx[0][a] : idx[1][c]);
    grabWeight[0].push_back(fromA ? wt[0][a++] : 0.0);
    grabWeight[1].push_back(fromC ? wt[1][c++] : 0.0);
    grabFrom.push_back(p[grabbed.back()]);
  }
}

void Sculptor::offsets(V3 c, double pressure, double radius, std::vector<uint32_t> &which, std::vector<double> &w, std::vector<V3> &by) const {
  within(c, radius, which, w);
  by.assign(which.size(), V3{0, 0, 0});
  if (which.empty()) return;
  double s = strength * pressure * (invert ? -1 : 1);
  // The surface's average way out under the brush, and its middle.
  V3 out{0, 0, 0}, mid{0, 0, 0};
  double total = 0;
  for (size_t k = 0; k < which.size(); k++) out += n[which[k]] * w[k], mid += p[which[k]] * w[k], total += w[k];
  out = norm2(out) > 0 ? unit(out) : V3{0, 0, 0};
  if (total > 0) mid = mid / total;
  for (size_t k = 0; k < which.size(); k++) {
    uint32_t i = which[k];
    V3 q = p[i];
    V3 in = c - q;
    in = in - out * dot(in, out);  // toward the brush's middle, along the surface
    switch (brush) {
      case Draw: by[k] = out * (s * radius * 0.1 * w[k]); break;
      case Inflate: by[k] = n[i] * (s * radius * 0.1 * w[k]); break;
      case Smooth: {
        uint32_t a = nbStart[i], e = nbStart[i + 1];
        if (a == e) break;
        V3 m{0, 0, 0};
        for (uint32_t j = a; j < e; j++) m += p[nbList[j]];
        by[k] = (m / (double)(e - a) - q) * std::min(1.0, std::fabs(s) * w[k]);
        break;
      }
      case Flatten: by[k] = out * (-dot(q - mid, out) * std::max(-1.0, std::min(1.0, s * w[k]))); break;
      case Pinch: by[k] = in * (s * 0.3 * w[k]); break;
      case Crease: by[k] = out * (-s * radius * 0.08 * w[k]) + in * (std::fabs(s) * 0.3 * w[k]); break;
    }
  }
}

// One dab: what it does at c and (with the mirror) across x = 0, both from the points as they are, together.
void Sculptor::dabAt(V3 c, double pressure, double r) {
  offsets(c, pressure, r, idx[0], wt[0], off[0]);
  if (mirror) offsets(V3{-c.x, c.y, c.z}, pressure, r, idx[1], wt[1], off[1]);
  else idx[1].clear(), off[1].clear();
  sum.clear(), offSum.clear();
  size_t a = 0, b = 0;
  while (a < idx[0].size() || b < idx[1].size()) {
    if (b == idx[1].size() || (a < idx[0].size() && idx[0][a] < idx[1][b])) sum.push_back(idx[0][a]), offSum.push_back(off[0][a++]);
    else if (a == idx[0].size() || idx[1][b] < idx[0][a]) sum.push_back(idx[1][b]), offSum.push_back(off[1][b++]);
    else sum.push_back(idx[0][a]), offSum.push_back(off[0][a++] + off[1][b++]);
  }
  for (size_t k = 0; k < sum.size(); k++) record(sum[k]), p[sum[k]] += offSum[k];
  moved(sum);
}

void Sculptor::dab(V3 at, double pressure, double size) {
  if (!stroking) return;
  pressure = std::min(1.0, std::max(0.0, pressure));
  // (A pen pressed lightly may make the brush smaller, down to a twentieth.)
  double r = radius * std::min(1.0, std::max(0.05, size));
  if (brush == Grab) {
    V3 d = at - start, md{-d.x, d.y, d.z};
    for (size_t k = 0; k < grabbed.size(); k++) record(grabbed[k]), p[grabbed[k]] = grabFrom[k] + d * grabWeight[0][k] + md * grabWeight[1][k];
    moved(grabbed);
    last = at;
    return;
  }
  if (!dabbed) {
    dabAt(at, pressure, r);
    last = at, dabbed = true;
    return;
  }
  // Every fifth of the radius along the way.
  double gap = norm(at - last), spacing = 0.2 * r;
  if (!(gap >= spacing)) return;
  int steps = (int)std::min(1000.0, std::floor(gap / spacing));
  V3 way = (at - last) / gap;
  for (int s = 1; s <= steps; s++) dabAt(last + way * (s * spacing), pressure, r);
  last = last + way * (steps * spacing);
}

void Sculptor::record(uint32_t i) {
  if (touchedAt[i] == strokeId) return;
  touchedAt[i] = strokeId;
  step.idx.push_back(i), step.before.push_back(p[i]);
}

void Sculptor::end() {
  if (!stroking) return;
  stroking = false;
  std::vector<uint32_t> by(step.idx.size());
  for (size_t k = 0; k < by.size(); k++) by[k] = (uint32_t)k;
  std::sort(by.begin(), by.end(), [&](uint32_t a, uint32_t b) { return step.idx[a] < step.idx[b]; });
  Step s;
  for (uint32_t k : by) {
    uint32_t i = step.idx[k];
    if (p[i] == step.before[k]) continue;
    s.idx.push_back(i), s.before.push_back(step.before[k]), s.after.push_back(p[i]);
  }
  step = Step();
  if (!s.idx.empty()) keep(std::move(s));
}

// Kept to undo: the last 100 strokes, or fewer when they've moved more than 4 million points between them.
void Sculptor::keep(Step s) {
  for (const auto &u : undone) kept -= u.idx.size();
  undone.clear();
  kept += s.idx.size();
  done.push_back(std::move(s));
  size_t drop = 0;
  while (done.size() - drop > 1 && (done.size() - drop > 100 || kept > 4000000)) kept -= done[drop++].idx.size();
  done.erase(done.begin(), done.begin() + drop);
}

bool Sculptor::undo() {
  if (stroking) end();
  if (done.empty()) return false;
  undone.push_back(std::move(done.back()));
  done.pop_back();
  restore(undone.back(), true);
  return true;
}

bool Sculptor::redo() {
  if (stroking) end();
  if (undone.empty()) return false;
  done.push_back(std::move(undone.back()));
  undone.pop_back();
  restore(done.back(), false);
  return true;
}

void Sculptor::restore(const Step &s, bool back) {
  for (size_t k = 0; k < s.idx.size(); k++) p[s.idx[k]] = back ? s.before[k] : s.after[k];
  moved(s.idx);
}

void Sculptor::moved(const std::vector<uint32_t> &which) {
  if (++seenStamp == 0) std::fill(seen.begin(), seen.end(), 0), seenStamp = 1;
  around.clear();
  for (uint32_t i : which) {
    if (seen[i] != seenStamp) seen[i] = seenStamp, around.push_back(i);
    for (uint32_t s = vtStart[i]; s < vtStart[i + 1]; s++) {
      uint32_t t = vtList[s];
      for (int x = (int)leafOf[t]; x >= 0 && !dirtyNode[x]; x = parentOf[x]) dirtyNode[x] = 1, dirtyList.push_back((uint32_t)x);
      for (int q = 0; q < 3; q++) {
        uint32_t v = tri[3 * t + q];
        if (seen[v] != seenStamp) seen[v] = seenStamp, around.push_back(v);
      }
    }
  }
  // Boxes made again from the leaves up (a node's children come after it).
  std::sort(dirtyList.begin(), dirtyList.end(), std::greater<uint32_t>());
  for (uint32_t x : dirtyList) {
    TreeNode &nd = nodes[x];
    dirtyNode[x] = 0;
    if (nd.left >= 0) {
      nd.lo = vmin(nodes[nd.left].lo, nodes[nd.right].lo), nd.hi = vmax(nodes[nd.left].hi, nodes[nd.right].hi);
      continue;
    }
    boxOf(order[nd.first], nd.lo, nd.hi);
    for (int k = nd.first + 1; k < nd.first + nd.count; k++) {
      V3 a, b;
      boxOf(order[k], a, b);
      nd.lo = vmin(nd.lo, a), nd.hi = vmax(nd.hi, b);
    }
  }
  dirtyList.clear();
  for (uint32_t v : around) {
    n[v] = normalAt(v);
    if (!changedFlag[v]) changedFlag[v] = 1, changed.push_back(v);
  }
}

std::vector<uint32_t> Sculptor::takeChanged() {
  std::vector<uint32_t> out;
  out.swap(changed);
  std::sort(out.begin(), out.end());
  for (uint32_t v : out) changedFlag[v] = 0;
  return out;
}

}  // namespace bce
