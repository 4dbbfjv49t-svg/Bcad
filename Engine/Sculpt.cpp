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

namespace {

const uint32_t none = 0xffffffffu;
inline uint32_t nextC(uint32_t c) { return c % 3 == 2 ? c - 2 : c + 1; }
inline uint32_t prevC(uint32_t c) { return c % 3 == 0 ? c + 2 : c - 1; }

// The side the brush makes its triangles about, for each mm of detail: its sides come out between 0.6 and 4/3 of it,
// about 0.8 of it on average, so the same detail under the brush is as fine as a remesh.
const double sideOfDetail = 0.88;

// The cotangent of the angle at o between u and v (huge for one of 0 or half a turn): two facing a side add up to below 0
// when those angles pass half a turn, the side then better turned the other way across its two triangles.
double cotangent(V3 o, V3 u, V3 v) {
  V3 s = u - o, t = v - o;
  double c = norm(cross(s, t)), d = dot(s, t);
  return c > 0 ? d / c : d > 0 ? 1e30 : -1e30;
}

// How far c lies from segment ab.
double toSegment(V3 c, V3 a, V3 b) {
  V3 ab = b - a;
  double l2 = norm2(ab), t = l2 > 0 ? std::min(1.0, std::max(0.0, dot(c - a, ab) / l2)) : 0;
  return norm(a + ab * t - c);
}

}  // namespace

Sculptor::Sculptor(std::vector<V3> pts, std::vector<uint32_t> tris) : p(std::move(pts)), tri(std::move(tris)) {
  size_t nt = tri.size() / 3;
  tri.resize(nt * 3);
  if (nt < 4 || !(good = link())) {
    good = false;
    return;
  }
  size_t np = p.size();
  ptAlive.assign(np, 0);
  for (size_t v = 0; v < np; v++) ptAlive[v] = corner[v] != none;
  triAlive.assign(nt, 1);
  live = nt;
  n.assign(np, V3{0, 0, 0});
  for (size_t v = 0; v < np; v++)
    if (ptAlive[v]) n[v] = normalAt((uint32_t)v);
  seen.assign(np, 0), region.assign(np, 0), pointTouched.assign(np, 0), changedFlag.assign(np, 0);
  triTouched.assign(nt, 0), changedTriFlag.assign(nt, 0), leafOf.assign(nt, -1);
  rebuildTree();
}

// The corner table from the triangles: each side found running back in another triangle (exactly one), and each point's
// corners a single fan round it (a point two fans meet at made one point per fan). False when the mesh isn't closed.
bool Sculptor::link() {
  size_t nc = tri.size();
  for (uint32_t v : tri)
    if (v >= p.size()) return false;
  std::vector<std::pair<uint64_t, uint32_t>> sides(nc);
  for (uint32_t c = 0; c < nc; c++) {
    uint32_t a = tri[nextC(c)], b = tri[prevC(c)];
    if (a == b || tri[c] == a || tri[c] == b) return false;
    sides[c] = {(uint64_t)a << 32 | b, c};
  }
  std::sort(sides.begin(), sides.end());
  opp.assign(nc, none);
  for (size_t i = 0; i < nc; i++) {
    if (i + 1 < nc && sides[i].first == sides[i + 1].first) return false;
    uint64_t back = sides[i].first << 32 | sides[i].first >> 32;
    auto it = std::lower_bound(sides.begin(), sides.end(), std::make_pair(back, (uint32_t)0));
    if (it == sides.end() || it->first != back) return false;
    opp[sides[i].second] = it->second;
  }
  corner.assign(p.size(), none);
  std::vector<uint8_t> walked(nc, 0);
  for (uint32_t c = 0; c < nc; c++) {
    if (walked[c]) continue;
    uint32_t v = tri[c], w = v;
    if (corner[v] != none) {
      w = (uint32_t)p.size();
      p.push_back(p[v]);
      corner.push_back(none);
    }
    corner[w] = c;
    uint32_t x = c;
    size_t steps = 0;
    do {
      walked[x] = 1, tri[x] = w, x = swing(x);
      if (++steps > nc) return false;
    } while (x != c);
  }
  return true;
}

// The next corner round the same point: across the side from it to the corner's next point.
uint32_t Sculptor::swing(uint32_t c) const { return prevC(opp[prevC(c)]); }

// The corner facing side a→b (in the triangle running a→b), or none.
uint32_t Sculptor::sideOf(uint32_t a, uint32_t b) const {
  uint32_t c = corner[a], x = c;
  do {
    if (tri[nextC(x)] == b) return prevC(x);
    x = swing(x);
  } while (x != c);
  return none;
}

// A point's neighbours, in order.
void Sculptor::neighbours(uint32_t v, std::vector<uint32_t> &out) const {
  out.clear();
  uint32_t c = corner[v], x = c;
  do {
    out.push_back(tri[nextC(x)]);
    x = swing(x);
  } while (x != c);
  std::sort(out.begin(), out.end());
}

void Sculptor::boxOf(uint32_t t, V3 &lo, V3 &hi) const {
  V3 a = p[tri[3 * t]], b = p[tri[3 * t + 1]], c = p[tri[3 * t + 2]];
  lo = vmin(vmin(a, b), c), hi = vmax(vmax(a, b), c);
}

// The area-weighted normal of a point's triangles (summed in their slots' order); with `cache`, each triangle's own
// normal taken from faceN when made since faceStamp was last moved on.
V3 Sculptor::normalAt(uint32_t v, bool cache) const {
  fan.clear();
  uint32_t c = corner[v], x = c;
  do {
    fan.push_back(x / 3);
    x = swing(x);
  } while (x != c);
  std::sort(fan.begin(), fan.end());
  V3 sum{0, 0, 0};
  for (uint32_t t : fan) {
    if (cache && faceAt[t] == faceStamp) {
      sum += faceN[t];
      continue;
    }
    V3 f = cross(p[tri[3 * t + 1]] - p[tri[3 * t]], p[tri[3 * t + 2]] - p[tri[3 * t]]);
    if (cache) faceN[t] = f, faceAt[t] = faceStamp;
    sum += f;
  }
  return norm2(sum) > 0 ? unit(sum) : V3{0, 0, 0};
}

void Sculptor::rebuildTree() {
  nodes.clear(), parentOf.clear();
  std::vector<uint32_t> order;
  std::vector<V3> mid(triAlive.size());
  for (uint32_t t = 0; t < triAlive.size(); t++) {
    leafOf[t] = -1;
    if (!triAlive[t]) continue;
    order.push_back(t);
    mid[t] = (p[tri[3 * t]] + p[tri[3 * t + 1]] + p[tri[3 * t + 2]]) / 3;
  }
  nodes.reserve(order.size() / 2 + 1), parentOf.reserve(order.size() / 2 + 1);
  if (!order.empty()) build(order, 0, (int)order.size(), mid, -1);
  dirtyNode.assign(nodes.size(), 0);
  grownLeaves = 0, liveAtBuild = live;
}

// Halved by the triangles' middles along the box's longest way (the same halves however they're sorted), down to four.
int Sculptor::build(std::vector<uint32_t> &order, int first, int count, const std::vector<V3> &mid, int parent) {
  int id = (int)nodes.size();
  nodes.push_back({});
  parentOf.push_back(parent);
  const double big = std::numeric_limits<double>::infinity();
  if (count <= 4) {
    V3 lo{big, big, big}, hi{-big, -big, -big};
    for (int k = first; k < first + count; k++) {
      V3 a, b;
      boxOf(order[k], a, b);
      lo = vmin(lo, a), hi = vmax(hi, b);
    }
    nodes[id].lo = lo, nodes[id].hi = hi;
    nodes[id].items.assign(order.begin() + first, order.begin() + first + count);
    for (int k = first; k < first + count; k++) leafOf[order[k]] = id;
    return id;
  }
  V3 mlo{big, big, big}, mhi{-big, -big, -big};
  for (int k = first; k < first + count; k++) mlo = vmin(mlo, mid[order[k]]), mhi = vmax(mhi, mid[order[k]]);
  V3 span = mhi - mlo;
  int axis = span.x >= span.y && span.x >= span.z ? 0 : span.y >= span.z ? 1 : 2;
  int half = count / 2;
  std::nth_element(order.begin() + first, order.begin() + first + half, order.begin() + first + count, [&](uint32_t a, uint32_t b) {
    double x = mid[a][axis], y = mid[b][axis];
    return x < y || (x == y && a < b);
  });
  int left = build(order, first, half, mid, id);
  int right = build(order, first + half, count - half, mid, id);
  nodes[id].left = left, nodes[id].right = right;
  nodes[id].lo = vmin(nodes[left].lo, nodes[right].lo), nodes[id].hi = vmax(nodes[left].hi, nodes[right].hi);
  return id;
}

void Sculptor::dirty(int x) {
  for (; x >= 0 && !dirtyNode[x]; x = parentOf[x]) dirtyNode[x] = 1, dirtyList.push_back((uint32_t)x);
}

// A new triangle joins a leaf (the one of the triangle it came from); a leaf grown past 32 is halved into two new ones.
void Sculptor::addToLeaf(uint32_t t, int leaf) {
  leafOf[t] = leaf;
  nodes[leaf].items.push_back(t);
  dirty(leaf);
  if (nodes[leaf].items.size() <= 32) return;
  std::vector<uint32_t> items;
  items.swap(nodes[leaf].items);
  std::vector<V3> mid(items.size());
  V3 mlo{INFINITY, INFINITY, INFINITY}, mhi{-INFINITY, -INFINITY, -INFINITY};
  for (size_t k = 0; k < items.size(); k++) {
    uint32_t u = items[k];
    mid[k] = (p[tri[3 * u]] + p[tri[3 * u + 1]] + p[tri[3 * u + 2]]) / 3;
    mlo = vmin(mlo, mid[k]), mhi = vmax(mhi, mid[k]);
  }
  V3 span = mhi - mlo;
  int axis = span.x >= span.y && span.x >= span.z ? 0 : span.y >= span.z ? 1 : 2;
  std::vector<uint32_t> by(items.size());
  for (size_t k = 0; k < by.size(); k++) by[k] = (uint32_t)k;
  std::sort(by.begin(), by.end(), [&](uint32_t a, uint32_t b) {
    double x = mid[a][axis], y = mid[b][axis];
    return x < y || (x == y && items[a] < items[b]);
  });
  int halves[2];
  for (int h = 0; h < 2; h++) {
    halves[h] = (int)nodes.size();
    nodes.push_back({});
    parentOf.push_back(leaf);
    dirtyNode.push_back(0);
  }
  size_t half = by.size() / 2;
  for (size_t k = 0; k < by.size(); k++) {
    int h = halves[k < half ? 0 : 1];
    nodes[h].items.push_back(items[by[k]]);
    leafOf[items[by[k]]] = h;
  }
  nodes[leaf].left = halves[0], nodes[leaf].right = halves[1];
  dirtyNode[leaf] = 0;  // (made dirty again below, with its new halves)
  for (int h : halves) dirty(h);
  grownLeaves++;
}

void Sculptor::removeFromLeaf(uint32_t t) {
  int leaf = leafOf[t];
  if (leaf < 0) return;
  auto &items = nodes[leaf].items;
  for (size_t k = 0; k < items.size(); k++)
    if (items[k] == t) {
      items.erase(items.begin() + (long)k);
      break;
    }
  leafOf[t] = -1;
  dirty(leaf);
}

bool Sculptor::ray(V3 o, V3 d, V3 &at, V3 &normal) const {
  if (nodes.empty() || !(norm2(d) > 0)) return false;
  double best = std::numeric_limits<double>::infinity(), bu = 0, bv = 0;
  int64_t hit = -1;
  std::vector<int> stack{0};
  while (!stack.empty()) {
    const TreeNode &nd = nodes[stack.back()];
    stack.pop_back();
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
      miss = !(t0 <= t1);
    }
    if (miss) continue;
    if (nd.left >= 0) {
      stack.push_back(nd.right), stack.push_back(nd.left);
      continue;
    }
    for (uint32_t t : nd.items) {
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
  std::vector<int> stack{0};
  while (!stack.empty()) {
    const TreeNode &nd = nodes[stack.back()];
    stack.pop_back();
    double d2 = 0;
    for (int a = 0; a < 3; a++) {
      double e = std::max({nd.lo[a] - c[a], 0.0, c[a] - nd.hi[a]});
      d2 += e * e;
    }
    if (!(d2 < r2)) continue;
    if (nd.left >= 0) {
      stack.push_back(nd.right), stack.push_back(nd.left);
      continue;
    }
    for (uint32_t t : nd.items)
      for (int q = 0; q < 3; q++) {
        uint32_t v = tri[3 * t + q];
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

void Sculptor::touching(V3 c, double r, std::vector<uint32_t> &out) const {
  out.clear();
  if (nodes.empty() || !(r > 0)) return;
  if (++seenStamp == 0) std::fill(seen.begin(), seen.end(), 0), seenStamp = 1;
  double r2 = r * r;
  std::vector<int> stack{0};
  while (!stack.empty()) {
    const TreeNode &nd = nodes[stack.back()];
    stack.pop_back();
    double d2 = 0;
    for (int a = 0; a < 3; a++) {
      double e = std::max({nd.lo[a] - c[a], 0.0, c[a] - nd.hi[a]});
      d2 += e * e;
    }
    if (!(d2 < r2)) continue;
    if (nd.left >= 0) {
      stack.push_back(nd.right), stack.push_back(nd.left);
      continue;
    }
    for (uint32_t t : nd.items) {
      V3 a = p[tri[3 * t]], b = p[tri[3 * t + 1]], d = p[tri[3 * t + 2]];
      if (!(norm2(nearestOnTriangle(c, a, b, d) - c) < r2)) continue;
      for (int q = 0; q < 3; q++) {
        uint32_t v = tri[3 * t + q];
        if (seen[v] != seenStamp) seen[v] = seenStamp, out.push_back(v);
      }
    }
  }
  std::sort(out.begin(), out.end());
}

void Sculptor::begin(int b, V3 at, double r, double s, bool mir, bool inv) {
  if (stroking) end();
  stroking = true, dabbed = false;
  brush = b >= Grab && b <= Detail ? b : Draw;
  radius = r > 1e-9 ? r : 1e-9, strength = std::min(1.0, std::max(0.0, s));
  mirror = mir, invert = inv;
  strokeDetail = detail;
  start = last = at;
  if (++strokeId == 0) {
    std::fill(pointTouched.begin(), pointTouched.end(), 0), std::fill(triTouched.begin(), triTouched.end(), 0);
    strokeId = 1;
  }
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
  if (which.empty() || brush == Detail) return;
  double s = strength * pressure * (invert ? -1 : 1);
  // The surface's average way out under the brush, and its middle.
  V3 out{0, 0, 0}, mid{0, 0, 0};
  double total = 0;
  for (size_t k = 0; k < which.size(); k++) out += n[which[k]] * w[k], mid += p[which[k]] * w[k], total += w[k];
  out = norm2(out) > 0 ? unit(out) : V3{0, 0, 0};
  if (total > 0) mid = mid / total;
  std::vector<uint32_t> &nb = const_cast<std::vector<uint32_t> &>(fan);
  for (size_t k = 0; k < which.size(); k++) {
    uint32_t i = which[k];
    V3 q = p[i];
    V3 in = c - q;
    in = in - out * dot(in, out);  // toward the brush's middle, along the surface
    switch (brush) {
      case Draw: by[k] = out * (s * radius * 0.1 * w[k]); break;
      case Inflate: by[k] = n[i] * (s * radius * 0.1 * w[k]); break;
      case Smooth: {
        neighbours(i, nb);
        if (nb.empty()) break;
        V3 m{0, 0, 0};
        for (uint32_t j : nb) m += p[j];
        by[k] = (m / (double)nb.size() - q) * std::min(1.0, std::fabs(s) * w[k]);
        break;
      }
      case Flatten: by[k] = out * (-dot(q - mid, out) * std::max(-1.0, std::min(1.0, s * w[k]))); break;
      case Pinch: by[k] = in * (s * 0.3 * w[k]); break;
      case Crease: by[k] = out * (-s * radius * 0.08 * w[k]) + in * (std::fabs(s) * 0.3 * w[k]); break;
    }
  }
}

// One dab: the triangles under it made the detail size first (with the detail on), then what it does at c and (with the
// mirror) across x = 0, both from the points as they are, together.
void Sculptor::dabAt(V3 c, double pressure, double r) {
  if (strokeDetail > 0) {
    V3 mc{-c.x, c.y, c.z};
    touching(c, r, idx[0]);
    if (mirror) touching(mc, r, idx[1]);
    else idx[1].clear();
    sum.clear();
    std::set_union(idx[0].begin(), idx[0].end(), idx[1].begin(), idx[1].end(), std::back_inserter(sum));
    std::vector<uint32_t> seeds(sum);
    // (Their normals made again with the points the dab moves, but for the Detail brush, which moves none.)
    retopo(seeds, [&](uint32_t a, uint32_t b) { return toSegment(c, p[a], p[b]) < r || (mirror && toSegment(mc, p[a], p[b]) < r); }, false, 8000,
           brush != Detail);
  } else {
    fresh.clear();
  }
  if (brush == Detail) return;
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
  for (size_t k = 0; k < sum.size(); k++) recordPoint(sum[k]), p[sum[k]] += offSum[k];
  if (!fresh.empty()) {
    std::vector<uint32_t> both;
    std::set_union(sum.begin(), sum.end(), fresh.begin(), fresh.end(), std::back_inserter(both));
    sum.swap(both);
  }
  moved(sum);
}

void Sculptor::dab(V3 at, double pressure, double size) {
  if (!stroking) return;
  pressure = std::min(1.0, std::max(0.0, pressure));
  // (A pen pressed lightly may make the brush smaller, down to a twentieth.)
  double r = radius * std::min(1.0, std::max(0.05, size));
  if (brush == Grab) {
    V3 d = at - start, md{-d.x, d.y, d.z};
    for (size_t k = 0; k < grabbed.size(); k++) recordPoint(grabbed[k]), p[grabbed[k]] = grabFrom[k] + d * grabWeight[0][k] + md * grabWeight[1][k];
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

// The triangles under a dab (or a grabbed area) made the detail size, in rounds. Each round: the sides for which
// `inside` holds from the points to look at longer than 4/3 of the side length the detail asks for halved, longest
// first; those shorter than 0.6 of it merged away, shortest first; then the sides round what changed turned where that
// makes their triangles rounder, the worst first. The next round looks again at what changed and the points beside it,
// until nothing does (up to eight rounds), so a second pass over the same place changes nothing. Sides are taken in a fixed order (by how
// much they ask for it, then by their points' numbers); at most `most` changes in all, so a dab stays quick (the next
// dab carries on).
template <class Inside>
void Sculptor::retopo(const std::vector<uint32_t> &seeds, Inside inside, bool mark, int most, bool later) {
  fresh.clear();
  if (!(strokeDetail > 0) || seeds.empty()) return;
  const double side = strokeDetail * sideOfDetail, hi = side * 4 / 3, lo = side * 0.6, hi2 = hi * hi, lo2 = lo * lo;
  struct Side {
    double key;
    uint32_t a, b;
  };
  // (The largest key first; between equal keys, the lowest points.)
  auto before = [](const Side &x, const Side &y) { return x.key != y.key ? x.key < y.key : x.a != y.a ? x.a > y.a : x.b > y.b; };
  std::vector<Side> heap, shortOnes;
  auto push = [&](double key, uint32_t u, uint32_t w) {
    heap.push_back({key, std::min(u, w), std::max(u, w)});
    std::push_heap(heap.begin(), heap.end(), before);
  };
  auto pop = [&]() {
    std::pop_heap(heap.begin(), heap.end(), before);
    Side s = heap.back();
    heap.pop_back();
    return s;
  };
  // (Each takes a corner at v, and looks at the side from v to the triangle's next point.)
  auto offerLong = [&](uint32_t v, uint32_t c) {
    uint32_t w = tri[nextC(c)];
    double l2 = norm2(p[v] - p[w]);
    if (l2 > hi2 && inside(v, w)) push(l2, v, w);
  };
  auto offerShort = [&](uint32_t v, uint32_t c) {
    uint32_t w = tri[nextC(c)];
    double l2 = norm2(p[v] - p[w]);
    if (l2 < lo2 && inside(v, w)) push(-l2, v, w);
  };
  auto offerTurn = [&](uint32_t v, uint32_t c) {
    uint32_t w = tri[nextC(c)], e = prevC(c);
    // (How much the angles facing it pass half a turn.)
    double k = cotangent(p[tri[e]], p[v], p[w]) + cotangent(p[tri[opp[e]]], p[v], p[w]);
    if (k < 0 && inside(v, w)) push(-k, v, w);
  };
  // Each side from v.
  auto from = [&](uint32_t v, auto offer) {
    uint32_t c = corner[v], x = c;
    do {
      offer(v, x);
      x = swing(x);
    } while (x != c);
  };
  int left = most;
  std::vector<uint32_t> &look = work, &changedHere = front;
  look.clear();
  for (uint32_t v : seeds)
    if (ptAlive[v]) look.push_back(v);
  for (int round = 0; round < 8 && left > 0 && !look.empty(); round++) {
    changedHere.clear();
    auto changedAll = [&](std::initializer_list<uint32_t> vs) {
      for (uint32_t v : vs) changedHere.push_back(v);
    };
    // Halved (the short sides found on the way, to merge after).
    heap.clear(), shortOnes.clear();
    for (uint32_t v : look) {
      uint32_t c = corner[v], x = c;
      do {
        uint32_t w = tri[nextC(x)];
        double l2 = norm2(p[v] - p[w]);
        if (l2 > hi2) {
          if (inside(v, w)) push(l2, v, w);
        } else if (l2 < lo2 && inside(v, w)) {
          shortOnes.push_back({-l2, std::min(v, w), std::max(v, w)});
        }
        x = swing(x);
      } while (x != c);
    }
    while (!heap.empty() && left > 0) {
      Side s = pop();
      if (!ptAlive[s.a] || !ptAlive[s.b] || !(norm2(p[s.a] - p[s.b]) > hi2)) continue;
      uint32_t e = sideOf(s.a, s.b), m;
      if (e == none) continue;
      uint32_t x = tri[e], y = tri[opp[e]];
      if (!split(e, m)) continue;
      left--;
      if (mark && region[s.a] == regionStamp && region[s.b] == regionStamp) region[m] = regionStamp;
      changedAll({m, s.a, s.b, x, y});
      from(m, offerLong);
    }
    // Merged (the point kept moved: what's round it changed too).
    heap.swap(shortOnes);
    std::make_heap(heap.begin(), heap.end(), before);
    for (size_t k = 0; k < changedHere.size(); k++)
      if (ptAlive[changedHere[k]]) from(changedHere[k], offerShort);
    while (!heap.empty() && left > 0) {
      Side s = pop();
      if (!ptAlive[s.a] || !ptAlive[s.b] || !(norm2(p[s.a] - p[s.b]) < lo2)) continue;
      uint32_t e = sideOf(s.a, s.b);
      if (e == none) continue;
      if (!collapse(e)) continue;
      left--;
      uint32_t c = corner[s.a], x = c;
      do {
        changedHere.push_back(tri[nextC(x)]);
        x = swing(x);
      } while (x != c);
      changedAll({s.a});
      from(s.a, offerShort);
    }
    // Turned, round what changed (and again round each side turned).
    heap.clear();
    std::sort(changedHere.begin(), changedHere.end());
    changedHere.erase(std::unique(changedHere.begin(), changedHere.end()), changedHere.end());
    for (size_t k = 0; k < changedHere.size(); k++)
      if (ptAlive[changedHere[k]]) from(changedHere[k], offerTurn);
    while (!heap.empty() && left > 0) {
      Side s = pop();
      if (!ptAlive[s.a] || !ptAlive[s.b]) continue;
      uint32_t e = sideOf(s.a, s.b);
      if (e == none) continue;
      uint32_t x = tri[e], y = tri[opp[e]];
      if (!flip(e)) continue;
      left--;
      changedAll({s.a, s.b, x, y});
      for (uint32_t v : {s.a, s.b, x, y}) from(v, offerTurn);
    }
    if (changedHere.empty()) break;
    // Next: what changed and the points beside it.
    if (++seenStamp == 0) std::fill(seen.begin(), seen.end(), 0), seenStamp = 1;
    look.clear();
    for (uint32_t v : changedHere) {
      if (!ptAlive[v]) continue;
      fresh.push_back(v);
      if (seen[v] != seenStamp) seen[v] = seenStamp, look.push_back(v);
      uint32_t c = corner[v], x = c;
      do {
        uint32_t w = tri[nextC(x)];
        if (seen[w] != seenStamp) seen[w] = seenStamp, look.push_back(w);
        x = swing(x);
      } while (x != c);
    }
    std::sort(look.begin(), look.end());
  }
  std::sort(fresh.begin(), fresh.end());
  fresh.erase(std::unique(fresh.begin(), fresh.end()), fresh.end());
  fresh.erase(std::remove_if(fresh.begin(), fresh.end(), [&](uint32_t v) { return !ptAlive[v]; }), fresh.end());
  if (later) refreshBoxes();
  else moved(fresh);
}

// Side e (the one facing corner e) halved: a point at its middle, its two triangles four. False when its two triangles
// are all there is of a piece.
bool Sculptor::split(uint32_t e, uint32_t &m) {
  uint32_t f = opp[e];
  uint32_t a = tri[nextC(e)], b = tri[prevC(e)], x = tri[e], y = tri[f];
  if (x == y) return false;
  uint32_t t1 = e / 3, t2 = f / 3;
  uint32_t o1 = opp[prevC(e)], o2 = opp[nextC(e)], o3 = opp[prevC(f)], o4 = opp[nextC(f)];
  for (uint32_t t : {t1, t2, o1 / 3, o2 / 3, o3 / 3, o4 / 3}) recordTriangle(t);
  for (uint32_t v : {a, b, x, y}) recordPoint(v);
  // Its middle, bowed out as the surface bends there (from its ends' normals; straight across a sharp bend).
  V3 mid = (p[a] + p[b]) / 2, ab = p[b] - p[a];
  if (dot(n[a], n[b]) >= 0.5) mid += (n[b] * dot(ab, n[b]) - n[a] * dot(ab, n[a])) / 8;
  m = newPoint(mid);
  uint32_t t3 = newTriangle(), t4 = newTriangle();
  // (x a b) and (y b a) become (x a m) (x m b) and (y b m) (y m a).
  auto set = [&](uint32_t t, uint32_t v0, uint32_t v1, uint32_t v2) { tri[3 * t] = v0, tri[3 * t + 1] = v1, tri[3 * t + 2] = v2; };
  auto pair = [&](uint32_t c, uint32_t d) { opp[c] = d, opp[d] = c; };
  set(t1, x, a, m), set(t3, x, m, b), set(t2, y, b, m), set(t4, y, m, a);
  pair(3 * t1 + 2, o1), pair(3 * t3 + 1, o2), pair(3 * t2 + 2, o3), pair(3 * t4 + 1, o4);
  pair(3 * t1, 3 * t4), pair(3 * t3, 3 * t2), pair(3 * t1 + 1, 3 * t3 + 2), pair(3 * t2 + 1, 3 * t4 + 2);
  corner[x] = 3 * t1, corner[a] = 3 * t1 + 1, corner[m] = 3 * t1 + 2, corner[b] = 3 * t3 + 2, corner[y] = 3 * t2;
  addToLeaf(t3, leafOf[t1]), addToLeaf(t4, leafOf[t2]);
  dirty(leafOf[t1]), dirty(leafOf[t2]);
  for (uint32_t t : {t1, t2, t3, t4}) touchTriangle(t);
  n[m] = normalAt(m);
  return true;
}

// Side e merged away: its two points one, at its middle, its two triangles gone. Only where that leaves the surface
// sound: the two points share no neighbour but the two across the side (so no side would be had twice), those two keep
// at least three neighbours, no triangle round them turns over or flattens, and no side comes out longer than a split's.
bool Sculptor::collapse(uint32_t e) {
  uint32_t f = opp[e];
  uint32_t a = tri[nextC(e)], b = tri[prevC(e)], x = tri[e], y = tri[f];
  if (x == y || live < 12) return false;
  uint32_t t1 = e / 3, t2 = f / 3;
  if (valence(x) < 4 || valence(y) < 4 || valence(a) + valence(b) - 4 > 12) return false;
  V3 m = (p[a] + p[b]) / 2;
  const double side = strokeDetail * sideOfDetail, hi2 = side * side * 16 / 9, tiny = 1e-12 * side * side * side * side;
  // Their corners, their neighbours no further than a split's side from the middle, and none shared but x and y.
  ring.clear();
  if (++seenStamp == 0) std::fill(seen.begin(), seen.end(), 0), seenStamp = 1;
  for (uint32_t v : {a, b}) {
    uint32_t c = corner[v], z = c;
    do {
      ring.push_back(z);
      uint32_t w = tri[nextC(z)];
      if (w != a && w != b) {
        if (norm2(p[w] - m) > hi2) return false;
        if (v == a) seen[w] = seenStamp;
        else if (seen[w] == seenStamp && w != x && w != y) return false;
      }
      z = swing(z);
    } while (z != c);
  }
  // Every other triangle at a or b keeps facing the way it did.
  for (uint32_t c : ring) {
    uint32_t t = c / 3;
    if (t == t1 || t == t2) continue;
    V3 q[3];
    for (int k = 0; k < 3; k++) q[k] = p[tri[3 * t + k]];
    V3 was = cross(q[1] - q[0], q[2] - q[0]);
    q[c % 3] = m;
    V3 now = cross(q[1] - q[0], q[2] - q[0]);
    // (Turned by no more than 45°, and not a sliver; a sliver before may go anywhere that faces out.)
    double d = dot(was, now);
    if (!(norm2(now) > tiny)) return false;
    if (norm2(was) > tiny ? !(d > 0 && d * d >= 0.5 * norm2(was) * norm2(now)) : !(dot(now, n[a]) > 0)) return false;
  }
  for (uint32_t c : ring) recordTriangle(c / 3);
  for (uint32_t v : {a, b, x, y}) recordPoint(v);
  uint32_t oBX = opp[nextC(e)], oXA = opp[prevC(e)], oAY = opp[nextC(f)], oYB = opp[prevC(f)];
  for (uint32_t c : ring)
    if (tri[c] == b) tri[c] = a;
  opp[oBX] = oXA, opp[oXA] = oBX, opp[oAY] = oYB, opp[oYB] = oAY;
  killTriangle(t1), killTriangle(t2);
  p[a] = m;
  ptAlive[b] = 0, corner[b] = none;
  freePts.push(b);
  if (!changedFlag[b]) changedFlag[b] = 1, changed.push_back(b);
  corner[a] = nextC(oXA), corner[x] = prevC(oXA), corner[y] = nextC(oAY);
  for (uint32_t c : ring) {
    uint32_t t = c / 3;
    if (triAlive[t]) touchTriangle(t), dirty(leafOf[t]);
  }
  return true;
}

// Side e turned to run between the two points across it: (x a b) and (y b a) become (x a y) and (y b x). Only where
// that makes the two rounder (the angles facing the new side add up to less than those facing the old), both were and
// stay about flat (within 20° of each other, so a crease stays), face the way they did, aren't slivers, the new side
// isn't there already and is within the detail's range, and a and b keep four neighbours or more.
bool Sculptor::flip(uint32_t e) {
  uint32_t f = opp[e];
  uint32_t a = tri[nextC(e)], b = tri[prevC(e)], x = tri[e], y = tri[f];
  if (x == y) return false;
  V3 A = p[a], B = p[b], X = p[x], Y = p[y];
  const double side = strokeDetail * sideOfDetail, hi2 = side * side * 16 / 9, lo2 = side * side * 0.36, tiny = 1e-12 * side * side * side * side;
  double xy2 = norm2(X - Y);
  if (!(xy2 >= lo2 && xy2 <= hi2)) return false;
  double was = cotangent(X, A, B) + cotangent(Y, A, B), now = cotangent(A, X, Y) + cotangent(B, X, Y);
  if (!(was < -1e-9 && now > was)) return false;
  V3 n1 = cross(A - X, B - X), n2 = cross(B - Y, A - Y), m1 = cross(A - X, Y - X), m2 = cross(B - Y, X - Y), up = n1 + n2;
  auto flat = [](V3 u, V3 v) {
    double d = dot(u, v);
    return d > 0 && d * d >= 0.883 * norm2(u) * norm2(v);
  };
  if (!(norm2(m1) > tiny && norm2(m2) > tiny) || !flat(n1, n2) || !flat(m1, m2) || !(dot(m1, up) > 0 && dot(m2, up) > 0)) return false;
  if (valence(a) < 5 || valence(b) < 5 || sideOf(x, y) != none) return false;
  uint32_t t1 = e / 3, t2 = f / 3;
  uint32_t o1 = opp[prevC(e)], o2 = opp[nextC(e)], o3 = opp[prevC(f)], o4 = opp[nextC(f)];
  for (uint32_t t : {t1, t2, o1 / 3, o2 / 3, o3 / 3, o4 / 3}) recordTriangle(t);
  for (uint32_t v : {a, b, x, y}) recordPoint(v);
  auto set = [&](uint32_t t, uint32_t v0, uint32_t v1, uint32_t v2) { tri[3 * t] = v0, tri[3 * t + 1] = v1, tri[3 * t + 2] = v2; };
  auto pair = [&](uint32_t c, uint32_t d) { opp[c] = d, opp[d] = c; };
  set(t1, x, a, y), set(t2, y, b, x);
  pair(3 * t1, o4), pair(3 * t1 + 2, o1), pair(3 * t2, o2), pair(3 * t2 + 2, o3), pair(3 * t1 + 1, 3 * t2 + 1);
  corner[x] = 3 * t1, corner[a] = 3 * t1 + 1, corner[y] = 3 * t2, corner[b] = 3 * t2 + 1;
  dirty(leafOf[t1]), dirty(leafOf[t2]);
  touchTriangle(t1), touchTriangle(t2);
  return true;
}

int Sculptor::valence(uint32_t v) const {
  uint32_t c = corner[v], z = c;
  int k = 0;
  do {
    k++;
    z = swing(z);
  } while (z != c);
  return k;
}

uint32_t Sculptor::newPoint(V3 at) {
  uint32_t v;
  if (!freePts.empty()) {
    v = freePts.top();
    freePts.pop();
  } else {
    v = (uint32_t)p.size();
    p.push_back(at), n.push_back(V3{0, 0, 0}), corner.push_back(none), ptAlive.push_back(0);
    seen.push_back(0), region.push_back(0), pointTouched.push_back(0), changedFlag.push_back(0);
  }
  recordPoint(v);
  p[v] = at, ptAlive[v] = 1;
  if (!changedFlag[v]) changedFlag[v] = 1, changed.push_back(v);
  return v;
}

uint32_t Sculptor::newTriangle() {
  uint32_t t;
  if (!freeTris.empty()) {
    t = freeTris.top();
    freeTris.pop();
  } else {
    t = (uint32_t)triAlive.size();
    for (int k = 0; k < 3; k++) tri.push_back(0), opp.push_back(none);
    triAlive.push_back(0), leafOf.push_back(-1), triTouched.push_back(0), changedTriFlag.push_back(0);
  }
  recordTriangle(t);
  triAlive[t] = 1, live++;
  return t;
}

void Sculptor::killTriangle(uint32_t t) {
  recordTriangle(t);
  removeFromLeaf(t);
  triAlive[t] = 0, live--;
  freeTris.push(t);
  touchTriangle(t);
}

void Sculptor::touchTriangle(uint32_t t) {
  if (!changedTriFlag[t]) changedTriFlag[t] = 1, changedTris.push_back(t);
}

void Sculptor::recordPoint(uint32_t v) {
  if (pointTouched[v] == strokeId) return;
  pointTouched[v] = strokeId;
  step.pIdx.push_back(v), step.pBefore.push_back({p[v], corner[v], ptAlive[v]});
}

void Sculptor::recordTriangle(uint32_t t) {
  if (triTouched[t] == strokeId) return;
  triTouched[t] = strokeId;
  const uint32_t *v = &tri[3 * t], *o = &opp[3 * t];
  step.tIdx.push_back(t), step.tBefore.push_back({{v[0], v[1], v[2]}, {o[0], o[1], o[2]}, triAlive[t]});
}

void Sculptor::end() {
  if (!stroking) return;
  // Grab: the stretched part made the detail size now the drag is over (the points it took and what's made between them).
  if (brush == Grab && strokeDetail > 0 && !grabbed.empty()) {
    if (++regionStamp == 0) std::fill(region.begin(), region.end(), 0), regionStamp = 1;
    for (uint32_t v : grabbed) region[v] = regionStamp;
    retopo(grabbed, [&](uint32_t a, uint32_t b) { return region[a] == regionStamp || region[b] == regionStamp; }, true, 400000, false);
  }
  stroking = false;
  Step s;
  {
    std::vector<uint32_t> by(step.pIdx.size());
    for (size_t k = 0; k < by.size(); k++) by[k] = (uint32_t)k;
    std::sort(by.begin(), by.end(), [&](uint32_t a, uint32_t b) { return step.pIdx[a] < step.pIdx[b]; });
    for (uint32_t k : by) {
      uint32_t v = step.pIdx[k];
      PointSlot now{p[v], corner[v], ptAlive[v]};
      if (now == step.pBefore[k]) continue;
      s.pIdx.push_back(v), s.pBefore.push_back(step.pBefore[k]), s.pAfter.push_back(now);
    }
  }
  {
    std::vector<uint32_t> by(step.tIdx.size());
    for (size_t k = 0; k < by.size(); k++) by[k] = (uint32_t)k;
    std::sort(by.begin(), by.end(), [&](uint32_t a, uint32_t b) { return step.tIdx[a] < step.tIdx[b]; });
    for (uint32_t k : by) {
      uint32_t t = step.tIdx[k];
      const uint32_t *v = &tri[3 * t], *o = &opp[3 * t];
      TriSlot now{{v[0], v[1], v[2]}, {o[0], o[1], o[2]}, triAlive[t]};
      if (now == step.tBefore[k]) continue;
      s.tIdx.push_back(t), s.tBefore.push_back(step.tBefore[k]), s.tAfter.push_back(now);
    }
  }
  step = Step();
  // (A box tree much grown or shrunk since it was made is made again: it keeps itself sound as it grows, its leaves
  // halved as they fill, but slowly loses its shape.)
  if (!s.tIdx.empty() && (grownLeaves > live / 8 || live > liveAtBuild * 2 || live < liveAtBuild / 2)) rebuildTree();
  if (s.size()) keep(std::move(s));
}

// Kept to undo: the last 100 strokes, or fewer when they've changed more than 4 million slots between them.
void Sculptor::keep(Step s) {
  for (const auto &u : undone) kept -= u.size();
  undone.clear();
  kept += s.size();
  done.push_back(std::move(s));
  size_t drop = 0;
  while (done.size() - drop > 1 && (done.size() - drop > 100 || kept > 4000000)) kept -= done[drop++].size();
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
  const auto &P = back ? s.pBefore : s.pAfter;
  const auto &T = back ? s.tBefore : s.tAfter;
  for (size_t k = 0; k < s.pIdx.size(); k++) {
    uint32_t v = s.pIdx[k];
    p[v] = P[k].at, corner[v] = P[k].corner, ptAlive[v] = P[k].alive;
  }
  std::vector<uint32_t> placed;
  for (size_t k = 0; k < s.tIdx.size(); k++) {
    uint32_t t = s.tIdx[k];
    for (int q = 0; q < 3; q++) tri[3 * t + q] = T[k].v[q], opp[3 * t + q] = T[k].o[q];
    if (triAlive[t] && !T[k].alive) removeFromLeaf(t);
    if (T[k].alive && leafOf[t] < 0) placed.push_back(t);
    else if (T[k].alive) dirty(leafOf[t]);
    triAlive[t] = T[k].alive;
    touchTriangle(t);
  }
  // Triangles back in use join the leaf of a neighbour already in the tree (the first found, in turn).
  while (!placed.empty()) {
    std::vector<uint32_t> later;
    for (uint32_t t : placed) {
      int leaf = -1;
      for (int q = 0; q < 3 && leaf < 0; q++) leaf = leafOf[opp[3 * t + q] / 3];
      if (leaf >= 0) addToLeaf(t, leaf);
      else later.push_back(t);
    }
    if (later.size() == placed.size()) {
      // (None beside one in the tree: the first leaf takes them.)
      int leaf = 0;
      while (nodes[leaf].left >= 0) leaf = nodes[leaf].left;
      for (uint32_t t : later) addToLeaf(t, leaf);
      break;
    }
    placed.swap(later);
  }
  std::vector<uint32_t> which;
  for (uint32_t v : s.pIdx) {
    if (ptAlive[v]) which.push_back(v);
    else if (!changedFlag[v]) changedFlag[v] = 1, changed.push_back(v);
  }
  if (!s.tIdx.empty()) {
    // The free slots as they now are.
    freeTris = {}, freePts = {};
    live = 0;
    for (uint32_t t = 0; t < triAlive.size(); t++) {
      if (triAlive[t]) live++;
      else freeTris.push(t);
    }
    for (uint32_t v = 0; v < ptAlive.size(); v++)
      if (!ptAlive[v]) freePts.push(v);
    for (uint32_t t : s.tIdx)
      if (triAlive[t])
        for (int q = 0; q < 3; q++) which.push_back(tri[3 * t + q]);
    std::sort(which.begin(), which.end());
    which.erase(std::unique(which.begin(), which.end()), which.end());
  }
  moved(which);
}

void Sculptor::moved(const std::vector<uint32_t> &which) {
  if (++seenStamp == 0) std::fill(seen.begin(), seen.end(), 0), seenStamp = 1;
  around.clear();
  for (uint32_t i : which) {
    if (!ptAlive[i]) continue;
    if (seen[i] != seenStamp) seen[i] = seenStamp, around.push_back(i);
    uint32_t c = corner[i], x = c;
    do {
      uint32_t t = x / 3;
      dirty(leafOf[t]);
      for (int q = 0; q < 3; q++) {
        uint32_t v = tri[3 * t + q];
        if (seen[v] != seenStamp) seen[v] = seenStamp, around.push_back(v);
      }
      x = swing(x);
    } while (x != c);
  }
  refreshBoxes();
  if (faceAt.size() != triAlive.size()) faceAt.resize(triAlive.size(), 0), faceN.resize(triAlive.size());
  if (++faceStamp == 0) std::fill(faceAt.begin(), faceAt.end(), 0), faceStamp = 1;
  for (uint32_t v : around) {
    n[v] = normalAt(v, true);
    if (!changedFlag[v]) changedFlag[v] = 1, changed.push_back(v);
  }
}

// Boxes made again from the leaves up (a node's children come after it).
void Sculptor::refreshBoxes() {
  std::sort(dirtyList.begin(), dirtyList.end(), std::greater<uint32_t>());
  for (uint32_t x : dirtyList) {
    TreeNode &nd = nodes[x];
    dirtyNode[x] = 0;
    if (nd.left >= 0) {
      nd.lo = vmin(nodes[nd.left].lo, nodes[nd.right].lo), nd.hi = vmax(nodes[nd.left].hi, nodes[nd.right].hi);
      continue;
    }
    nd.lo = V3{INFINITY, INFINITY, INFINITY}, nd.hi = V3{-INFINITY, -INFINITY, -INFINITY};
    for (uint32_t t : nd.items) {
      V3 a, b;
      boxOf(t, a, b);
      nd.lo = vmin(nd.lo, a), nd.hi = vmax(nd.hi, b);
    }
  }
  dirtyList.clear();
}

std::vector<uint32_t> Sculptor::takeChanged() {
  std::vector<uint32_t> out;
  out.swap(changed);
  std::sort(out.begin(), out.end());
  for (uint32_t v : out) changedFlag[v] = 0;
  return out;
}

std::vector<uint32_t> Sculptor::takeChangedTriangles() {
  std::vector<uint32_t> out;
  out.swap(changedTris);
  std::sort(out.begin(), out.end());
  for (uint32_t t : out) changedTriFlag[t] = 0;
  return out;
}

void Sculptor::compact(std::vector<V3> &pts, std::vector<uint32_t> &tris) const {
  pts.clear(), tris.clear();
  std::vector<uint32_t> number(p.size(), none);
  for (uint32_t v = 0; v < p.size(); v++)
    if (ptAlive[v]) number[v] = (uint32_t)pts.size(), pts.push_back(p[v]);
  for (uint32_t t = 0; t < triAlive.size(); t++)
    if (triAlive[t])
      for (int q = 0; q < 3; q++) tris.push_back(number[tri[3 * t + q]]);
}

std::string Sculptor::check() const {
  if (!good) return "not made";
  auto say = [](const char *what, size_t at) { return std::string(what) + " at " + std::to_string(at); };
  size_t alive = 0;
  std::vector<uint32_t> uses(p.size(), 0);
  for (uint32_t t = 0; t < triAlive.size(); t++) {
    if (!triAlive[t]) continue;
    alive++;
    for (int q = 0; q < 3; q++) {
      uint32_t c = 3 * t + q, o = opp[c], v = tri[c];
      if (v >= p.size() || !ptAlive[v]) return say("a corner at a point not in use", c);
      uses[v]++;
      if (o >= opp.size() || !triAlive[o / 3]) return say("a side facing a triangle not in use", c);
      if (opp[o] != c) return say("a side not met back", c);
      if (tri[nextC(o)] != tri[prevC(c)] || tri[prevC(o)] != tri[nextC(c)]) return say("a side met by another", c);
      if (!std::isfinite(p[v].x + p[v].y + p[v].z)) return say("a point not a number", v);
    }
    int leaf = leafOf[t];
    if (leaf < 0 || nodes[leaf].left >= 0 || std::find(nodes[leaf].items.begin(), nodes[leaf].items.end(), t) == nodes[leaf].items.end())
      return say("a triangle not in its leaf", t);
  }
  if (alive != live) return "the count of triangles is off";
  for (uint32_t v = 0; v < p.size(); v++) {
    if (!ptAlive[v]) {
      if (uses[v]) return say("a point not in use with corners", v);
      continue;
    }
    if (corner[v] >= tri.size() || !triAlive[corner[v] / 3] || tri[corner[v]] != v) return say("a point's corner elsewhere", v);
    uint32_t c = corner[v], x = c, count = 0;
    do {
      if (tri[x] != v) return say("a fan leaving its point", v);
      x = swing(x);
      if (++count > uses[v]) return say("a fan not closing", v);
    } while (x != c);
    if (count != uses[v]) return say("a point two fans meet at", v);
  }
  size_t held = 0;
  for (const auto &nd : nodes)
    for (uint32_t t : nd.items) {
      if (t >= triAlive.size() || !triAlive[t]) return say("a leaf holding a triangle not in use", t);
      held++;
    }
  if (held != live) return "the box tree holds a triangle twice";
  return "";
}

}  // namespace bce
