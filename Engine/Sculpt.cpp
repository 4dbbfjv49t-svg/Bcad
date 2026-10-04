// Sculpted bodies: a closed mesh made again at a detail. What it encloses is found on a grid by counting, along each of
// the grid's lines, where the line crosses the mesh (exactly, so a line grazing a side or a corner is counted right); the
// new surface is marching cubes on that, its points where the lines cross the mesh, then evened out.
#include "Engine/Sculpt.hpp"

#include <algorithm>
#include <cmath>
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
  // Inside or not at each grid point, by the lines along x.
  size_t N = (size_t)cells;
  std::vector<uint64_t> in((N + 63) / 64, 0);
  auto inside = [&](int i, int j, int k) {
    size_t id = g.index(i, j, k);
    return (in[id >> 6] >> (id & 63) & 1) != 0;
  };
  size_t inner = 0;
  {
    const Lines &L = lines[0];
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
  }
  if (!inner) return why = "nothing inside", false;
  // The cubes the surface passes through (some corners in, some out): about four triangles each.
  auto corners = [&](int i, int j, int k) {
    int m = 0;
    for (int c = 0; c < 8; c++) m |= (int)inside(i + (c & 1), j + (c >> 1 & 1), k + (c >> 2 & 1)) << c;
    return m;
  };
  size_t mixed = 0;
  for (int k = 0; k + 1 < g.n[2]; k++)
    for (int j = 0; j + 1 < g.n[1]; j++)
      for (int i = 0; i + 1 < g.n[0]; i++) {
        int m = corners(i, j, k);
        mixed += m != 0 && m != 255;
      }
  if (4 * mixed > most) return why = "too fine: about " + std::to_string(4 * mixed) + " triangles", false;
  lines[1] = cast(pts, tris, g, 1);
  lines[2] = cast(pts, tris, g, 2);

  // Marching cubes. On each face of a cube, the surface cuts off each run of inside corners (taken counter-clockwise seen
  // from outside the cube) by a side from where it goes in to where it comes out; inside corners diagonal on a face stay
  // apart. A face's choice is the same seen from both cubes, so the pieces meet, and each crossing point (one per grid
  // edge) is where one side ends and the next begins: they close into loops, each a polygon of the surface.
  static const Cube cube;
  std::unordered_map<uint64_t, uint32_t> pointOf;  // per grid edge crossed (its axis and lower end), its point
  pointOf.reserve(4 * mixed);
  auto point = [&](int i, int j, int k, int e) {
    int a = cube.edgeAxis[e], c = cube.edgeLow[e];
    int at[3] = {i + (c & 1), j + (c >> 1 & 1), k + (c >> 2 & 1)};
    uint64_t key = (uint64_t)a << 62 | g.index(at[0], at[1], at[2]);
    auto found = pointOf.find(key);
    if (found != pointOf.end()) return found->second;
    int up[3] = {at[0], at[1], at[2]};
    up[a]++;
    double f = crossingOn(lines[a], g, at[0], at[1], at[2], inside(up[0], up[1], up[2]));
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

  // Evened out: each point moved halfway to the middle of its neighbours, along the surface (not across it), twice.
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
  for (int pass = 0; pass < 2; pass++) {
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
  return true;
}

}  // namespace bce
