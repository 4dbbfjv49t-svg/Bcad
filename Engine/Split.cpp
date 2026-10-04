// A shape cut by a plane, one side kept: what the shape has in common with a block standing on the plane on that side
// (a merge, so as exact and as surely closed as one). A point on the plane (or a hair off it) counts as cut away, as if
// the plane were a hair into the kept side: a cut through a face leaves that face's plane as the new face, never a sheet
// of no thickness. Each piece of a triangle kept takes its share of the triangle's slivers, so a piece split again and
// again keeps its volume exact.
#include "Engine/Model.hpp"
#include "Engine/Weld.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace bce {

namespace {

// Which side of the plane each point lies: 1 kept, −1 cut away, 0 on it (or a hair off it: finer than anything measured,
// a turned shape's sines and cosines).
std::vector<int> sides(const std::vector<V3> &pts, V3 p, V3 n, int side, std::vector<double> *dist = nullptr) {
  double keepSign = side == 0 ? 1 : -1;
  double size = norm(p);
  for (V3 q : pts) size = std::max({size, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
  double hair = 1e-12 * (1 + size);
  std::vector<int> sd(pts.size());
  if (dist) dist->resize(pts.size());
  for (size_t i = 0; i < pts.size(); i++) {
    double d = dot(pts[i] - p, n) * keepSign;
    if (dist) (*dist)[i] = d;
    sd[i] = std::fabs(d) <= hair ? 0 : planeSide(pts[i], p, n) * (int)keepSign;
  }
  return sd;
}

}  // namespace

Solid cut(const Solid &s, V3 p, V3 n, int side) {
  std::vector<int> sd = sides(s.p, p, n, side);
  bool kept = false, gone = false;
  for (int k : sd) (k > 0 ? kept : gone) = true;
  if (!gone) return s;
  if (!kept) {
    Solid none;
    none.faces = s.faces;
    return none;
  }
  // The kept side as a solid: a three-sided pyramid standing on the plane about the shape. The plane crosses the shape's
  // box, so what's kept lies within twice the box's half diagonal (r) above the plane and r of the box's middle across:
  // a base reaching 2r from its middle every way and a top 8r up hold that with r to spare.
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (V3 q : s.p) lo = vmin(lo, q), hi = vmax(hi, q);
  V3 mid = (lo + hi) * 0.5, u = unit(n) * (side == 0 ? 1.0 : -1.0);
  double r = norm(hi - lo) * 0.5;
  V3 m0 = mid - u * dot(mid - p, u);
  V3 e1 = unit(std::fabs(u.x) < 0.9 ? cross(u, V3{1, 0, 0}) : cross(u, V3{0, 1, 0})), e2 = cross(u, e1);
  const double h3 = 0.8660254037844386;
  V3 B[3] = {m0 + e2 * (4 * r), m0 + (e1 * -h3 - e2 * 0.5) * (4 * r), m0 + (e1 * h3 - e2 * 0.5) * (4 * r)}, top = m0 + u * (8 * r);
  Solid block;
  auto face = [&](V3 a, V3 b, V3 c, V3 opposite, bool base) {
    if (dot(cross(b - a, c - a), opposite - a) > 0) std::swap(b, c);
    V3 nf = base ? -u : unit(cross(b - a, c - a));
    int f = (int)block.faces.size();
    Solid::Face sf;
    sf.normal = nf;
    sf.geom.kind = FaceGeom::Flat, sf.geom.flat = true, sf.geom.pn = nf, sf.geom.pd = dot(nf, base ? p : a);
    block.faces.push_back(sf);
    uint32_t ia = block.vertex(a, nf), ib = block.vertex(b, nf), ic = block.vertex(c, nf);
    block.triangle(ia, ib, ic, f);
  };
  // The base first: the new face, numbered after the shape's own.
  face(B[0], B[1], B[2], top, true);
  for (int k = 0; k < 3; k++) face(B[k], B[(k + 1) % 3], top, B[(k + 2) % 3], false);
  // Merged on the grid the shape is on (a merge's result: its points stay where they are, and what it knows of its own
  // triangles not crossing holds), else on one as fine as doubles hold the plane to (a merge's own, for the block's size,
  // would move the new face by up to half its step) and the result on none, as a merge of it then puts it on its own.
  bool fine = s.grid <= 0;
  if (fine) {
    double scale = 1;
    for (const Solid *m : {&s, (const Solid *)&block})
      for (V3 q : m->p) scale = std::max({scale, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
    block.grid = std::ldexp(1.0, std::ilogb(scale) + 1 - 52);
  }
  Solid out = combine(s, block, BK_INTERSECT, 0, true);
  if (fine) out.grid = 0, out.sound.clear();
  // Of the block's faces only its base is left (its sides lie past the shape).
  size_t base = s.faces.size();
  for (auto &f : out.triFace) f = std::min<uint32_t>(f, (uint32_t)base);
  out.faces.resize(base + 1);
  return out;
}

std::vector<CutSide> cutOutline(const Solid &s, V3 p, V3 n, int side) {
  Welded w = weld(s);
  std::vector<double> dist;
  std::vector<int> sd = sides(w.pts, p, n, side, &dist);
  std::vector<V3> pts = w.pts;
  // The point where the plane crosses edge u–v (kept u, cut-away v): v itself when it's on the plane; made once for both
  // triangles at the edge.
  std::unordered_map<uint64_t, uint32_t> crossing;
  auto across = [&](uint32_t u, uint32_t v) -> uint32_t {
    if (sd[v] == 0) return v;
    uint32_t a = std::min(u, v), b = std::max(u, v);
    uint64_t key = (uint64_t)a << 32 | b;
    auto it = crossing.find(key);
    if (it != crossing.end()) return it->second;
    double den = dist[a] - dist[b], t = den != 0 ? std::clamp(dist[a] / den, 0.0, 1.0) : 0.5;
    uint32_t id = (uint32_t)pts.size();
    pts.push_back(w.pts[a] + (w.pts[b] - w.pts[a]) * t);
    crossing[key] = id;
    return id;
  };
  std::vector<std::pair<uint32_t, uint32_t>> outline;
  std::vector<int> faceOf;
  for (size_t t = 0; t < w.count(); t++) {
    uint32_t v[3] = {w.tri[3 * t], w.tri[3 * t + 1], w.tri[3 * t + 2]};
    int kept = (sd[v[0]] > 0) + (sd[v[1]] > 0) + (sd[v[2]] > 0);
    if (kept == 3 || kept == 0) continue;
    // Turned so the odd one out comes first; the side the kept piece leaves open, run the other way.
    int k = 0;
    for (int i = 0; i < 3; i++)
      if ((sd[v[i]] > 0) == (kept == 1)) k = i;
    uint32_t a = v[k], b = v[(k + 1) % 3], c = v[(k + 2) % 3];
    uint32_t from = kept == 1 ? across(a, c) : across(b, a), to = kept == 1 ? across(a, b) : across(c, a);
    if (from != to) outline.push_back({from, to}), faceOf.push_back((int)w.face[t]);
  }
  // Two triangles on the plane side by side (an edge of the shape lying in it) give the same piece both ways: neither is
  // part of the outline.
  std::unordered_map<uint64_t, int> count;
  for (auto &e : outline) count[(uint64_t)e.first << 32 | e.second]++;
  std::vector<CutSide> out;
  for (size_t i = 0; i < outline.size(); i++)
    if (!count.count((uint64_t)outline[i].second << 32 | outline[i].first)) out.push_back({pts[outline[i].first], pts[outline[i].second], faceOf[i]});
  return out;
}

}  // namespace bce
