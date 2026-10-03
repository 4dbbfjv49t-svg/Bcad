// A shape cut by a plane, one side kept: every triangle clipped by the plane (which side each point is on decided exactly),
// the point where the plane crosses an edge made once for both triangles at that edge, and the cut outline filled as a new
// flat face — holes and all. A point on the plane (or a hair off it) counts as cut away, as if the plane were a hair into
// the kept side: a cut through a face leaves that face's plane as the new face, never a sheet of no thickness. Each piece
// of a triangle kept takes its share of the triangle's slivers, so a piece split again and again keeps its volume exact.
#include "Engine/Model.hpp"
#include "Engine/Triangulate.hpp"
#include "Engine/Weld.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace bce {

Solid cut(const Solid &s, V3 p, V3 n, int side) {
  Welded w = weld(s), out;
  out.pts = w.pts;
  double keepSign = side == 0 ? 1 : -1;
  std::vector<int> sd(w.pts.size());
  std::vector<double> dist(w.pts.size());
  // A point a hair off the plane (finer than anything measured: a turned shape's sines and cosines) counts as on it.
  double size = norm(p);
  for (V3 q : w.pts) size = std::max({size, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
  double hair = 1e-12 * (1 + size);
  for (size_t i = 0; i < w.pts.size(); i++) {
    dist[i] = dot(w.pts[i] - p, n) * keepSign;
    sd[i] = std::fabs(dist[i]) <= hair ? 0 : planeSide(w.pts[i], p, n) * (int)keepSign;
  }
  // The point where the plane crosses edge u–v (kept u, cut-away v): v itself when it's on the plane.
  std::unordered_map<uint64_t, uint32_t> crossing;
  auto across = [&](uint32_t u, uint32_t v) -> uint32_t {
    if (sd[v] == 0) return v;
    uint32_t a = std::min(u, v), b = std::max(u, v);
    uint64_t key = (uint64_t)a << 32 | b;
    auto it = crossing.find(key);
    if (it != crossing.end()) return it->second;
    double den = dist[a] - dist[b], t = den != 0 ? std::clamp(dist[a] / den, 0.0, 1.0) : 0.5;
    V3 q = w.pts[a] + (w.pts[b] - w.pts[a]) * t;
    uint32_t id = (uint32_t)out.pts.size();
    out.pts.push_back(q);
    crossing[key] = id;
    return id;
  };
  // The cut's outline, piece by piece, running the way the new face goes round (against the kept faces beside it).
  std::vector<std::pair<uint32_t, uint32_t>> outline;
  // A piece of triangle `from` (or of the new face, with no slivers: from < 0).
  auto emit = [&](uint32_t a, uint32_t b, uint32_t c, V3 na, V3 nb, V3 nc, uint32_t f, long from) {
    out.tri.insert(out.tri.end(), {a, b, c});
    out.nrm.insert(out.nrm.end(), {na, nb, nc});
    out.face.push_back(f);
    double g[6] = {0, 0, 0, 0, 0, 0};
    if (from >= 0 && !w.gap.empty()) {
      V3 q[3] = {out.pts[a], out.pts[b], out.pts[c]};
      gapOfPiece(&w.gap[6 * from], w.pts[w.tri[3 * from]], w.pts[w.tri[3 * from + 1]], w.pts[w.tri[3 * from + 2]], q, g);
    }
    out.gap.insert(out.gap.end(), g, g + 6);
  };
  for (size_t t = 0; t < w.count(); t++) {
    uint32_t v[3] = {w.tri[3 * t], w.tri[3 * t + 1], w.tri[3 * t + 2]};
    V3 nv[3] = {w.nrm[3 * t], w.nrm[3 * t + 1], w.nrm[3 * t + 2]};
    int kept = (sd[v[0]] > 0) + (sd[v[1]] > 0) + (sd[v[2]] > 0);
    if (kept == 3) {
      emit(v[0], v[1], v[2], nv[0], nv[1], nv[2], w.face[t], (long)t);
      continue;
    }
    if (kept == 0) continue;
    // Turned so the odd one out comes first.
    int k = 0;
    for (int i = 0; i < 3; i++)
      if ((sd[v[i]] > 0) == (kept == 1)) k = i;
    uint32_t a = v[k], b = v[(k + 1) % 3], c = v[(k + 2) % 3];
    V3 na = nv[k], nb = nv[(k + 1) % 3], nc = nv[(k + 2) % 3];
    auto normalAt = [&](uint32_t x, uint32_t y, V3 nx, V3 ny, uint32_t q) {
      double l = norm(w.pts[y] - w.pts[x]);
      double t = l > 0 ? norm(out.pts[q] - w.pts[x]) / l : 0;
      return unit(nx * (1 - t) + ny * t);
    };
    if (kept == 1) {
      // a kept: the corner by it stays.
      uint32_t ab = across(a, b), ac = across(a, c);
      V3 nab = normalAt(a, b, na, nb, ab), nac = normalAt(a, c, na, nc, ac);
      emit(a, ab, ac, na, nab, nac, w.face[t], (long)t);
      if (ab != ac) outline.push_back({ac, ab});
    } else {
      // a cut away: the rest stays, as two triangles.
      uint32_t ba = across(b, a), ca = across(c, a);
      V3 nba = normalAt(b, a, nb, na, ba), nca = normalAt(c, a, nc, na, ca);
      emit(b, c, ca, nb, nc, nca, w.face[t], (long)t);
      if (ca != ba) emit(b, ca, ba, nb, nca, nba, w.face[t], (long)t);
      if (ba != ca) outline.push_back({ba, ca});
    }
  }
  // Two triangles on the plane side by side (an edge of the shape lying in it) give the same piece both ways: neither is
  // part of the outline.
  {
    std::unordered_map<uint64_t, int> count;
    for (auto &e : outline) count[(uint64_t)e.first << 32 | e.second]++;
    std::vector<std::pair<uint32_t, uint32_t>> kept;
    for (auto &e : outline)
      if (!count.count((uint64_t)e.second << 32 | e.first)) kept.push_back(e);
    outline.swap(kept);
  }

  Solid result;
  result.faces = s.faces;
  if (!outline.empty()) {
    // The new face, flat, facing out of the kept side.
    V3 c = n * -keepSign;
    c = unit(c);
    V3 e1 = unit(std::fabs(c.x) < 0.9 ? cross(c, V3{1, 0, 0}) : cross(c, V3{0, 1, 0}));
    V3 e2 = cross(c, e1);
    double lo[2] = {INFINITY, INFINITY}, hi[2] = {-INFINITY, -INFINITY};
    // The outline's points in the order they come round it (so they go into the triangulation alike everywhere).
    std::vector<std::pair<uint32_t, std::pair<double, double>>> uv;
    std::unordered_map<uint32_t, int> seen;
    for (auto &e : outline)
      for (uint32_t q : {e.first, e.second}) {
        if (!seen.emplace(q, 1).second) continue;
        double u = dot(out.pts[q] - p, e1), v = dot(out.pts[q] - p, e2);
        uv.push_back({q, {u, v}});
        lo[0] = std::min(lo[0], u), lo[1] = std::min(lo[1], v), hi[0] = std::max(hi[0], u), hi[1] = std::max(hi[1], v);
      }
    double cx = (lo[0] + hi[0]) / 2, cy = (lo[1] + hi[1]) / 2, r = 10 * std::max({hi[0] - lo[0], hi[1] - lo[1], 1e-6});
    Tri2 fillIn(cx - 2 * r, cy - r, cx + 2 * r, cy - r, cx, cy + 2 * r);
    std::unordered_map<uint32_t, int> local;
    std::vector<uint32_t> global{0, 0, 0};
    for (auto &[q, at] : uv) {
      int i = fillIn.insert(at.first, at.second);
      local[q] = i;
      if (i >= (int)global.size()) global.resize(i + 1);
      global[i] = q;
    }
    for (auto &e : outline) fillIn.keep(local[e.first], local[e.second]);
    // Points the triangulation made where two pieces of the outline crossed (pieces a hair apart, crossed by rounding):
    // put on the first piece in space too, in the order they were made (one can lie on another).
    if (fillIn.count() > (int)global.size()) global.resize(fillIn.count(), UINT32_MAX);
    for (int i = 0; i < fillIn.count(); i++) {
      auto it = fillIn.made().find(i);
      if (it == fillIn.made().end()) continue;
      V3 r = out.pts[global[it->second.r]], l = out.pts[global[it->second.l]];
      global[i] = (uint32_t)out.pts.size();
      out.pts.push_back(r + (l - r) * it->second.s);
    }
    std::vector<int> tris = fillIn.insideKept();
    int f = (int)result.faces.size();
    Solid::Face face;
    face.normal = c;
    face.geom.kind = FaceGeom::Flat, face.geom.flat = true, face.geom.pn = c, face.geom.pd = dot(c, p);
    result.faces.push_back(face);
    for (size_t k = 0; k < tris.size(); k += 3) {
      uint32_t a = global[tris[k]], b = global[tris[k + 1]], d = global[tris[k + 2]];
      if (a != UINT32_MAX && b != UINT32_MAX && d != UINT32_MAX) emit(a, b, d, c, c, c, (uint32_t)f, -1);
    }
  }
  unweld(out, result);
  return result;
}

}  // namespace bce
