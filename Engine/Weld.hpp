// A mesh with its points shared between the triangles that meet at them (what merging and splitting work on), and back.
#pragma once
#include "Engine/Model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace bce {

struct Welded {
  std::vector<V3> pts;
  std::vector<uint32_t> tri;   // 3 per triangle
  std::vector<uint32_t> face;  // per triangle
  std::vector<V3> nrm;         // 3 per triangle: the normal at each corner (for shading)
  std::vector<double> gap;     // 6 per triangle, as Solid::gap (or none)
  size_t count() const { return face.size(); }
};

// Points in one place made one, numbered in the order they first come (a flat table, no allocation per point). With
// `step`, every point is first put on a grid that fine (see combine).
inline Welded weld(const Solid &s, double step = 0) {
  Welded w;
  size_t n = s.p.size(), cap = 16;
  while (cap < 2 * n) cap *= 2;
  std::vector<uint32_t> table(cap, UINT32_MAX), id(n);
  w.pts.reserve(n);
  for (size_t i = 0; i < n; i++) {
    V3 q = s.p[i];
    if (step > 0) q = {std::nearbyint(q.x / step) * step, std::nearbyint(q.y / step) * step, std::nearbyint(q.z / step) * step};
    // -0 and +0 are one place.
    q = {q.x + 0.0, q.y + 0.0, q.z + 0.0};
    uint64_t a, b, c;
    std::memcpy(&a, &q.x, 8), std::memcpy(&b, &q.y, 8), std::memcpy(&c, &q.z, 8);
    uint64_t h = (a * 0x9E3779B97F4A7C15ull) ^ (b * 0xC2B2AE3D27D4EB4Full) ^ (c * 0x165667B19E3779F9ull);
    h ^= h >> 29;
    for (size_t k = h & (cap - 1);; k = (k + 1) & (cap - 1)) {
      uint32_t at = table[k];
      if (at == UINT32_MAX) {
        table[k] = id[i] = (uint32_t)w.pts.size();
        w.pts.push_back(q);
        break;
      }
      if (w.pts[at].x == q.x && w.pts[at].y == q.y && w.pts[at].z == q.z) {
        id[i] = at;
        break;
      }
    }
  }
  w.tri.resize(s.tri.size());
  w.nrm.resize(s.tri.size());
  for (size_t k = 0; k < s.tri.size(); k++) w.tri[k] = id[s.tri[k]], w.nrm[k] = s.n[s.tri[k]];
  w.face = s.triFace;
  w.gap = s.gap;
  return w;
}

// Triangles thinner than `eps` (a point that far or less off the line between two others) swapped away: the long side
// for one from that point across to the neighbour's far corner (Csg.cpp).
void unneedle(Welded &w, double eps);
// Edges shorter than `eps` collapsed to a point where that keeps the surface closed (Csg.cpp).
void tidy(Welded &w, double eps);
// Whether every side of every triangle is met by one running the other way (point numbers, not places).
bool balanced(const Welded &w);

// Back to a mesh: a vertex per point and face (normals differ from face to face, and at a point where a surface comes to
// a tip); faces, their exact forms and deficits as given.
inline void unweld(const Welded &w, Solid &out) {
  out.p.clear(), out.n.clear(), out.tri.clear(), out.triFace.clear();
  size_t nc = w.tri.size(), np = w.pts.size();
  out.p.reserve(np * 2), out.n.reserve(np * 2), out.tri.resize(nc);
  // The corners at each point, in order; a corner joins an earlier one's vertex with the same face and normal.
  std::vector<uint32_t> start(np + 1, 0), at(nc);
  for (size_t k = 0; k < nc; k++) start[w.tri[k] + 1]++;
  for (size_t i = 0; i < np; i++) start[i + 1] += start[i];
  {
    std::vector<uint32_t> fill(start.begin(), start.end() - 1);
    for (size_t k = 0; k < nc; k++) at[fill[w.tri[k]]++] = (uint32_t)k;
  }
  for (size_t k = 0; k < nc; k++) {
    uint32_t pt = w.tri[k], f = w.face[k / 3];
    V3 n = w.nrm[k];
    uint32_t v = UINT32_MAX;
    for (uint32_t i = start[pt]; i < start[pt + 1] && at[i] < k; i++) {
      uint32_t j = at[i];
      if (w.face[j / 3] == f && w.nrm[j].x == n.x && w.nrm[j].y == n.y && w.nrm[j].z == n.z) {
        v = out.tri[j];
        break;
      }
    }
    if (v == UINT32_MAX) v = (uint32_t)out.p.size(), out.p.push_back(w.pts[pt]), out.n.push_back(n);
    out.tri[k] = v;
  }
  out.triFace = w.face;
  out.gap = w.gap;
}

}  // namespace bce
