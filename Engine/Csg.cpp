// Shapes as a tree: a primitive's mesh made from its model, a merge's or a split's from its parts' meshes (made at the same
// detail, and kept for when they're asked again); what's found again on a merged or split mesh (edges a hair long taken
// out, faces on one surface joined, edges, corners, circles); and how far a shape reaches and how much it holds — exactly
// where that can be told.
#include "Engine/Model.hpp"
#include "Engine/Treat.hpp"
#include "Engine/Weld.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <unordered_map>

namespace bce {

// MARK: - evaluating

std::shared_ptr<const Solid> evaluate(const Node &node, double d) {
  for (const auto &m : node.made)
    if (std::fabs(m.first - d) <= 1e-12 * d) return m.second;
  auto out = std::make_shared<Solid>();
  if (node.kind == Node::Prim) {
    node.model->build(*out, d);
    out->centroids();
    out->slivers();
  } else if (node.kind == Node::Treat) {
    Solid a;
    mesh(node.a, d, a);
    TreatFit fit;
    *out = treated(a, *node.treat, d, fit);
  } else {
    Solid a;
    mesh(node.a, d, a);
    if (node.kind == Node::Bool) {
      Solid b;
      mesh(node.b, d, b);
      *out = combine(a, b, node.op);
    } else {
      *out = cut(a, node.p, node.n, node.side);
    }
    finish(*out, d);
  }
  node.made.push_back({d, out});
  if (node.made.size() > 4) node.made.erase(node.made.begin());
  return out;
}

void mesh(const Shape &s, double deflection, Solid &out) {
  double grow = s.place.stretch();
  out = *evaluate(*s.node, grow > 0 ? deflection / grow : deflection);
  out.transform(s.place);
}

// MARK: - inside

namespace {

double winding(const Solid &s, V3 q) {
  double sum = 0;
  for (size_t k = 0; k < s.tri.size(); k += 3) {
    V3 a = s.p[s.tri[k]] - q, b = s.p[s.tri[k + 1]] - q, c = s.p[s.tri[k + 2]] - q;
    double la = norm(a), lb = norm(b), lc = norm(c);
    double det = dot(a, cross(b, c)), div = la * lb * lc + dot(a, b) * lc + dot(a, c) * lb + dot(b, c) * la;
    sum += 2 * std::atan2(det, div);
  }
  return sum / (4 * M_PI);
}

}  // namespace

bool inside(const Solid &s, V3 q) { return winding(s, q) > 0.5; }

// MARK: - pieces

int pieces(const Solid &s) {
  Welded w = weld(s);
  size_t nt = w.count();
  std::vector<uint32_t> parent(nt);
  std::iota(parent.begin(), parent.end(), 0);
  auto find = [&](uint32_t x) {
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
  };
  // Triangles sharing a side are one piece (sides sorted to find them).
  std::vector<std::pair<uint64_t, uint32_t>> sides;
  sides.reserve(3 * nt);
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) {
      uint32_t a = w.tri[3 * t + k], b = w.tri[3 * t + (k + 1) % 3];
      sides.push_back({(uint64_t)std::min(a, b) << 32 | std::max(a, b), (uint32_t)t});
    }
  std::sort(sides.begin(), sides.end());
  for (size_t k = 1; k < sides.size(); k++)
    if (sides[k].first == sides[k - 1].first) parent[find(sides[k].second)] = find(sides[k - 1].second);
  std::vector<double> vol(nt, 0);
  double scale = 0;
  for (const auto &p : w.pts) scale = std::max({scale, std::fabs(p.x), std::fabs(p.y), std::fabs(p.z)});
  for (size_t t = 0; t < nt; t++) vol[find((uint32_t)t)] += dot(w.pts[w.tri[3 * t]], cross(w.pts[w.tri[3 * t + 1]], w.pts[w.tri[3 * t + 2]])) / 6;
  int n = 0;
  for (size_t t = 0; t < nt; t++)
    if (find((uint32_t)t) == t && vol[t] > 1e-9 * (1 + scale * scale * scale)) n++;
  return n;
}

// MARK: - finishing a merged or split mesh

namespace {

bool sameSurface(const FaceGeom &a, const FaceGeom &b) {
  if (a.flat && b.flat) return norm(a.pn - b.pn) < 1e-9 && std::fabs(a.pd - b.pd) < 1e-9 * (1 + std::fabs(a.pd));
  if (a.kind == FaceGeom::Turned && b.kind == FaceGeom::Turned && a.exact && b.exact && !a.flat && !b.flat && a.elem.same(b.elem)) {
    for (int i = 0; i < 12; i++)
      if (std::fabs(a.place.m[i] - b.place.m[i]) > 1e-9 * (1 + std::fabs(a.place.m[i]))) return false;
    return true;
  }
  return false;
}

// A frame with z along `axis` at `origin`.
Affine frameAt(V3 origin, V3 axis) {
  V3 z = unit(axis), x = unit(std::fabs(z.x) < 0.9 ? cross(z, V3{1, 0, 0}) : cross(z, V3{0, 1, 0})), y = cross(z, x);
  Affine f;
  f.m[0] = x.x, f.m[1] = y.x, f.m[2] = z.x, f.m[3] = origin.x;
  f.m[4] = x.y, f.m[5] = y.y, f.m[6] = z.y, f.m[7] = origin.y;
  f.m[8] = x.z, f.m[9] = y.z, f.m[10] = z.z, f.m[11] = origin.z;
  return f;
}

// A line of points that is (part of) a circle, known exactly from the face it lies on: a turned face cut at one height
// along its axis (radius from the face's profile there), or a sphere cut by any plane.
struct Ring {
  EdgeGeom geom;
  V3 centre, axis;
  double radius, turn;  // turn: how far round the line goes (radians)
};

bool circleOn(const FaceGeom &g, const std::vector<V3> &pts, double deflection, Ring &out) {
  if (g.kind != FaceGeom::Turned || !g.exact || g.flat || pts.size() < 3) return false;
  double scale = 1;
  g.place.similarity(&scale);
  Affine back = g.place.inverse();
  std::vector<V3> q(pts.size());
  double size = 0, z = 0;
  for (size_t i = 0; i < pts.size(); i++) {
    q[i] = back.point(pts[i]);
    size = std::max({size, std::fabs(q[i].x), std::fabs(q[i].y), std::fabs(q[i].z)});
    z += q[i].z;
  }
  z /= q.size();
  double tol = 1e-9 * (1 + size);
  bool level = true;
  for (V3 v : q) level = level && std::fabs(v.z - z) <= tol;
  const Elem &el = g.elem;
  // In the face's own frame: the circle's middle and axis, the line's points about it, and the radius.
  V3 mid, ax{0, 0, 1};
  double r;
  if (level) {
    mid = {0, 0, z};
    if (!el.arc) {
      if (el.z0 == el.z1) return false;
      double t = (z - el.z0) / (el.z1 - el.z0);
      if (t < -1e-9 || t > 1 + 1e-9) return false;
      r = el.r0 + std::clamp(t, 0.0, 1.0) * (el.r1 - el.r0);
    } else {
      double rho = 0;
      for (V3 v : q) rho += std::hypot(v.x, v.y);
      rho /= q.size();
      double a = std::asin(std::clamp((z - el.cz) / el.rad, -1.0, 1.0));
      double r1 = el.cr + el.rad * std::cos(a), r2 = el.cr - el.rad * std::cos(a);
      r = std::fabs(r1 - rho) <= std::fabs(r2 - rho) ? r1 : r2;
    }
  } else {
    // A sphere: any flat line on it is a circle.
    if (!el.arc || el.cr != 0) return false;
    V3 c, n;
    for (V3 v : q) c += v;
    c = c / (double)q.size();
    for (size_t i = 0; i + 1 < q.size(); i++) n += cross(q[i] - c, q[i + 1] - c);
    if (norm(n) <= 0) return false;
    ax = unit(n);
    for (V3 v : q)
      if (std::fabs(dot(v - c, ax)) > tol) return false;
    V3 ball{0, 0, el.cz};
    double h = dot(ball - c, ax);
    mid = ball - ax * h;
    if (el.rad * el.rad - h * h <= 0) return false;
    r = std::sqrt(el.rad * el.rad - h * h);
  }
  // The mesh's points lie on chords inside the circle, by no more than the chord error.
  double turn = 0;
  V3 e1 = unit(std::fabs(ax.x) < 0.9 ? cross(ax, V3{1, 0, 0}) : cross(ax, V3{0, 1, 0})), e2 = cross(ax, e1);
  for (size_t i = 0; i < q.size(); i++) {
    double rho = norm((q[i] - mid) - ax * dot(q[i] - mid, ax));
    if (r <= tol || rho > r + tol || rho < r - 2 * deflection / scale - tol) return false;
    if (i == 0) continue;
    V3 u = q[i - 1] - mid, v = q[i] - mid;
    double d = std::atan2(dot(v, e2), dot(v, e1)) - std::atan2(dot(u, e2), dot(u, e1));
    turn += d > M_PI ? d - 2 * M_PI : d < -M_PI ? d + 2 * M_PI : d;
  }
  out.turn = turn;
  out.centre = g.place.point(mid), out.axis = unit(g.place.vector(ax)), out.radius = r * scale;
  out.geom.kind = EdgeGeom::Circle;
  if (level) {
    out.geom.place = g.place, out.geom.r = r, out.geom.z = z;
  } else {
    out.geom.place = frameAt(out.centre, out.axis), out.geom.r = out.radius, out.geom.z = 0;
  }
  return true;
}

// Edges far shorter than anything a printer or a screen tells apart (a cut a hair from a corner) collapsed to a point, so
// the mesh stays clean when it's handed on in floats. A collapse is made only where it keeps the surface closed, one
// sheet and unflipped; the point kept is the one on more faces (a corner before a point inside a face).
void tidy(Welded &w, double eps) {
  size_t nt = w.count();
  struct Short {
    double l2;
    uint32_t a, b;
  };
  std::vector<Short> shorts;
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) {
      uint32_t a = w.tri[3 * t + k], b = w.tri[3 * t + (k + 1) % 3];
      double l2 = norm2(w.pts[a] - w.pts[b]);
      if (a < b && l2 < eps * eps) shorts.push_back({l2, a, b});
    }
  if (shorts.empty()) return;
  std::sort(shorts.begin(), shorts.end(), [](const Short &x, const Short &y) { return x.l2 < y.l2; });
  std::vector<char> dead(nt, 0);
  std::vector<std::vector<uint32_t>> around(w.pts.size());
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) around[w.tri[3 * t + k]].push_back((uint32_t)t);
  auto has = [&](uint32_t t, uint32_t v) { return w.tri[3 * t] == v || w.tri[3 * t + 1] == v || w.tri[3 * t + 2] == v; };
  auto facesAt = [&](uint32_t v) {
    std::vector<uint32_t> f;
    for (uint32_t t : around[v])
      if (!dead[t] && std::find(f.begin(), f.end(), w.face[t]) == f.end()) f.push_back(w.face[t]);
    return f.size();
  };
  auto turned = [&](uint32_t t, uint32_t gone, V3 to) {
    V3 q[3];
    for (int k = 0; k < 3; k++) q[k] = w.pts[w.tri[3 * t + k]];
    V3 before = cross(q[1] - q[0], q[2] - q[0]);
    for (int k = 0; k < 3; k++)
      if (w.tri[3 * t + k] == gone) q[k] = to;
    return dot(before, cross(q[1] - q[0], q[2] - q[0])) < 0;
  };
  // Where each point went (itself until it's collapsed into another).
  std::vector<uint32_t> to(w.pts.size());
  std::iota(to.begin(), to.end(), 0);
  auto now = [&](uint32_t v) {
    while (to[v] != v) v = to[v] = to[to[v]];
    return v;
  };
  // A collapse can wait on another (a pinch undone by a neighbour's), so a few rounds.
  for (int round = 0; round < 4 && !shorts.empty(); round++) {
  std::vector<Short> left;
  for (const Short &e : shorts) {
    uint32_t a = now(e.a), b = now(e.b);
    if (a == b || norm2(w.pts[a] - w.pts[b]) >= eps * eps) continue;
    std::vector<uint32_t> both, onlyA, onlyB;
    for (uint32_t t : around[a])
      if (!dead[t]) (has(t, b) ? both : onlyA).push_back(t);
    for (uint32_t t : around[b])
      if (!dead[t] && !has(t, a)) onlyB.push_back(t);
    if (both.size() != 2) {
      if (!both.empty()) left.push_back(e);
      continue;
    }
    // Their neighbours in common must be just the two across the edge, or the collapse would pinch the surface.
    std::vector<uint32_t> na, opposite;
    for (uint32_t t : both)
      for (int k = 0; k < 3; k++)
        if (w.tri[3 * t + k] != a && w.tri[3 * t + k] != b) opposite.push_back(w.tri[3 * t + k]);
    if (opposite[0] == opposite[1]) continue;
    for (uint32_t t : onlyA)
      for (int k = 0; k < 3; k++) na.push_back(w.tri[3 * t + k]);
    bool pinch = false;
    for (uint32_t t : onlyB)
      for (int k = 0; k < 3 && !pinch; k++) {
        uint32_t v = w.tri[3 * t + k];
        pinch = v != b && v != opposite[0] && v != opposite[1] && std::find(na.begin(), na.end(), v) != na.end();
      }
    if (pinch) {
      left.push_back(e);
      continue;
    }
    uint32_t keep = a, gone = b;
    if (facesAt(b) > facesAt(a)) std::swap(keep, gone);
    auto flips = [&](uint32_t k, uint32_t g) {
      for (uint32_t t : g == a ? onlyA : onlyB)
        if (turned(t, g, w.pts[k])) return true;
      return false;
    };
    if (flips(keep, gone)) {
      std::swap(keep, gone);
      if (flips(keep, gone)) {
        left.push_back(e);
        continue;
      }
    }
    for (uint32_t t : both) dead[t] = 1;
    for (uint32_t t : gone == a ? onlyA : onlyB) {
      for (int k = 0; k < 3; k++)
        if (w.tri[3 * t + k] == gone) w.tri[3 * t + k] = keep;
      around[keep].push_back(t);
    }
    around[gone].clear();
    to[gone] = keep;
  }
  if (left.size() == shorts.size()) break;
  shorts.swap(left);
  }
  size_t n = 0;
  for (size_t t = 0; t < nt; t++) {
    if (dead[t]) continue;
    for (int k = 0; k < 3; k++) w.tri[3 * n + k] = w.tri[3 * t + k], w.nrm[3 * n + k] = w.nrm[3 * t + k];
    if (!w.gap.empty())
      for (int k = 0; k < 6; k++) w.gap[6 * n + k] = w.gap[6 * t + k];
    w.face[n++] = w.face[t];
  }
  w.tri.resize(3 * n), w.nrm.resize(3 * n), w.face.resize(n);
  if (!w.gap.empty()) w.gap.resize(6 * n);
}

}  // namespace

// Triangles thinner than anything told apart (a point a hair off the line between two others, where a cut grazes a side)
// done away with: the long side swapped for one from that point to the corner across it, the two triangles there taking
// the neighbour's face, normals and slivers. Swaps that would fold or double a side are left.
void unneedle(Welded &w, double eps) {
  size_t nt = w.count();
  for (int round = 0; round < 4; round++) {
    std::unordered_map<uint64_t, uint32_t> side;
    side.reserve(3 * nt);
    for (size_t t = 0; t < nt; t++)
      for (int k = 0; k < 3; k++) side[(uint64_t)w.tri[3 * t + k] << 32 | w.tri[3 * t + (k + 1) % 3]] = (uint32_t)t;
    std::vector<char> done(nt, 0);
    int swaps = 0;
    for (size_t t = 0; t < nt; t++) {
      if (done[t]) continue;
      // The longest side a → b, and c across it.
      int k = 0;
      double best = -1;
      for (int j = 0; j < 3; j++) {
        double l2 = norm2(w.pts[w.tri[3 * t + (j + 1) % 3]] - w.pts[w.tri[3 * t + j]]);
        if (l2 > best) best = l2, k = j;
      }
      uint32_t a = w.tri[3 * t + k], b = w.tri[3 * t + (k + 1) % 3], c = w.tri[3 * t + (k + 2) % 3];
      V3 A = w.pts[a], B = w.pts[b], C = w.pts[c], ab = B - A;
      if (!(best > 0) || norm(cross(ab, C - A)) / std::sqrt(best) >= eps) continue;
      double s = dot(C - A, ab) / best;
      if (!(s > 0 && s < 1)) continue;
      auto it = side.find((uint64_t)b << 32 | a);
      if (it == side.end() || done[it->second] || it->second == t) continue;
      uint32_t n = it->second;
      int kn = 0;
      while (w.tri[3 * n + kn] != b) kn++;
      uint32_t d = w.tri[3 * n + (kn + 2) % 3];
      V3 D = w.pts[d], up = cross(A - B, D - B);
      if (norm(up) / std::sqrt(best) < eps || side.count((uint64_t)c << 32 | d) || side.count((uint64_t)d << 32 | c)) continue;
      if (dot(cross(D - A, C - A), up) <= 0 || dot(cross(B - D, C - D), up) <= 0) continue;
      // The neighbour's corners, normals and slivers, as they were.
      V3 q[3] = {w.pts[w.tri[3 * n]], w.pts[w.tri[3 * n + 1]], w.pts[w.tri[3 * n + 2]]};
      V3 nb = w.nrm[3 * n + kn], na = w.nrm[3 * n + (kn + 1) % 3], nd = w.nrm[3 * n + (kn + 2) % 3];
      V3 nc = unit(na * (1 - s) + nb * s);
      double g[6];
      bool gaps = !w.gap.empty();
      if (gaps) std::copy(&w.gap[6 * n], &w.gap[6 * n] + 6, g);
      uint32_t f = w.face[n];
      auto put = [&](size_t at, uint32_t x, uint32_t y, uint32_t z, V3 nx, V3 ny, V3 nz) {
        w.tri[3 * at] = x, w.tri[3 * at + 1] = y, w.tri[3 * at + 2] = z;
        w.nrm[3 * at] = nx, w.nrm[3 * at + 1] = ny, w.nrm[3 * at + 2] = nz;
        w.face[at] = f;
        if (gaps) {
          V3 piece[3] = {w.pts[x], w.pts[y], w.pts[z]};
          gapOfPiece(g, q[0], q[1], q[2], piece, &w.gap[6 * at]);
        }
        done[at] = 1;
      };
      put(t, a, d, c, na, nd, nc);
      put(n, d, b, c, nd, nb, nc);
      for (int j = 0; j < 3; j++)
        for (size_t at : {t, (size_t)n}) side[(uint64_t)w.tri[3 * at + j] << 32 | w.tri[3 * at + (j + 1) % 3]] = (uint32_t)at;
      side.erase((uint64_t)a << 32 | b), side.erase((uint64_t)b << 32 | a);
      swaps++;
    }
    if (swaps == 0) break;
  }
}

void finish(Solid &s, double deflection) {
  Welded w = weld(s);
  {
    double size = 0;
    for (V3 q : w.pts) size = std::max({size, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
    tidy(w, std::max(1e-4, 2e-7 * size));
    unneedle(w, std::max(1e-4, 2e-7 * size));
  }
  size_t nt = w.count();
  // Edges between triangles: which triangles, so faces meeting on one surface can be joined and edges found.
  // (Sorted pairs grouped, rather than a map of lists: one allocation, not one per edge.)
  struct Span {
    const uint32_t *p;
    size_t n;
    size_t size() const { return n; }
    uint32_t operator[](size_t i) const { return p[i]; }
  };
  std::vector<std::pair<uint64_t, uint32_t>> sides;
  sides.reserve(3 * nt);
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) {
      uint32_t a = w.tri[3 * t + k], b = w.tri[3 * t + (k + 1) % 3];
      sides.push_back({(uint64_t)std::min(a, b) << 32 | std::max(a, b), (uint32_t)t});
    }
  std::sort(sides.begin(), sides.end());
  std::vector<uint32_t> sideTris(sides.size());
  for (size_t i = 0; i < sides.size(); i++) sideTris[i] = sides[i].second;
  std::vector<std::pair<uint64_t, Span>> edgeTris;
  edgeTris.reserve(sides.size() / 2 + 1);
  for (size_t i = 0, j; i < sides.size(); i = j) {
    for (j = i + 1; j < sides.size() && sides[j].first == sides[i].first;) j++;
    edgeTris.push_back({sides[i].first, Span{&sideTris[i], j - i}});
  }
  // A face left in separate patches (a side cut through, the two ends of a ring's cut) is a face per patch, its deficit
  // shared by area.
  {
    std::vector<int> patch(nt);
    std::iota(patch.begin(), patch.end(), 0);
    auto top = [&](int x) {
      while (patch[x] != x) x = patch[x] = patch[patch[x]];
      return x;
    };
    for (auto &[key, ts] : edgeTris)
      for (size_t i = 1; i < ts.size(); i++)
        if (w.face[ts[0]] == w.face[ts[i]]) patch[top((int)ts[i])] = top((int)ts[0]);
    std::vector<double> area(nt), faceArea(s.faces.size(), 0);
    for (size_t t = 0; t < nt; t++) {
      area[t] = norm(cross(w.pts[w.tri[3 * t + 1]] - w.pts[w.tri[3 * t]], w.pts[w.tri[3 * t + 2]] - w.pts[w.tri[3 * t]]));
      faceArea[w.face[t]] += area[t];
    }
    // The first patch of a face keeps its number; any more get new ones.
    std::unordered_map<int, uint32_t> number;
    std::vector<char> taken(s.faces.size(), 0);
    std::vector<uint32_t> source(s.faces.size());
    std::iota(source.begin(), source.end(), 0);
    std::vector<double> patchArea(s.faces.size(), 0);
    std::vector<Solid::Face> faces = s.faces;
    for (size_t t = 0; t < nt; t++) {
      auto it = number.find(top((int)t));
      if (it == number.end()) {
        uint32_t f = w.face[t], id = f;
        if (taken[f]) id = (uint32_t)faces.size(), faces.push_back(s.faces[f]), source.push_back(f), patchArea.push_back(0);
        taken[f] = 1;
        it = number.emplace(top((int)t), id).first;
      }
      patchArea[it->second] += area[t];
      w.face[t] = it->second;
    }
    for (size_t f = 0; f < faces.size(); f++) {
      double whole = faceArea[source[f]];
      faces[f].deficit = whole > 0 ? s.faces[source[f]].deficit * patchArea[f] / whole : 0;
    }
    s.faces = faces;
  }
  size_t nf = s.faces.size();
  std::vector<int> parent(nf);
  std::iota(parent.begin(), parent.end(), 0);
  auto find = [&](int x) {
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
  };
  for (auto &[key, ts] : edgeTris)
    for (size_t i = 1; i < ts.size(); i++) {
      int fa = find((int)w.face[ts[0]]), fb = find((int)w.face[ts[i]]);
      if (fa != fb && sameSurface(s.faces[fa].geom, s.faces[fb].geom)) parent[fb] = fa;
    }
  // Faces renumbered: only those with triangles, joined ones as one.
  std::vector<int> used(nf, 0), number(nf, -1);
  for (size_t t = 0; t < nt; t++) used[find((int)w.face[t])] = 1;
  std::vector<Solid::Face> faces;
  for (size_t f = 0; f < nf; f++)
    if (used[f] && find((int)f) == (int)f) number[f] = (int)faces.size(), faces.push_back(s.faces[f]), faces.back().deficit = 0;
  for (size_t f = 0; f < nf; f++)
    if (number[find((int)f)] >= 0) faces[number[find((int)f)]].deficit += s.faces[f].deficit;
  for (size_t t = 0; t < nt; t++) w.face[t] = (uint32_t)number[find((int)w.face[t])];
  s.faces = faces;

  // Edges: where two faces meet, chained into lines. (Sorted lists throughout, not maps: no allocation per point.)
  struct Link {
    int fa, fb;
    uint32_t a, b;
  };
  std::vector<Link> links;
  for (auto &[key, ts] : edgeTris) {
    if (ts.size() != 2) continue;
    int fa = (int)w.face[ts[0]], fb = (int)w.face[ts[1]];
    if (fa == fb) continue;
    links.push_back({std::min(fa, fb), std::max(fa, fb), (uint32_t)(key >> 32), (uint32_t)(key & 0xffffffffu)});
  }
  std::sort(links.begin(), links.end(), [](const Link &x, const Link &y) {
    return x.fa != y.fa ? x.fa < y.fa : x.fb != y.fb ? x.fb < y.fb : x.a != y.a ? x.a < y.a : x.b < y.b;
  });
  s.edges.clear(), s.circles.clear(), s.corners.clear();
  std::vector<std::array<uint32_t, 3>> ends;  // (point, the other end, piece) both ways round, by point
  std::vector<char> walked;
  for (size_t i = 0, j; i < links.size(); i = j) {
    for (j = i + 1; j < links.size() && links[j].fa == links[i].fa && links[j].fb == links[i].fb;) j++;
    size_t n = j - i;
    ends.clear();
    for (size_t k = 0; k < n; k++) {
      const Link &l = links[i + k];
      ends.push_back({l.a, l.b, (uint32_t)k}), ends.push_back({l.b, l.a, (uint32_t)k});
    }
    std::sort(ends.begin(), ends.end());
    walked.assign(n, 0);
    auto at = [&](uint32_t pt) {
      auto lo = std::lower_bound(ends.begin(), ends.end(), std::array<uint32_t, 3>{pt, 0, 0});
      auto hi = lo;
      while (hi != ends.end() && (*hi)[0] == pt) ++hi;
      return std::make_pair(lo, hi);
    };
    // Lines from their ends first (points not met by exactly two pieces), then the closed loops left.
    std::vector<uint32_t> starts;
    for (int pass = 0; pass < 2; pass++)
      for (size_t k = 0, m; k < ends.size(); k = m) {
        for (m = k + 1; m < ends.size() && ends[m][0] == ends[k][0];) m++;
        if ((m - k != 2) == (pass == 0)) starts.push_back(ends[k][0]);
      }
    for (uint32_t st : starts) {
      auto [lo, hi] = at(st);
      for (auto it = lo; it != hi; ++it) {
        uint32_t piece = (*it)[2];
        if (walked[piece]) continue;
        Solid::Edge e;
        e.f0 = links[i].fa, e.f1 = links[i].fb;
        e.pts.push_back(w.pts[st]);
        uint32_t cur = (*it)[1];
        walked[piece] = 1;
        e.pts.push_back(w.pts[cur]);
        while (cur != st) {
          auto [clo, chi] = at(cur);
          if (chi - clo != 2) break;
          const auto &next = (*clo)[2] == piece ? *(clo + 1) : *clo;
          if (walked[next[2]]) break;
          piece = next[2], walked[piece] = 1, cur = next[1];
          e.pts.push_back(w.pts[cur]);
        }
        // A line on a turned face at one height along its axis is a circle round that axis (a hole's rim, a cut across a
        // cylinder): exact again from the face's profile. Half a turn or more is listed as a circle, as OpenCascade lists them.
        for (int f : {e.f0, e.f1}) {
          Ring ring;
          if (!circleOn(faces[f].geom, e.pts, deflection, ring)) continue;
          e.geom = ring.geom;
          if (std::fabs(ring.turn) >= M_PI - 1e-6) s.circles.push_back({ring.centre, ring.axis, ring.radius});
          break;
        }
        if (e.geom.kind != EdgeGeom::Circle && e.pts.size() == 2) e.geom.kind = EdgeGeom::Line;
        s.edges.push_back(e);
      }
    }
  }
  // Corners: points where three or more faces meet.
  {
    std::vector<uint64_t> at;
    at.reserve(3 * nt);
    for (size_t t = 0; t < nt; t++)
      for (int k = 0; k < 3; k++) at.push_back((uint64_t)w.tri[3 * t + k] << 32 | w.face[t]);
    std::sort(at.begin(), at.end());
    at.erase(std::unique(at.begin(), at.end()), at.end());
    for (size_t k = 0, m; k < at.size(); k = m) {
      for (m = k + 1; m < at.size() && at[m] >> 32 == at[k] >> 32;) m++;
      if (m - k >= 3) s.corners.push_back(w.pts[at[k] >> 32]);
    }
  }
  unweld(w, s);
  s.centroids();
  // Each face's deficit: its triangles' slivers.
  if (!s.gap.empty()) {
    for (auto &f : s.faces) f.deficit = 0;
    for (size_t t = 0; t < s.triFace.size(); t++) s.faces[s.triFace[t]].deficit += s.sliver(t);
  }
  // Each face's normal: its plane's, or its own at the triangle nearest its middle.
  std::vector<double> best(s.faces.size(), INFINITY);
  for (size_t t = 0; t < s.triFace.size(); t++) {
    auto &f = s.faces[s.triFace[t]];
    if (f.geom.flat) {
      f.normal = f.geom.pn;
      continue;
    }
    V3 c = (s.p[s.tri[3 * t]] + s.p[s.tri[3 * t + 1]] + s.p[s.tri[3 * t + 2]]) / 3;
    double d = norm2(c - f.centroid);
    if (d < best[s.triFace[t]]) best[s.triFace[t]] = d, f.normal = unit(s.n[s.tri[3 * t]] + s.n[s.tri[3 * t + 1]] + s.n[s.tri[3 * t + 2]]);
  }
  (void)deflection;
}

// MARK: - reach, box and volume

namespace {

// How far a node reaches along d (its own coordinates), exactly where that can be told; otherwise a bound, not exact
// (the box then comes from the mesh at hand).
Reach reach(const Node &node, V3 d, bool prove) {
  if (node.kind == Node::Prim) {
    V3 at;
    double v = node.model->support(d, &at);
    return {v, at, true};
  }
  // Whether a point is inside a part as shown: its kept mesh, the point taken back into the part's own frame (no copy).
  auto within = [&](const Shape &s, V3 q) {
    double grow = s.place.stretch();
    return inside(*evaluate(*s.node, grow > 0 ? 0.05 / grow : 0.05), s.place.inverse().point(q));
  };
  Reach a = support(node.a, d, prove);
  if (node.kind == Node::Treat) {
    // Edges taken off (or filled in): no farther than the shape, not exactly as far.
    a.exact = false;
    return a;
  }
  if (node.kind == Node::Split) {
    // Exact while the farthest point stays on the kept side (or on the plane, the edge of it).
    a.exact = a.exact && planeSide(a.point, node.p, node.n) * (node.side == 0 ? 1 : -1) >= 0;
    return a;
  }
  Reach b = support(node.b, d, prove);
  if (node.op == BK_UNION) {
    Reach r = a.value >= b.value ? a : b;
    r.exact = a.exact && b.exact;
    return r;
  }
  if (node.op == BK_SUBTRACT) {
    a.exact = a.exact && prove && !within(node.b, a.point);
    return a;
  }
  if (prove && a.exact && within(node.b, a.point)) return a;
  if (prove && b.exact && within(node.a, b.point)) return b;
  Reach r = a.value <= b.value ? a : b;
  r.exact = false;
  return r;
}

}  // namespace

Reach support(const Shape &s, V3 d, bool prove) {
  V3 local{dot(s.place.column(0), d), dot(s.place.column(1), d), dot(s.place.column(2), d)};
  Reach r = reach(*s.node, local, prove);
  V3 t{s.place.m[3], s.place.m[7], s.place.m[11]};
  return {r.value + dot(t, d), s.place.point(r.point), r.exact};
}

bool placedBounds(const Shape &s, V3 &lo, V3 &hi) {
  bool exact[6], all = true;
  for (int i = 0; i < 3; i++) {
    V3 e{i == 0 ? 1.0 : 0, i == 1 ? 1.0 : 0, i == 2 ? 1.0 : 0};
    Reach up = support(s, e, false), down = support(s, -e, false);
    hi[i] = up.value, lo[i] = -down.value;
    exact[i] = down.exact, exact[3 + i] = up.exact;
    all = all && up.exact && down.exact;
  }
  if (all) return true;
  // The rest from the points of the shape's display mesh (kept from when it was shown), placed: short of the shape by no
  // more than its chord error.
  double grow = s.place.stretch();
  auto shown = evaluate(*s.node, grow > 0 ? 0.05 / grow : 0.05);
  V3 a{INFINITY, INFINITY, INFINITY}, b{-INFINITY, -INFINITY, -INFINITY};
  for (V3 p : shown->p) {
    V3 q = s.place.point(p);
    a = vmin(a, q), b = vmax(b, q);
  }
  if (shown->p.empty()) a = b = V3{s.place.m[3], s.place.m[7], s.place.m[11]};
  for (int i = 0; i < 3; i++) {
    if (!exact[i]) lo[i] = a[i];
    if (!exact[3 + i]) hi[i] = b[i];
  }
  return false;
}

int pieceCount(const Shape &s) {
  if (s.node->kind == Node::Prim) return 1;
  double grow = s.place.stretch(), d = grow > 0 ? 0.05 / grow : 0.05;
  for (const auto &c : s.node->counted)
    if (std::fabs(c.first - d) <= 1e-12 * d) return c.second;
  // Pieces don't change with a placement: counted on the node's own mesh, no copy.
  int n = pieces(*evaluate(*s.node, d));
  s.node->counted.push_back({d, n});
  if (s.node->counted.size() > 4) s.node->counted.erase(s.node->counted.begin());
  return n;
}

void bounds(const Shape &s, V3 &lo, V3 &hi, const Solid *meshed) {
  for (int i = 0; i < 3; i++) {
    V3 e{i == 0 ? 1.0 : 0, i == 1 ? 1.0 : 0, i == 2 ? 1.0 : 0};
    Reach up = support(s, e), down = support(s, -e);
    hi[i] = up.value, lo[i] = -down.value;
    if (meshed && (!up.exact || !down.exact)) {
      double a = INFINITY, b = -INFINITY;
      for (const auto &p : meshed->p) a = std::min(a, p[i]), b = std::max(b, p[i]);
      if (!up.exact) hi[i] = b;
      if (!down.exact) lo[i] = a;
    }
  }
}

double volume(const Shape &s, const Solid *meshed) {
  if (s.node->kind == Node::Prim) return s.node->model->volume * std::fabs(s.place.det());
  Solid own;
  if (!meshed) mesh(s, 0.01, own), meshed = &own;
  double v = meshed->meshVolume();
  for (const auto &f : meshed->faces) v += f.deficit;
  return v;
}

}  // namespace bce
