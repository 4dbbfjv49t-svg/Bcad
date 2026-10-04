// Closed meshes as files hold them. Shells: points welded where they're one to the last bit, each side met by the one
// bounding the same wedge of material, and the shells and voids those joins make (STEP writes them as they are). A print
// mesh: the same, with a point where parts touch given once per part, so every edge lies between exactly two triangles
// (as 3MF requires and slicers check), checked in float, as the file will hold it.
#include "Engine/Print.hpp"
#include "Engine/Treat.hpp"
#include "Engine/Trig.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace bce {

namespace {

struct Key {
  uint64_t x, y, z;
  bool operator==(const Key &k) const { return x == k.x && y == k.y && z == k.z; }
};
struct Hash {
  size_t operator()(const Key &k) const { return (size_t)(k.x * 0x9E3779B97F4A7C15ull ^ k.y * 0xC2B2AE3D27D4EB4Full ^ k.z * 0x165667B19E3779F9ull); }
};
template <typename T> Key key(T x, T y, T z) {
  // (+0 makes -0 and 0 one.)
  x += 0, y += 0, z += 0;
  Key k{0, 0, 0};
  std::memcpy(&k.x, &x, sizeof x), std::memcpy(&k.y, &y, sizeof y), std::memcpy(&k.z, &z, sizeof z);
  return k;
}

}  // namespace

bool shells(const Solid &s, Shells &out, std::string &why) {
  out = Shells();
  if (s.tri.empty()) return why = "empty", false;
  if (!shut(s)) return why = "open", false;
  auto &P = out.P;
  auto &T = out.T;
  T.resize(s.tri.size());
  {
    std::unordered_map<Key, uint32_t, Hash> at;
    for (size_t i = 0; i < s.tri.size(); i++) {
      V3 q = s.p[s.tri[i]];
      auto [it, fresh] = at.emplace(key(q.x, q.y, q.z), (uint32_t)P.size());
      if (fresh) P.push_back(q);
      T[i] = it->second;
    }
  }
  size_t nt = T.size() / 3, ns = T.size();
  for (V3 q : P) out.size = std::max({out.size, std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});

  // Each side met by one running the other way, as one edge. Along a line where more than two triangles meet (parts
  // touching there), each is met by the one that bounds the same wedge of material: in the turning order round the line,
  // material lies just past a triangle running back along it, and just short of one running forward.
  auto &mate = out.mate, &edgeOf = out.edgeOf;
  mate.assign(ns, 0), edgeOf.assign(ns, 0);
  {
    std::vector<std::pair<uint64_t, uint32_t>> sides(ns);
    for (uint32_t i = 0; i < ns; i++) {
      uint32_t a = out.from(i), b = out.to(i);
      sides[i] = {(uint64_t)std::min(a, b) << 32 | std::max(a, b), i};
    }
    std::sort(sides.begin(), sides.end());
    for (size_t i = 0, j; i < ns; i = j) {
      for (j = i + 1; j < ns && sides[j].first == sides[i].first;) j++;
      uint32_t u = (uint32_t)(sides[i].first >> 32), v = (uint32_t)(sides[i].first & 0xffffffffu);
      std::vector<uint32_t> fwd, back;
      for (size_t k = i; k < j; k++) (out.from(sides[k].second) == u ? fwd : back).push_back(sides[k].second);
      std::vector<std::pair<uint32_t, uint32_t>> pairs;
      if (fwd.size() == 1 && back.size() == 1) {
        pairs.push_back({fwd[0], back[0]});
      } else {
        V3 d = unit(P[v] - P[u]);
        V3 e1 = unit(std::fabs(d.x) < 0.9 ? cross(d, V3{1, 0, 0}) : cross(d, V3{0, 1, 0})), e2 = cross(d, e1);
        struct Round {
          double angle;
          uint32_t side;
          bool fwd;
        };
        std::vector<Round> round;
        for (auto *group : {&fwd, &back})
          for (uint32_t side : *group) {
            V3 w = P[out.across(side)] - P[u];
            w = w - d * dot(w, d);
            round.push_back({trig::atan2(dot(w, e2), dot(w, e1)), side, group == &fwd});
          }
        std::sort(round.begin(), round.end(), [](const Round &a, const Round &b) { return a.angle != b.angle ? a.angle < b.angle : a.side < b.side; });
        size_t n = round.size(), first = 0;
        while (first < n && round[first].fwd) first++;
        bool alternate = n % 2 == 0 && first < n;
        for (size_t k = 0; k < n && alternate; k++) alternate = round[(first + k) % n].fwd == (k % 2 == 1);
        if (alternate) {
          for (size_t k = 0; k < n; k += 2) pairs.push_back({round[(first + k + 1) % n].side, round[(first + k) % n].side});
        } else {
          // (Never expected of a closed solid: met in order instead.)
          for (size_t k = 0; k < std::min(fwd.size(), back.size()); k++) pairs.push_back({fwd[k], back[k]});
        }
      }
      for (auto [f, b] : pairs) {
        mate[f] = b, mate[b] = f;
        edgeOf[f] = edgeOf[b] = (uint32_t)out.edges.size();
        out.edges.push_back({u, v});
      }
    }
  }

  // Shells: triangles joined across their edges. Inside out (less than nothing within), a shell is a void, of the
  // smallest solid round it.
  Find shellOf(nt);
  for (uint32_t i = 0; i < ns; i++) shellOf.join(i / 3, mate[i] / 3);
  auto &shellAt = out.shellAt, &voidOf = out.voidOf;
  auto &vol = out.vol;
  std::vector<uint32_t> first;
  shellAt.assign(nt, -1);
  for (uint32_t t = 0; t < nt; t++) {
    uint32_t r = shellOf(t);
    if (shellAt[r] < 0) shellAt[r] = (int)first.size(), first.push_back(r), vol.push_back(0);
    shellAt[t] = shellAt[r];
    V3 a = P[T[3 * t]], b = P[T[3 * t + 1]], c = P[T[3 * t + 2]];
    vol[shellAt[t]] += dot(a, cross(b, c)) / 6;
  }
  size_t nsh = first.size();
  voidOf.assign(nsh, -1);
  for (size_t v = 0; v < nsh; v++) {
    if (!(vol[v] < 0)) continue;
    // A point within the void: the middle of its first triangle.
    V3 q{0, 0, 0};
    for (int k = 0; k < 3; k++) q = q + P[T[3 * first[v] + k]] / 3;
    double best = INFINITY;
    for (size_t o = 0; o < nsh; o++) {
      if (!(vol[o] > 0) || vol[o] >= best) continue;
      Solid one;
      one.p = P;
      for (uint32_t t = 0; t < nt; t++)
        if (shellAt[t] == (int)o) one.tri.insert(one.tri.end(), {T[3 * t], T[3 * t + 1], T[3 * t + 2]});
      if (inside(one, q)) best = vol[o], voidOf[v] = (int)o;
    }
    if (voidOf[v] < 0) return why = "inside out", false;
  }
  return true;
}

bool printMesh(const Shape &shape, PrintMesh &out, std::string &why) {
  out = PrintMesh();
  Solid s;
  mesh(shape, fileDeflection, s);
  Shells sh;
  if (!shells(s, sh, why)) {
    // What there is, as it is.
    for (V3 q : s.p) out.pos.insert(out.pos.end(), {(float)q.x, (float)q.y, (float)q.z});
    out.tri = s.tri;
    for (size_t t = 0; t + 2 < s.tri.size(); t += 3) out.volume += dot(s.p[s.tri[t]], cross(s.p[s.tri[t + 1]], s.p[s.tri[t + 2]])) / 6;
    return false;
  }
  size_t ns = sh.T.size();
  // A point once for each fan of triangles round it: corners joined where a side meets its mate (the corner at the
  // side's start and the one at its mate's end are the same point, on the same piece of surface).
  Find fan(ns);
  for (uint32_t i = 0; i < ns; i++) {
    uint32_t m = sh.mate[i];
    fan.join(i, m / 3 * 3 + (m % 3 + 1) % 3);
  }
  std::vector<uint32_t> at(ns, UINT32_MAX);
  std::vector<V3> Q;  // the points, a copy for each fan
  auto &tri = out.tri;
  tri.resize(ns);
  for (uint32_t c = 0; c < ns; c++) {
    uint32_t r = fan(c);
    if (at[r] == UINT32_MAX) at[r] = (uint32_t)Q.size(), Q.push_back(sh.P[sh.T[c]]);
    tri[c] = at[r];
  }

  // Slivers: triangles thinner than float can hold, their corners all but in a line. Each goes, and the triangle across
  // its longest side is split at its far corner instead, where both halves are no slivers: the surface stays closed,
  // moved by no more than the sliver is thin. (Again on what that frees up, a few times over; any left are flat, so have
  // no way round in float to lose.)
  double ulp = sh.size * std::ldexp(1.0, -23);  // float's spacing at the largest coordinate
  // Thinner than `k` spacings: its height (twice its area over its longest side) less than that.
  auto sliver = [&](uint32_t a, uint32_t b, uint32_t c, double k = 4) {
    double longest = std::max({norm(Q[b] - Q[a]), norm(Q[c] - Q[b]), norm(Q[a] - Q[c])});
    return !(norm(cross(Q[b] - Q[a], Q[c] - Q[a])) >= k * ulp * longest);
  };
  std::unordered_map<uint64_t, uint32_t> sideAt;
  auto sideKey = [](uint32_t a, uint32_t b) { return (uint64_t)a << 32 | b; };
  auto setSides = [&](uint32_t t, bool on) {
    for (int k = 0; k < 3; k++) {
      uint64_t key = sideKey(tri[3 * t + k], tri[3 * t + (k + 1) % 3]);
      if (on) sideAt[key] = 3 * t + k;
      else if (auto it = sideAt.find(key); it != sideAt.end() && it->second == 3 * t + (uint32_t)k) sideAt.erase(it);
    }
  };
  std::vector<char> gone(ns / 3, 0);
  for (uint32_t t = 0; t < ns / 3; t++) setSides(t, true);
  for (int pass = 0; pass < 8; pass++) {
    bool changed = false;
    for (uint32_t t = 0; t < tri.size() / 3; t++) {
      if (gone[t]) continue;
      // The longest side, from a to b, and c across it.
      int k = 0;
      double longest = 0;
      for (int j = 0; j < 3; j++) {
        double l = norm(Q[tri[3 * t + (j + 1) % 3]] - Q[tri[3 * t + j]]);
        if (l > longest) longest = l, k = j;
      }
      uint32_t a = tri[3 * t + k], b = tri[3 * t + (k + 1) % 3], c = tri[3 * t + (k + 2) % 3];
      if (!sliver(a, b, c)) continue;
      auto across = sideAt.find(sideKey(b, a));
      if (across == sideAt.end()) continue;
      uint32_t u = across->second / 3, d = tri[across->second / 3 * 3 + (across->second % 3 + 2) % 3];
      if (u == t || gone[u]) continue;
      if (d == c) {
        // (Two slivers back to back: both go, their outer sides meeting each other.)
        setSides(t, false), setSides(u, false);
        gone[t] = gone[u] = 1;
        changed = true;
        continue;
      }
      if (sideAt.count(sideKey(c, d)) || sideAt.count(sideKey(d, c)) || sliver(c, a, d) || sliver(c, d, b)) continue;
      setSides(t, false), setSides(u, false);
      gone[t] = 1;
      tri[3 * u] = c, tri[3 * u + 1] = a, tri[3 * u + 2] = d;
      tri.insert(tri.end(), {c, d, b});
      gone.push_back(0);
      setSides(u, true), setSides((uint32_t)gone.size() - 1, true);
      changed = true;
    }
    if (!changed) break;
  }
  {
    size_t n = 0;
    for (size_t t = 0; t < gone.size(); t++)
      if (!gone[t]) tri[3 * n] = tri[3 * t], tri[3 * n + 1] = tri[3 * t + 1], tri[3 * n + 2] = tri[3 * t + 2], n++;
    tri.resize(3 * n);
  }
  ns = tri.size();

  std::vector<V3> F(Q.size());  // the points as float holds them
  out.pos.reserve(Q.size() * 3);
  for (size_t i = 0; i < Q.size(); i++) {
    float x = (float)Q[i].x, y = (float)Q[i].y, z = (float)Q[i].z;
    out.pos.insert(out.pos.end(), {x, y, z});
    F[i] = {x, y, z};
  }
  bool thin = false;
  // Two points float makes one: an STL (which has no point numbers) would join what's apart.
  {
    std::unordered_map<Key, V3, Hash> seen;
    for (size_t i = 0; i < Q.size() && !thin; i++) {
      auto [it, fresh] = seen.emplace(key((float)F[i].x, (float)F[i].y, (float)F[i].z), Q[i]);
      thin = !fresh && !(it->second == Q[i]);
    }
  }
  // A triangle float flattens or turns over (other than the slivers left, flat to within float's spacing).
  for (size_t t = 0; t < ns && !thin; t += 3) {
    if (sliver(tri[t], tri[t + 1], tri[t + 2], 2)) {
      out.slivers++;
      continue;
    }
    V3 exact = cross(Q[tri[t + 1]] - Q[tri[t]], Q[tri[t + 2]] - Q[tri[t]]);
    thin = !(dot(cross(F[tri[t + 1]] - F[tri[t]], F[tri[t + 2]] - F[tri[t]]), exact) > 0);
  }
  for (size_t t = 0; t < ns; t += 3) out.volume += dot(F[tri[t]], cross(F[tri[t + 1]], F[tri[t + 2]])) / 6;
  // Edges still crowded: by point numbers, more than one side each way.
  {
    std::vector<uint64_t> sides(ns);
    for (uint32_t i = 0; i < ns; i++) sides[i] = (uint64_t)tri[i] << 32 | tri[i / 3 * 3 + (i % 3 + 1) % 3];
    std::sort(sides.begin(), sides.end());
    for (size_t i = 1; i < ns; i++)
      if (sides[i] == sides[i - 1] && (i < 2 || sides[i] != sides[i - 2])) out.crowded++;
  }
  if (thin) return why = "thin", false;
  return true;
}

}  // namespace bce
