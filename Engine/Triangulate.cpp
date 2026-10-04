// A triangulation in a plane keeping given segments as edges: points go in one at a time (found by walking from the last
// triangle), and a segment is made an edge by taking out the triangles it crosses and filling the two holes either side of
// it. Every decision is an exact orientation: of points held as doubles here, or asked of whoever holds them (exactly,
// for merging meshes: then nothing is ever nudged, led round or made up, and what can't be done is counted).
#include "Engine/Triangulate.hpp"

#include "Engine/Math.hpp"

#include <algorithm>

namespace bce {

Tri2::Tri2(double ax, double ay, double bx, double by, double cx, double cy) {
  x = {ax, bx, cx}, y = {ay, by, cy};
  points = 3;
  add(0, 1, 2);
}

Tri2::Tri2(Points &h) : held(&h) {
  points = 3;
  add(0, 1, 2);
}

int Tri2::orient(int a, int b, int c) const { return held ? held->orient(a, b, c) : orient2d(x[a], y[a], x[b], y[b], x[c], y[c]); }

bool Tri2::between(int a, int m, int b) const {
  if (held) return held->between(a, m, b);
  double reach = (x[b] - x[a]) * (x[b] - x[a]) + (y[b] - y[a]) * (y[b] - y[a]);
  double p = (x[m] - x[a]) * (x[b] - x[a]) + (y[m] - y[a]) * (y[b] - y[a]);
  return p > 0 && p < reach;
}

int Tri2::add(int a, int b, int c) {
  // A corner twice (a hole whose outline came back through a point, never expected): no triangle; the hole stays a hair
  // open rather than the triangulation holding one that isn't.
  if (a == b || b == c || c == a) return misses++, -1;
  int t = (int)tris.size();
  tris.push_back({{a, b, c}, true});
  half.set(key(a, b), t), half.set(key(b, c), t), half.set(key(c, a), t);
  last = t;
  return t;
}

void Tri2::remove(int t) {
  T &tr = tris[t];
  for (int k = 0; k < 3; k++) {
    uint64_t e = key(tr.v[k], tr.v[(k + 1) % 3]);
    if (half.find(e) == t) half.erase(e);
  }
  tr.alive = false;
}

int Tri2::locate(int p, int &where, int &which) {
  int cur = last;
  if (cur < 0 || cur >= (int)tris.size() || !tris[cur].alive) {
    cur = -1;
    for (int i = (int)tris.size() - 1; i >= 0 && cur < 0; i--)
      if (tris[i].alive) cur = i;
  }
  // A walk that picks its way out of each triangle starting from a different edge each time never goes round in circles.
  int limit = 4 * (int)tris.size() + 64;
  for (int step = 0; step < limit; step++) {
    const T &t = tris[cur];
    seed = seed * 1664525u + 1013904223u;
    int start = (int)(seed >> 16) % 3, next = -1, out = -1;
    for (int j = 0; j < 3 && next < 0; j++) {
      int k = (start + j) % 3;
      if (orient(t.v[k], t.v[(k + 1) % 3], p) < 0) {
        out = k;
        next = across(t.v[k], t.v[(k + 1) % 3]);
        if (next < 0) {
          where = -1, which = k;
          return cur;
        }
      }
    }
    if (out < 0) {
      int o[3], zeros = 0;
      for (int k = 0; k < 3; k++) zeros += (o[k] = orient(t.v[k], t.v[(k + 1) % 3], p)) == 0;
      if (zeros == 0) {
        where = 0;
      } else if (zeros == 1) {
        where = 1;
        for (int k = 0; k < 3; k++)
          if (o[k] == 0) which = k;
      } else {
        // On two edges: their shared corner.
        where = 2;
        for (int k = 0; k < 3; k++)
          if (o[k] == 0 && o[(k + 1) % 3] == 0) which = (k + 1) % 3;
      }
      return cur;
    }
    cur = next;
  }
  // Never expected; the plain way.
  for (int i = 0; i < (int)tris.size(); i++) {
    if (!tris[i].alive) continue;
    const T &t = tris[i];
    int o[3], zeros = 0, neg = 0;
    for (int k = 0; k < 3; k++) {
      o[k] = orient(t.v[k], t.v[(k + 1) % 3], p);
      zeros += o[k] == 0, neg += o[k] < 0;
    }
    if (neg) continue;
    where = zeros == 0 ? 0 : zeros == 1 ? 1 : 2;
    for (int k = 0; k < 3; k++) {
      if (zeros == 1 && o[k] == 0) which = k;
      if (zeros == 2 && o[k] == 0 && o[(k + 1) % 3] == 0) which = (k + 1) % 3;
    }
    return i;
  }
  where = -1, which = 0;
  return cur;
}

void Tri2::splitInside(int t, int p) {
  int a = tris[t].v[0], b = tris[t].v[1], c = tris[t].v[2];
  remove(t);
  add(a, b, p), add(b, c, p), add(c, a, p);
}

void Tri2::splitEdge(int t, int k, int p) {
  int a = tris[t].v[k], b = tris[t].v[(k + 1) % 3], c = tris[t].v[(k + 2) % 3];
  int u = across(a, b);
  int d = -1;
  if (u >= 0)
    for (int j = 0; j < 3; j++)
      if (tris[u].v[j] != a && tris[u].v[j] != b) d = tris[u].v[j];
  remove(t);
  add(a, p, c), add(p, b, c);
  if (u >= 0 && d >= 0) {
    remove(u);
    add(b, p, d), add(p, a, d);
  }
  // A kept edge split in two stays kept, for what it was kept for.
  int tag = std::max(fixed.find(key(a, b)), fixed.find(key(b, a)));
  if (tag > 0) fixed.erase(key(a, b)), fixed.erase(key(b, a)), fixed.set(key(a, p), tag), fixed.set(key(p, b), tag);
}

int Tri2::place(int p) {
  int where, which;
  int t = locate(p, where, which);
  if (where == 2) return tris[t].v[which];
  if (where == 1) {
    splitEdge(t, which, p);
  } else if (where == 0 || !held) {
    // Inside (or, in doubles, a hair outside the outermost edge by rounding: in the triangle it was found by).
    splitInside(t, p);
  } else {
    return misses++, -1;
  }
  return p;
}

int Tri2::insert(double px, double py) {
  int p = (int)x.size();
  x.push_back(px), y.push_back(py);
  int at = place(p);
  if (at != p) x.pop_back(), y.pop_back();
  else points = (int)x.size();
  return at;
}

int Tri2::insert(int p) {
  int at = place(p);
  if (at == p) points = std::max(points, p + 1);
  return at;
}

int Tri2::onEdge(int u, int v, int p) {
  int t = half.find(key(u, v));
  if (t < 0) {
    t = half.find(key(v, u));
    std::swap(u, v);
  }
  int k = 0;
  while (t >= 0 && k < 3 && tris[t].v[k] != u) k++;
  if (t < 0 || !tris[t].alive || k == 3) {
    // Not an edge (never expected): where it lies instead.
    fallbacks++;
    return place(p);
  }
  splitEdge(t, k, p);
  return p;
}

int Tri2::insertOnEdge(int u, int v, int p) {
  int at = onEdge(u, v, p);
  if (at == p) points = std::max(points, p + 1);
  return at;
}

// Ear by ear: a corner that turns left with no other corner on or inside the triangle it cuts off.
void Tri2::fill(std::vector<int> poly) {
  int guard = 0;
  while (poly.size() > 3 && guard++ < 100000) {
    size_t n = poly.size();
    bool cut = false;
    for (size_t i = 0; i < n && !cut; i++) {
      int a = poly[(i + n - 1) % n], b = poly[i], c = poly[(i + 1) % n];
      if (orient(a, b, c) <= 0) continue;
      bool clear = true;
      for (size_t j = 0; j < n && clear; j++) {
        int q = poly[j];
        if (q == a || q == b || q == c) continue;
        if (orient(a, b, q) >= 0 && orient(b, c, q) >= 0 && orient(c, a, q) >= 0) clear = false;
      }
      if (!clear) continue;
      add(a, b, c);
      poly.erase(poly.begin() + i);
      cut = true;
    }
    if (!cut) {
      // Only flat corners left (never expected): cut one off anyway so the hole is closed.
      misses++;
      add(poly[n - 1], poly[0], poly[1]);
      poly.erase(poly.begin());
    }
  }
  if (poly.size() == 3) add(poly[0], poly[1], poly[2]);
}

bool Tri2::keep(int a, int b, int tag) {
  // Piece by piece, as many as there are points on the way: up to the first point of the triangulation on the segment,
  // then on from there.
  for (;;) {
    if (a == b) return true;
    if (half.has(key(a, b)) || half.has(key(b, a))) {
      fixed.set(key(a, b), tag + 1);
      return true;
    }
    int on = -1;
    if (!keepFrom(a, b, tag, on)) return false;
    if (on < 0) return true;
    a = on;
  }
}

bool Tri2::keepFrom(int a, int b, int tag, int &on) {
  // The triangle round a that b's direction leaves through.
  int t0 = -1, R = -1, L = -1;
  for (int i = 0; i < (int)tris.size() && t0 < 0; i++) {
    const T &t = tris[i];
    if (!t.alive) continue;
    for (int k = 0; k < 3; k++) {
      if (t.v[k] != a) continue;
      int c = t.v[(k + 1) % 3], d = t.v[(k + 2) % 3];
      int oc = orient(a, c, b), od = orient(a, d, b);
      // A point of the triangulation on the segment (strictly between a and b: one beyond b, on the same line, is no
      // stop on the way): the side to it kept, the rest from there.
      bool onC = oc == 0 && between(a, c, b), onD = !onC && od == 0 && between(a, d, b);
      if (onC || onD) {
        on = onC ? c : d;
        fixed.set(key(a, on), tag + 1);
        return true;
      }
      if (oc > 0 && od < 0) t0 = i, R = c, L = d;
    }
  }
  if (t0 < 0) return misses += held != nullptr, false;
  std::vector<int> crossed{t0}, left{L}, right{R};
  int end = b;
  for (int guard = 0;; guard++) {
    // A walk crossing more triangles than there are has lost its way (points a hair apart): no edge kept.
    if (guard > (int)tris.size()) return misses += held != nullptr, false;
    if (kept(R, L)) {
      // Another kept edge crosses the segment: both go through the point where they cross, so neither is broken.
      if (depth > 64) return misses += held != nullptr, false;
      depth++;
      bool ok;
      if (held) {
        int p = held->cross(a, b, tag, R, L, tagOf(R, L));
        ok = p >= 0 && insertOnEdge(R, L, p) == p && keep(a, p, tag) && keep(p, b, tag);
        if (!ok) misses++;
      } else {
        // (In doubles: two cuts a hair apart, crossed by rounding.)
        double dx = x[b] - x[a], dy = y[b] - y[a], ex = x[L] - x[R], ey = y[L] - y[R], den = dx * ey - dy * ex;
        double s = den != 0 ? ((x[R] - x[a]) * dy - (y[R] - y[a]) * dx) / den : 0.5;
        if (!(s > 1e-9)) ok = keep(a, R, tag) && keep(R, b, tag);
        else if (!(s < 1 - 1e-9)) ok = keep(a, L, tag) && keep(L, b, tag);
        else {
          int p = (int)x.size();
          x.push_back(x[R] + s * ex), y.push_back(y[R] + s * ey);
          int at = onEdge(R, L, p);
          if (at != p) x.pop_back(), y.pop_back(), p = at;
          else madeAt[p] = {R, L, s};
          points = (int)x.size();
          ok = keep(a, p, tag) && keep(p, b, tag);
        }
      }
      depth--;
      return ok;
    }
    int t = across(R, L);
    if (t < 0) {
      // Out through the outline. Held, never expected; in doubles (bent a hair by its points' rounding), by the outline's
      // nearer corner instead.
      if (held || depth > 64) return misses += held != nullptr, false;
      double dx = x[b] - x[a], dy = y[b] - y[a], ex = x[L] - x[R], ey = y[L] - y[R], den = dx * ey - dy * ex;
      double s = den != 0 ? ((x[R] - x[a]) * dy - (y[R] - y[a]) * dx) / den : 0.5;
      int via = s < 0.5 ? R : L;
      if (via == a || via == b) return false;
      detours++;
      depth++;
      bool ok = keep(a, via, tag) && keep(via, b, tag);
      depth--;
      return ok;
    }
    crossed.push_back(t);
    int e = -1;
    for (int k = 0; k < 3; k++)
      if (tris[t].v[k] != R && tris[t].v[k] != L) e = tris[t].v[k];
    if (e == b) break;
    int o = orient(a, b, e);
    if (o == 0) {
      // A point on the segment: the part to it now, the rest after.
      end = e;
      break;
    }
    if (o > 0) L = e, left.push_back(e);
    else R = e, right.push_back(e);
  }
  for (int t : crossed) remove(t);
  std::vector<int> upper{a, end}, lower{end, a};
  for (auto it = left.rbegin(); it != left.rend(); ++it) upper.push_back(*it);
  for (int v : right) lower.push_back(v);
  fill(upper);
  fill(lower);
  fixed.set(key(a, end), tag + 1);
  if (end != b) on = end;
  return true;
}

std::vector<int> Tri2::triangles() const {
  std::vector<int> out;
  for (const auto &t : tris)
    if (t.alive) out.insert(out.end(), {t.v[0], t.v[1], t.v[2]});
  return out;
}

std::vector<int> Tri2::insideKept() const {
  std::vector<int> parity(tris.size(), -1), queue, out;
  for (int i = 0; i < (int)tris.size(); i++)
    if (tris[i].alive && (tris[i].v[0] < 3 || tris[i].v[1] < 3 || tris[i].v[2] < 3)) parity[i] = 0, queue.push_back(i);
  for (size_t q = 0; q < queue.size(); q++) {
    int t = queue[q];
    for (int k = 0; k < 3; k++) {
      int u = tris[t].v[k], v = tris[t].v[(k + 1) % 3];
      int nb = across(u, v);
      if (nb < 0 || parity[nb] >= 0) continue;
      parity[nb] = parity[t] ^ (kept(u, v) ? 1 : 0);
      queue.push_back(nb);
    }
  }
  for (int i = 0; i < (int)tris.size(); i++) {
    const T &t = tris[i];
    if (t.alive && parity[i] == 1 && t.v[0] >= 3 && t.v[1] >= 3 && t.v[2] >= 3) out.insert(out.end(), {t.v[0], t.v[1], t.v[2]});
  }
  return out;
}

}  // namespace bce
