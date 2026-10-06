// Bodies round an axis whose radius changes with angle and height (Radial.hpp), meshed from an (angle, height) sheet.
//
// Each zone's side is a sheet: angle t across (columns: by the chord error, and where sectors meet), height z up, cut
// along the thread's rows where it's threaded (straight lines on the sheet: a row rises p a turn). Every line on the sheet
// (a column, a row within a strip, the zone's bottom or top within a strip) has its points worked out once: where the
// other lines cross it, and where the surface it lies on changes from one atom to another (found along it to the last
// bit). A cell takes its sides' points from those lines, so cells either side of a line meet point for point and the
// mesh is closed by how it's made. Inside a cell the surfaces are parted along straight chords between the points where
// its sides change surface; whatever is chosen there can only make the mesh rougher, never open.
#include "Engine/Radial.hpp"

#include "Engine/Treat.hpp"
#include "Engine/Triangulate.hpp"

#include <algorithm>
#include <array>
#include <climits>
#include <cstring>
#include <map>
#include <memory>
#include <unordered_map>

namespace bce {

bool Atom::at(double t, double z, double &r, double &rt, double &rz) const {
  if (z < lo || z > hi) return false;
  switch (kind) {
  case Linear:
    r = a + bz * z, rt = 0, rz = bz;
    return true;
  case Wall: {
    double c = trig::cos(t - phi);
    if (c < 1e-9) return false;
    double s = trig::sin(t - phi), d = d0 + d1 * z;
    r = d / c, rt = d * s / (c * c), rz = d1 / c;
    return r > 0;
  }
  case Arc: {
    double ux = trig::cos(t), uy = trig::sin(t), uc = ux * cx + uy * cy, wc = ux * cy - uy * cx;
    double disc = uc * uc - (cx * cx + cy * cy) + rho * rho;
    if (disc < 0) return false;
    double s = std::sqrt(disc);
    r = uc + side * s;
    if (!(r > 0)) return false;
    rt = s > 0 ? wc * (1 + side * uc / s) : 0, rz = 0;
    return true;
  }
  }
  return false;
}

namespace {

constexpr double twoPi = 2 * M_PI;
// No chord turns by more than this round the axis, however coarse the mesh asked for (as for the other shapes).
constexpr double maxTurn = 0.35;
// Where a thread's rows lie within a pitch, from the crest's middle: crest ends, root starts, root ends, crest starts.
constexpr double rowAt[4] = {1.0 / 16, 6.0 / 16, 10.0 / 16, 15.0 / 16};
// The piece above each row: the flank down, the root, the flank up, the crest.
constexpr int pieceAbove[4] = {1, 2, 3, 0};

int floor4(int k) { return k >= 0 ? k / 4 : -((-k + 3) / 4); }

// Row k (4 a pitch) at angle t, and where it is at height z.
double rowZ(const Thread &th, int k, double t) {
  int n = floor4(k);
  return th.z0 + th.p * ((n + rowAt[k - 4 * n]) + t / twoPi);
}
double rowT(const Thread &th, int k, double z) {
  int n = floor4(k);
  return twoPi * ((z - th.z0) / th.p - (n + rowAt[k - 4 * n]));
}

struct Val {
  double r = 0, rt = 0, rz = 0;
  int cls = 0;   // which surface: -1 the thread, 0… the sector's atoms, 100… cuts, 200… adds
  int face = 0;
  int band = 0;  // the thread's row below (the thread only)
};

// The thread's piece above row k, at (t, z).
void bandAt(const Thread &th, int k, double t, double z, Val &v) {
  int n = floor4(k), m = k - 4 * n;
  double s = (th.rMaj - th.rMin) * 16 / 5;  // radius per pitch of height along a flank
  double u = (z - th.z0) / th.p - t / twoPi - (n + rowAt[m]);
  v.cls = -1, v.band = k, v.face = th.face[pieceAbove[m]];
  switch (pieceAbove[m]) {
  case 0: v.r = th.rMaj, v.rt = v.rz = 0; break;
  case 1: v.r = th.rMaj - u * s, v.rz = -s / th.p, v.rt = s / twoPi; break;
  case 2: v.r = th.rMin, v.rt = v.rz = 0; break;
  default: v.r = th.rMin + u * s, v.rz = s / th.p, v.rt = -s / twoPi; break;
  }
}

// The row below (t, z).
int bandOf(const Thread &th, double t, double z) {
  double u = (z - th.z0) / th.p - t / twoPi, n = std::floor(u), w = u - n;
  int m = w < rowAt[0] ? -1 : w < rowAt[1] ? 0 : w < rowAt[2] ? 1 : w < rowAt[3] ? 2 : 3;
  return 4 * (int)n + m;
}

int sectorAt(const Zone &zn, double t) {
  int s = (int)zn.sectors.size() - 1;
  for (int i = 0; i < (int)zn.sectors.size(); i++)
    if (zn.sectors[i].t0 <= t) s = i;
  return std::max(s, 0);
}

// The zone's surface at (t, z): its base (thread or the least of the sector's atoms), cut back, grown out.
bool valueAt(const Zone &zn, int sector, double t, double z, Val &v) {
  double r, rt, rz;
  bool any = false;
  if (zn.threaded) {
    bandAt(zn.thread, bandOf(zn.thread, t, z), t, z, v);
    any = true;
  } else if (!zn.sectors.empty()) {
    const auto &atoms = zn.sectors[sector].atoms;
    for (size_t i = 0; i < atoms.size(); i++)
      if (atoms[i].at(t, z, r, rt, rz) && (!any || r < v.r)) v = {r, rt, rz, (int)i, atoms[i].face, 0}, any = true;
  }
  if (!any) return false;
  for (size_t i = 0; i < zn.cuts.size(); i++)
    if (zn.cuts[i].at(t, z, r, rt, rz) && r < v.r) v = {r, rt, rz, 100 + (int)i, zn.cuts[i].face, 0};
  for (size_t i = 0; i < zn.adds.size(); i++)
    if (zn.adds[i].at(t, z, r, rt, rz) && r > v.r) v = {r, rt, rz, 200 + (int)i, zn.adds[i].face, 0};
  return true;
}

// One of the zone's surfaces itself at (t, z), even a hair past where it ends.
bool surfaceAt(const Zone &zn, int sector, int cls, int band, double t, double z, Val &v) {
  if (cls == -1) {
    bandAt(zn.thread, band, t, z, v);
    return true;
  }
  const Atom *a = cls >= 200 ? &zn.adds[cls - 200] : cls >= 100 ? &zn.cuts[cls - 100] : &zn.sectors[sector].atoms[cls];
  Atom free = *a;
  free.lo = -INFINITY, free.hi = INFINITY;
  double r, rt, rz;
  if (!free.at(t, z, r, rt, rz)) return false;
  v = {r, rt, rz, cls, a->face, band};
  return true;
}

V3 placeOf(double t, double r, double z) { return {r * trig::cos(t), r * trig::sin(t), z}; }

// The outward normal of a surface given as its radius (and slopes) at angle t: ∂/∂t × ∂/∂z of (r cos t, r sin t, z).
V3 normalOf(double t, const Val &v, bool inside) {
  double c = trig::cos(t), s = trig::sin(t);
  V3 n = unit(V3{v.r * c + v.rt * s, v.r * s - v.rt * c, -v.r * v.rz});
  return inside ? n * -1.0 : n;
}

// The mesh as it's made: vertices by (zone, face, point), so a point used by one face is one vertex, and what each
// face's chords miss of its surface.
struct Body {
  Solid s;
  std::vector<double> miss;
  std::unordered_map<uint64_t, uint32_t> made;
  uint32_t vertex(uint64_t key, V3 p, V3 n) {
    auto it = made.find(key);
    if (it != made.end()) return it->second;
    uint32_t v = s.vertex(p, n);
    made.emplace(key, v);
    return v;
  }
};

struct Ring {
  std::vector<V3> p;
  std::vector<double> t;
  double lo = INFINITY, hi = 0;  // its least and largest radius
};

// One zone's side, from its sheet.
class ZoneMesh {
 public:
  ZoneMesh(const RadialSpec &spec, const Zone &zn, bool inside, int tag, double d, Body &body)
      : spec(spec), zn(zn), inside(inside), tag(tag), d(d), body(body), th(zn.thread) {}

  bool run(std::string &why);
  Ring bottom, top;
  // The zone's own radius at (t, z) (for the walls between inside and out).
  bool radius(double t, double z, double &r) const {
    double tt = t - twoPi * std::floor(t / twoPi);
    Val v;
    if (!valueAt(zn, sectorAt(zn, tt), tt, z, v)) return false;
    r = v.r;
    return true;
  }
  // Every point it was meshed through, as (t, z, r).
  void points(std::vector<std::array<double, 3>> &out) const {
    for (size_t i = 0; i < pz.size(); i++) out.push_back({pt[i], pz[i], pr[i]});
  }

 private:
  struct P {
    double t, z;
    uint32_t id;
  };
  struct ColPt {
    double z;
    uint32_t id;
    int row;  // the row it's on, or INT_MIN
  };
  const RadialSpec &spec;
  const Zone &zn;
  bool inside;
  int tag;
  double d;
  Body &body;
  const Thread &th;
  bool bad = false;
  std::string trouble;
  // Heights this near the zone's bottom or top count as at it (and points this near along a line as one): no two points
  // of the sheet so near that placing the body could make them one.
  double tolZ = 0;
  static constexpr double near = 1e-9;
  // Where a height is: 0 below the zone, 1 at its bottom, 2 inside, 3 at its top, 4 above.
  int level(double z) const { return z < zn.zb - tolZ ? 0 : z <= zn.zb + tolZ ? 1 : z < zn.zt - tolZ ? 2 : z <= zn.zt + tolZ ? 3 : 4; }

  int N = 0;
  std::vector<double> col;  // N + 1 angles, the last 2π
  std::vector<int> sec;     // each strip's sector
  int kLo = 0, kHi = -1;    // the rows that come into the zone
  std::vector<std::vector<ColPt>> cols;           // per column (0 … N−1) by height
  std::vector<std::array<std::vector<P>, 2>> hor;  // per strip, along the bottom and the top: points between its ends
  std::map<std::pair<int, int>, std::vector<P>> rows;  // per strip and row: points between its ends
  std::vector<std::array<std::map<int, P>, 2>> crossing;  // per strip, bottom and top: where each row crosses
  // Points: angle (in [0, 2π)), height, radius, place.
  std::vector<double> pt, pz, pr;
  std::vector<V3> pos;
  std::map<std::pair<double, double>, uint32_t> ids;

  uint32_t point(double t, double z, int sector) {
    double key = t >= twoPi ? 0 : t;
    auto it = ids.find({key, z});
    if (it != ids.end()) return it->second;
    Val v;
    if (!valueAt(zn, sector, key, z, v)) fail("no surface at a point");
    uint32_t id = (uint32_t)pos.size();
    ids.emplace(std::make_pair(key, z), id);
    pt.push_back(key), pz.push_back(z), pr.push_back(v.r), pos.push_back(placeOf(key, v.r, z));
    return id;
  }
  void fail(const char *why) {
    if (!bad) bad = true, trouble = why;
  }
  int classAt(int sector, double t, double z) {
    Val v;
    if (!valueAt(zn, sector, t >= twoPi ? t - twoPi : t, z, v)) return INT_MIN;
    return v.cls;
  }

  // Where the surface changes along a line on the sheet (s from 0 to 1 by `at`), each to the last bit.
  template <typename F> void switches(int sector, F at, std::vector<std::array<double, 2>> &out) {
    const int n = 8;
    double sPrev = 0, last = 0;
    auto p0 = at(0.0);
    int cPrev = classAt(sector, p0[0], p0[1]);
    for (int i = 1; i <= n; i++) {
      double s = double(i) / n;
      auto ps = at(s);
      int c = classAt(sector, ps[0], ps[1]);
      double a = sPrev;
      int ca = cPrev;
      for (int guard = 0; ca != c && guard < 8; guard++) {
        double lo = a, hi = s;
        for (int it = 0; it < 64; it++) {
          double m = 0.5 * (lo + hi);
          if (!(m > lo && m < hi)) break;
          auto pm = at(m);
          if (classAt(sector, pm[0], pm[1]) == ca) lo = m;
          else hi = m;
        }
        if (hi > near && hi < 1 - near && hi > last + near) out.push_back(at(hi)), last = hi;
        a = hi;
        auto ph = at(hi);
        ca = classAt(sector, ph[0], ph[1]);
      }
      sPrev = s, cPrev = c;
    }
  }

  // Column j's height of row k (column N is column 0 a turn on: its rows four on).
  double colZ(int j, int k) const { return j == N ? rowZ(th, k + 4, col[0]) : rowZ(th, k, col[j]); }
  // The point on column j at the height of row k, held to the zone (its bottom or top where the row is past them).
  P colAt(int j, int k) {
    int jj = j == N ? 0 : j, kk = j == N ? k + 4 : k;
    double t = j == N ? twoPi : col[j], z = rowZ(th, kk, col[jj]);
    const auto &c = cols[jj];
    if (level(z) <= 1) return {t, c.front().z, c.front().id};
    if (level(z) >= 3) return {t, c.back().z, c.back().id};
    // (By height first, the column being in order of it: a long bolt's columns hold thousands of points.)
    auto at = std::lower_bound(c.begin(), c.end(), z, [](const ColPt &e, double v) { return e.z < v; });
    if (at != c.end() && at->row == kk) return {t, at->z, at->id};
    for (const auto &e : c)
      if (e.row == kk) return {t, e.z, e.id};
    fail("a row missing from a column");
    return {t, z, 0};
  }
  P colEnd(int j, bool atTop) {
    const auto &c = cols[j == N ? 0 : j];
    const ColPt &e = atTop ? c.back() : c.front();
    return {j == N ? twoPi : col[j], e.z, e.id};
  }
  P crossAt(int j, bool atTop, int k) {
    auto it = crossing[j][atTop].find(k);
    if (it == crossing[j][atTop].end()) {
      fail("a row's crossing missing");
      return {col[j], atTop ? zn.zt : zn.zb, 0};
    }
    return it->second;
  }

  void columns();
  void buildColumn(int j);
  void buildEdge(int j, bool atTop);
  void buildRow(int j, int k);
  void cellThreaded(int j, int k);
  void cellPlain(int j);
  void cell(int j, int band, std::vector<P> &B);
  void emit(const P &a, const P &b, const P &c, int cls, int band, int sector);
  uint32_t vertexOf(const P &p, bool shared, int cls, int band, int sector, int face);

  // Adds the points of a line strictly between two of its points: a column's by height, a strip's edge's or row's by
  // angle.
  void between(const std::vector<P> &line, double t0, double t1, std::vector<P> &out) {
    if (t0 < t1) {
      for (const P &p : line)
        if (p.t > t0 && p.t < t1) out.push_back(p);
    } else {
      for (auto it = line.rbegin(); it != line.rend(); ++it)
        if (it->t < t0 && it->t > t1) out.push_back(*it);
    }
  }
  // (The column in order of height: only the part between looked at.)
  void betweenCol(int j, double z0, double z1, std::vector<P> &out) {
    const auto &c = cols[j == N ? 0 : j];
    double t = j == N ? twoPi : col[j];
    auto above = [&](double v) { return std::upper_bound(c.begin(), c.end(), v, [](double x, const ColPt &e) { return x < e.z; }); };
    if (z0 < z1) {
      for (auto it = above(z0); it != c.end() && it->z < z1; ++it) out.push_back({t, it->z, it->id});
    } else {
      // (Those below z0, downward, while above z1.)
      for (auto it = std::make_reverse_iterator(std::lower_bound(c.begin(), c.end(), z0, [](const ColPt &e, double v) { return e.z < v; }));
           it != c.rend() && it->z > z1; ++it)
        out.push_back({t, it->z, it->id});
    }
  }
};

void ZoneMesh::columns() {
  double chord = zn.reach > 0 ? std::min(maxTurn, 2 * trig::acos(std::max(-1.0, 1 - d / zn.reach))) : maxTurn;
  int n = std::max(12, (int)std::ceil(twoPi / chord - 1e-9));
  double step = twoPi / n;
  std::vector<double> feat;
  for (const auto &s : zn.sectors) feat.push_back(s.t0);
  std::vector<double> all = feat;
  all.push_back(0);
  for (int i = 1; i < n; i++) {
    double u = step * i;
    bool near = false;
    for (double f : feat) {
      double g = std::fabs(u - f);
      g = std::min(g, twoPi - g);
      near = near || g < 0.25 * step;
    }
    if (!near) all.push_back(u);
  }
  std::sort(all.begin(), all.end());
  all.erase(std::unique(all.begin(), all.end()), all.end());
  col = all;
  N = (int)col.size();
  col.push_back(twoPi);
  sec.resize(N);
  for (int j = 0; j < N; j++) sec[j] = sectorAt(zn, 0.5 * (col[j] + col[j + 1]));
  if (zn.threaded) {
    // Every row that comes into the zone somewhere round the turn.
    int k = 4 * (int)std::floor((zn.zb - th.z0) / th.p) - 8;
    while (rowZ(th, k, twoPi) <= zn.zb) k++;
    kLo = k;
    while (rowZ(th, k + 1, 0) < zn.zt) k++;
    kHi = k;
  }
}

void ZoneMesh::buildColumn(int j) {
  double t = col[j];
  std::vector<ColPt> base;
  base.push_back({zn.zb, 0, INT_MIN});
  if (zn.threaded)
    for (int k = kLo; k <= kHi; k++) {
      double z = rowZ(th, k, t);
      if (level(z) == 2) base.push_back({z, 0, k});
    }
  base.push_back({zn.zt, 0, INT_MIN});
  std::vector<ColPt> &c = cols[j];
  for (size_t i = 0; i < base.size(); i++) {
    c.push_back(base[i]);
    if (i + 1 == base.size()) break;
    double z0 = base[i].z, z1 = base[i + 1].z;
    std::vector<std::array<double, 2>> sw;
    switches(sec[j], [&](double s) { return std::array<double, 2>{t, s >= 1 ? z1 : z0 + s * (z1 - z0)}; }, sw);
    for (auto &q : sw)
      if (q[1] > c.back().z + tolZ && q[1] < z1 - tolZ) c.push_back({q[1], 0, INT_MIN});
  }
  for (auto &e : c) e.id = point(t, e.z, sec[j]);
}

void ZoneMesh::buildEdge(int j, bool atTop) {
  double z = atTop ? zn.zt : zn.zb, ta = col[j], tb = col[j + 1];
  std::vector<std::pair<double, int>> cuts;
  if (zn.threaded)
    for (int k = kLo; k <= kHi; k++) {
      double a = colZ(j, k), b = colZ(j + 1, k);
      if (atTop ? !(level(a) <= 2 && level(b) == 4) : !(level(a) == 0 && level(b) >= 2)) continue;
      double t = rowT(th, k, z);
      t = std::min(std::max(t, std::nextafter(ta, INFINITY)), std::nextafter(tb, -INFINITY));
      cuts.push_back({t, k});
    }
  std::sort(cuts.begin(), cuts.end());
  std::vector<P> &out = hor[j][atTop];
  double from = ta;
  auto run = [&](double to) {
    std::vector<std::array<double, 2>> sw;
    switches(sec[j], [&](double s) { return std::array<double, 2>{s >= 1 ? to : from + s * (to - from), z}; }, sw);
    double gap = near * (tb - ta);
    for (auto &q : sw)
      if (q[0] > from + gap && q[0] < to - gap && (out.empty() || q[0] > out.back().t + gap)) out.push_back({q[0], z, point(q[0], z, sec[j])});
  };
  for (auto &c : cuts) {
    if (c.first <= from) continue;
    run(c.first);
    P p{c.first, z, point(c.first, z, sec[j])};
    out.push_back(p);
    crossing[j][atTop][c.second] = p;
    from = c.first;
  }
  run(tb);
}

void ZoneMesh::buildRow(int j, int k) {
  double L0 = colZ(j, k), L1 = colZ(j + 1, k);
  if (level(L1) <= 1 || level(L0) >= 3) return;
  P a = level(L0) >= 1 ? colAt(j, k) : crossAt(j, false, k);
  P b = level(L1) <= 3 ? colAt(j + 1, k) : crossAt(j, true, k);
  if (!(a.t < b.t)) return;
  std::vector<std::array<double, 2>> sw;
  switches(sec[j], [&](double s) {
    double t = s >= 1 ? b.t : a.t + s * (b.t - a.t);
    return std::array<double, 2>{t, rowZ(th, k, t)};
  }, sw);
  std::vector<P> &out = rows[{j, k}];
  double gap = near * (b.t - a.t);
  for (auto &q : sw)
    if (q[0] > a.t + gap && q[0] < b.t - gap && (out.empty() || q[0] > out.back().t + gap)) out.push_back({q[0], q[1], point(q[0], q[1], sec[j])});
}

// The cell between rows k and k + 1 in strip j, held to the zone.
void ZoneMesh::cellThreaded(int j, int k) {
  double L0 = colZ(j, k), L1 = colZ(j + 1, k), U0 = colZ(j, k + 1), U1 = colZ(j + 1, k + 1);
  int l0 = level(L0), l1 = level(L1), u0 = level(U0), u1 = level(U1);
  if (l0 >= 3 || u1 <= 1) return;
  static const std::vector<P> none;
  auto rowLine = [&](int r) -> const std::vector<P> & {
    auto it = rows.find({j, r});
    return it == rows.end() ? none : it->second;
  };
  std::vector<P> B;
  auto add = [&](const P &p) {
    if (B.empty() || B.back().id != p.id || B.back().t != p.t) B.push_back(p);
  };
  // Lower left, then along the bottom (the zone's bottom, then row k, as each is higher).
  P start = u0 >= 1 ? colAt(j, k) : crossAt(j, false, k + 1);
  add(start);
  P lowEnd = l1 <= 3 ? colAt(j + 1, k) : crossAt(j, true, k);
  if (l0 >= 1) {
    between(rowLine(k), start.t, lowEnd.t, B);
  } else if (l1 <= 1) {
    between(hor[j][0], start.t, lowEnd.t, B);
  } else {
    P x = crossAt(j, false, k);
    between(hor[j][0], start.t, x.t, B);
    add(x);
    between(rowLine(k), x.t, lowEnd.t, B);
  }
  add(lowEnd);
  // Up the right side, then back along the top (row k + 1, then the zone's top, as each is lower).
  P upRight = lowEnd;
  if (l1 <= 3) {
    upRight = colAt(j + 1, k + 1);
    betweenCol(j + 1, lowEnd.z, upRight.z, B);
    add(upRight);
  }
  P upLeft = u0 >= 1 ? colAt(j, k + 1) : start;
  if (u1 <= 3) {
    between(rowLine(k + 1), upRight.t, upLeft.t, B);
  } else if (u0 >= 3) {
    between(hor[j][1], upRight.t, upLeft.t, B);
  } else {
    P y = crossAt(j, true, k + 1);
    between(hor[j][1], upRight.t, y.t, B);
    add(y);
    between(rowLine(k + 1), y.t, upLeft.t, B);
  }
  if (u0 >= 1) {
    add(upLeft);
    betweenCol(j, upLeft.z, start.z, B);
  }
  while (B.size() > 1 && B.back().id == B.front().id && B.back().t == B.front().t) B.pop_back();
  cell(j, k, B);
}

void ZoneMesh::cellPlain(int j) {
  std::vector<P> B;
  P bl = colEnd(j, false), br = colEnd(j + 1, false), tr = colEnd(j + 1, true), tl = colEnd(j, true);
  B.push_back(bl);
  between(hor[j][0], bl.t, br.t, B);
  B.push_back(br);
  betweenCol(j + 1, br.z, tr.z, B);
  B.push_back(tr);
  between(hor[j][1], tr.t, tl.t, B);
  B.push_back(tl);
  betweenCol(j, tl.z, bl.z, B);
  cell(j, 0, B);
}

uint32_t ZoneMesh::vertexOf(const P &p, bool shared, int cls, int band, int sector, int face) {
  double t = p.t >= twoPi ? 0 : p.t;
  Val v;
  if (!surfaceAt(zn, sector, cls, band, t, p.z, v) && !valueAt(zn, sector, t, p.z, v)) fail("no surface at a corner");
  V3 n = normalOf(t, v, inside);
  if (shared) return body.vertex((uint64_t)tag << 48 | (uint64_t)face << 32 | p.id, pos[p.id], n);
  Val at;
  if (!valueAt(zn, sector, t, p.z, at)) at = v;
  return body.s.vertex(placeOf(t, at.r, p.z), n);
}

// A triangle of the sheet (counter-clockwise there), its face from `cls`; what its chords miss of the surface, by how
// far the surface lies beyond each side's middle.
void ZoneMesh::emit(const P &a, const P &b, const P &c, int cls, int band, int sector) {
  int face = cls == -1 ? th.face[pieceAbove[band - 4 * floor4(band)]]
             : cls >= 200 ? zn.adds[cls - 200].face
             : cls >= 100 ? zn.cuts[cls - 100].face
                          : zn.sectors[sector].atoms[cls].face;
  const P *q[3] = {&a, inside ? &c : &b, inside ? &b : &c};
  uint32_t v[3];
  for (int i = 0; i < 3; i++) v[i] = vertexOf(*q[i], q[i]->id != UINT32_MAX, cls, band, sector, face);
  body.s.triangle(v[0], v[1], v[2], face);
  if (spec.faces[face].kind == FaceGeom::Flat) return;
  V3 x[3] = {body.s.p[v[0]], body.s.p[v[1]], body.s.p[v[2]]};
  V3 nn = cross(x[1] - x[0], x[2] - x[0]);
  double area2 = norm(nn);
  if (!(area2 > 0)) return;
  nn = nn / area2;
  double g = 0;
  for (int i = 0; i < 3; i++) {
    const P &u = *q[i], &w = *q[(i + 1) % 3];
    double tm = 0.5 * (u.t + w.t), zm = 0.5 * (u.z + w.z), tk = tm >= twoPi ? tm - twoPi : tm;
    Val s;
    if (!surfaceAt(zn, sector, cls, band, tk, zm, s) && !valueAt(zn, sector, tk, zm, s)) continue;
    g += dot(placeOf(tk, s.r, zm) - (x[i] + x[(i + 1) % 3]) * 0.5, nn);
  }
  body.miss[face] += area2 / 6 * g;
}

// A cell's outline on the sheet (counter-clockwise), parted by surface and made triangles.
void ZoneMesh::cell(int j, int band, std::vector<P> &B) {
  int m = (int)B.size();
  if (m < 3) return;
  int sector = sec[j];
  std::vector<int> label(m);
  for (int i = 0; i < m; i++) {
    const P &a = B[i], &b = B[(i + 1) % m];
    label[i] = classAt(sector, 0.5 * (a.t + b.t), 0.5 * (a.z + b.z));
  }
  auto fan = [&](const std::vector<int> &poly, int cls) {
    double ct = 0, cz = 0;
    for (int i : poly) ct += B[i].t, cz += B[i].z;
    P c{ct / poly.size(), cz / poly.size(), UINT32_MAX};
    for (size_t i = 0; i < poly.size(); i++) emit(c, B[poly[i]], B[poly[(i + 1) % poly.size()]], cls, band, sector);
  };
  auto straight = [&](int a, int b, int c) { return orient2d(B[a].t, B[a].z, B[b].t, B[b].z, B[c].t, B[c].z) == 0; };
  auto fill = [&](const std::vector<int> &poly, int cls) {
    size_t n = poly.size();
    if (n == 3) {
      emit(B[poly[0]], B[poly[1]], B[poly[2]], cls, band, sector);
    } else if (n == 4 && !straight(poly[3], poly[0], poly[1]) && !straight(poly[0], poly[1], poly[2]) && !straight(poly[1], poly[2], poly[3]) &&
               !straight(poly[2], poly[3], poly[0])) {
      // Split along the shorter diagonal in place.
      double d02 = norm2(pos[B[poly[0]].id] - pos[B[poly[2]].id]), d13 = norm2(pos[B[poly[1]].id] - pos[B[poly[3]].id]);
      if (d02 <= d13) {
        emit(B[poly[0]], B[poly[1]], B[poly[2]], cls, band, sector);
        emit(B[poly[0]], B[poly[2]], B[poly[3]], cls, band, sector);
      } else {
        emit(B[poly[1]], B[poly[2]], B[poly[3]], cls, band, sector);
        emit(B[poly[1]], B[poly[3]], B[poly[0]], cls, band, sector);
      }
    } else {
      fan(poly, cls);
    }
  };
  // Where the surface changes round the outline, and chords pairing those changes (the same two surfaces either end)
  // without crossing.
  std::vector<int> sw;
  for (int i = 0; i < m; i++)
    if (label[i] != label[(i + m - 1) % m]) sw.push_back(i);
  std::vector<int> all(m);
  for (int i = 0; i < m; i++) all[i] = i;
  if (sw.empty()) {
    fill(all, label[0]);
    return;
  }
  std::vector<int> chord(m, -1);
  std::vector<int> stack;
  auto pairOf = [&](int i) {
    int a = label[(i + m - 1) % m], b = label[i];
    return std::make_pair(std::min(a, b), std::max(a, b));
  };
  for (int i : sw) {
    if (!stack.empty() && pairOf(stack.back()) == pairOf(i)) chord[i] = stack.back(), chord[stack.back()] = i, stack.pop_back();
    else stack.push_back(i);
  }
  bool ok = stack.empty();
  std::vector<std::pair<std::vector<int>, int>> parts;
  if (ok) {
    std::vector<char> used(m, 0);
    for (int s = 0; s < m && ok; s++) {
      if (used[s]) continue;
      std::vector<int> poly;
      int cls = label[s];
      for (int i = s, guard = 0; guard <= 2 * m; guard++) {
        if (label[i] != cls) ok = false;
        used[i] = 1, poly.push_back(i);
        int nx = (i + 1) % m;
        if (nx == s) break;
        if (chord[nx] >= 0) {
          poly.push_back(nx);
          if (chord[nx] == s) break;
          i = chord[nx];
        } else {
          i = nx;
        }
      }
      parts.push_back({poly, cls});
    }
  }
  if (!ok) {
    // Three surfaces meeting inside, or two crossing twice: a fan from the middle, each side its own surface. Rougher
    // here, closed all the same.
    double ct = 0, cz = 0;
    for (const P &p : B) ct += p.t, cz += p.z;
    P c{ct / m, cz / m, UINT32_MAX};
    for (int i = 0; i < m; i++) emit(c, B[i], B[(i + 1) % m], label[i], band, sector);
    return;
  }
  for (auto &part : parts) fill(part.first, part.second);
}

bool ZoneMesh::run(std::string &why) {
  tolZ = near * (1 + std::max(std::fabs(zn.zb), std::fabs(zn.zt)));
  columns();
  cols.assign(N, {});
  hor.assign(N, {});
  crossing.assign(N, {});
  for (int j = 0; j < N; j++) buildColumn(j);
  for (int j = 0; j < N; j++) buildEdge(j, false), buildEdge(j, true);
  if (zn.threaded)
    for (int j = 0; j < N; j++)
      for (int k = kLo; k <= kHi; k++) buildRow(j, k);
  if (bad) return why = trouble, false;
  for (int j = 0; j < N; j++) {
    if (zn.threaded)
      for (int k = kLo - 1; k <= kHi; k++) cellThreaded(j, k);
    else
      cellPlain(j);
  }
  // The rings round the bottom and the top, by angle.
  for (int atTop = 0; atTop < 2; atTop++) {
    Ring &r = atTop ? top : bottom;
    for (int j = 0; j < N; j++) {
      P e = colEnd(j, atTop);
      std::vector<P> run{e};
      for (const P &p : hor[j][atTop]) run.push_back(p);
      for (const P &p : run) {
        r.p.push_back(pos[p.id]), r.t.push_back(p.t);
        r.lo = std::min(r.lo, pr[p.id]), r.hi = std::max(r.hi, pr[p.id]);
      }
    }
  }
  if (bad) return why = trouble, false;
  return true;
}

// MARK: - ends and steps

// A flat face of a ring's inside: a fan from the axis. Up: seen from above counter-clockwise.
bool disc(Body &b, const Ring &r, double z, bool up, int face) {
  size_t n = r.p.size();
  V3 nz{0, 0, up ? 1.0 : -1.0};
  uint32_t c = b.s.vertex({0, 0, z}, nz);
  std::vector<uint32_t> v(n);
  for (size_t i = 0; i < n; i++) v[i] = b.s.vertex(r.p[i], nz);
  for (size_t i = 0; i < n; i++) {
    size_t k = (i + 1) % n;
    if (orient2d(0, 0, r.p[i].x, r.p[i].y, r.p[k].x, r.p[k].y) <= 0) return false;
  }
  for (size_t i = 0; i < n; i++) {
    size_t k = (i + 1) % n;
    if (up) b.s.triangle(c, v[i], v[k], face);
    else b.s.triangle(c, v[k], v[i], face);
  }
  return true;
}

// A flat face between two rings, the inner one strictly inside: zipped round by angle, or failing that by a
// triangulation keeping both rings' sides; either way checked to the last bit before it's taken.
bool annulus(Body &b, const Ring &out, const Ring &in, bool up, int face) {
  size_t no = out.p.size(), ni = in.p.size();
  std::vector<V3> pts = out.p;
  pts.insert(pts.end(), in.p.begin(), in.p.end());
  auto orient = [&](int a, int c, int e) { return orient2d(pts[a].x, pts[a].y, pts[c].x, pts[c].y, pts[e].x, pts[e].y); };
  // A set of triangles (counter-clockwise from above) is right when each turns the right way and every ring side is in
  // exactly one of them, the right way round (outer counter-clockwise, inner clockwise), every other side in two.
  auto right = [&](const std::vector<int> &tri) {
    std::map<std::pair<int, int>, int> sides;
    for (size_t k = 0; k < tri.size(); k += 3) {
      if (orient(tri[k], tri[k + 1], tri[k + 2]) <= 0) return false;
      for (int e = 0; e < 3; e++) sides[{tri[k + e], tri[k + (e + 1) % 3]}]++;
    }
    for (size_t i = 0; i < no; i++)
      if (sides[{(int)i, (int)((i + 1) % no)}] != 1 || sides.count({(int)((i + 1) % no), (int)i})) return false;
    for (size_t i = 0; i < ni; i++) {
      int a = (int)(no + i), c = (int)(no + (i + 1) % ni);
      if (sides[{c, a}] != 1 || sides.count({a, c})) return false;
    }
    for (auto &[k, n] : sides) {
      bool ring = (k.first < (int)no && k.second < (int)no && (k.second == (k.first + 1) % (int)no || k.first == (k.second + 1) % (int)no)) ||
                  (k.first >= (int)no && k.second >= (int)no &&
                   (k.second - (int)no == (k.first - (int)no + 1) % (int)ni || k.first - (int)no == (k.second - (int)no + 1) % (int)ni));
      if (ring) continue;
      if (n != 1 || sides.count({k.second, k.first}) != 1) return false;
    }
    return true;
  };
  std::vector<int> tri;
  {
    size_t i = 0, o = 0;
    auto angle = [](const Ring &r, size_t k) { return k < r.t.size() ? r.t[k] : twoPi; };
    while (i < ni || o < no) {
      double ti = i < ni ? angle(in, i + 1) : INFINITY, to = o < no ? angle(out, o + 1) : INFINITY;
      if (o < no && to <= ti) tri.insert(tri.end(), {(int)(no + i % ni), (int)(o % no), (int)((o + 1) % no)}), o++;
      else tri.insert(tri.end(), {(int)(no + i % ni), (int)(o % no), (int)(no + (i + 1) % ni)}), i++;
    }
  }
  if (!right(tri)) {
    tri.clear();
    double R = std::max(out.hi, in.hi) * 4 + 1;
    Tri2 t(-2 * R, -2 * R, 2 * R, -2 * R, 0, 2 * R);
    std::vector<int> id(pts.size());
    std::unordered_map<int, int> back;
    for (size_t k = 0; k < pts.size(); k++) id[k] = t.insert(pts[k].x, pts[k].y), back[id[k]] = (int)k;
    bool kept = true;
    for (size_t k = 0; k < no; k++) kept = kept && t.keep(id[k], id[(k + 1) % no]);
    for (size_t k = 0; k < ni; k++) kept = kept && t.keep(id[no + k], id[no + (k + 1) % ni]);
    if (!kept || !t.made().empty() || (int)back.size() != (int)pts.size()) return false;
    for (int q : t.insideKept()) {
      auto it = back.find(q);
      if (it == back.end()) return false;
      tri.push_back(it->second);
    }
    if (!right(tri)) return false;
  }
  V3 nz{0, 0, up ? 1.0 : -1.0};
  std::vector<uint32_t> v(pts.size());
  for (size_t k = 0; k < pts.size(); k++) v[k] = b.s.vertex(pts[k], nz);
  for (size_t k = 0; k < tri.size(); k += 3) {
    if (up) b.s.triangle(v[tri[k]], v[tri[k + 1]], v[tri[k + 2]], face);
    else b.s.triangle(v[tri[k]], v[tri[k + 2]], v[tri[k + 1]], face);
  }
  return true;
}

// MARK: - exact sizes

// The heights along angle t where the zone's surface may bend: its ends, the thread's rows, atoms' ends, and where any
// two of its surfaces cross.
void bendsAt(const Zone &zn, int sector, double t, std::vector<double> &zs) {
  zs = {zn.zb, zn.zt};
  if (zn.threaded) {
    const Thread &th = zn.thread;
    for (int k = 4 * (int)std::floor((zn.zb - th.z0) / th.p) - 8;; k++) {
      double z = rowZ(th, k, t);
      if (z >= zn.zt) break;
      if (z > zn.zb) zs.push_back(z);
    }
  }
  auto ends = [&](const Atom &a) {
    if (a.lo > zn.zb && a.lo < zn.zt) zs.push_back(a.lo);
    if (a.hi > zn.zb && a.hi < zn.zt) zs.push_back(a.hi);
  };
  if (!zn.threaded && !zn.sectors.empty())
    for (const auto &a : zn.sectors[sector].atoms) ends(a);
  for (const auto &a : zn.cuts) ends(a);
  for (const auto &a : zn.adds) ends(a);
  std::sort(zs.begin(), zs.end());
  zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
  // Within each stretch every surface is straight in z: where two cross.
  static thread_local std::vector<double> more;
  static thread_local std::vector<std::pair<double, double>> lines;  // value at the middle, slope
  more.clear();
  for (size_t i = 0; i + 1 < zs.size(); i++) {
    double z0 = zs[i], z1 = zs[i + 1], zm = 0.5 * (z0 + z1);
    lines.clear();
    double r, rt, rz;
    if (zn.threaded) {
      Val v;
      bandAt(zn.thread, bandOf(zn.thread, t, zm), t, zm, v);
      lines.push_back({v.r, v.rz});
    } else if (!zn.sectors.empty()) {
      for (const auto &a : zn.sectors[sector].atoms)
        if (a.at(t, zm, r, rt, rz)) lines.push_back({r, rz});
    }
    for (const auto &a : zn.cuts)
      if (a.at(t, zm, r, rt, rz)) lines.push_back({r, rz});
    for (const auto &a : zn.adds)
      if (a.at(t, zm, r, rt, rz)) lines.push_back({r, rz});
    for (size_t a = 0; a < lines.size(); a++)
      for (size_t c = a + 1; c < lines.size(); c++) {
        double ds = lines[a].second - lines[c].second;
        if (ds == 0) continue;
        double z = zm + (lines[c].first - lines[a].first) / ds;
        if (z > z0 && z < z1) more.push_back(z);
      }
  }
  zs.insert(zs.end(), more.begin(), more.end());
  std::sort(zs.begin(), zs.end());
  zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
}

// ∫ r²/2 dz over the zone at angle t: exact (straight in z between bends).
double columnIntegral(const Zone &zn, int sector, double t) {
  static thread_local std::vector<double> zs;
  bendsAt(zn, sector, t, zs);
  double sum = 0;
  for (size_t i = 0; i + 1 < zs.size(); i++) {
    double a = zs[i], b = zs[i + 1], m = 0.5 * (a + b);
    Val v;
    if (!valueAt(zn, sector, t, m, v)) continue;
    double r0 = v.r + v.rz * (a - m), r1 = v.r + v.rz * (b - m);
    sum += (b - a) * (r0 * r0 + r0 * r1 + r1 * r1) / 6;
  }
  return sum;
}

double zoneVolume(const Zone &zn) {
  static const double node[4] = {-0.8611363115940526, -0.3399810435848563, 0.3399810435848563, 0.8611363115940526};
  static const double weight[4] = {0.3478548451374538, 0.6521451548625461, 0.6521451548625461, 0.3478548451374538};
  std::vector<double> cut{0, twoPi};
  if (zn.threaded) {
    for (int i = 1; i < 256; i++) cut.push_back(twoPi * i / 256);
    // Where a row meets the zone's ends the column integral bends: pieces end there.
    const Thread &th = zn.thread;
    for (double z : {zn.zb, zn.zt})
      for (int m = 0; m < 4; m++) {
        double t = twoPi * ((z - th.z0) / th.p - rowAt[m]);
        t -= twoPi * std::floor(t / twoPi);
        cut.push_back(t);
      }
  } else {
    for (size_t s = 0; s < zn.sectors.size(); s++) {
      double a = zn.sectors[s].t0, b = s + 1 < zn.sectors.size() ? zn.sectors[s + 1].t0 : zn.sectors[0].t0 + twoPi;
      for (int i = 0; i <= 24; i++) {
        double t = a + (b - a) * i / 24;
        cut.push_back(t >= twoPi ? t - twoPi : t);
      }
    }
  }
  std::sort(cut.begin(), cut.end());
  cut.erase(std::unique(cut.begin(), cut.end()), cut.end());
  double sum = 0;
  for (size_t i = 0; i + 1 < cut.size(); i++) {
    double a = cut[i], b = cut[i + 1], h = 0.5 * (b - a), c = 0.5 * (a + b);
    if (!(h > 0)) continue;
    int sector = zn.sectors.empty() ? 0 : sectorAt(zn, c);
    for (int k = 0; k < 4; k++) sum += h * weight[k] * columnIntegral(zn, sector, c + h * node[k]);
  }
  return sum;
}

}  // namespace

double radialVolume(const RadialSpec &s) {
  double v = 0;
  for (const auto &z : s.outer) v += zoneVolume(z);
  for (const auto &z : s.inner) v -= zoneVolume(z);
  return v;
}

double radialWall(const RadialSpec &spec) {
  double w = INFINITY;
  for (const auto &in : spec.inner) {
    std::vector<double> ts;
    for (int i = 0; i < 720; i++) ts.push_back(twoPi * i / 720);
    for (const auto &sc : in.sectors) ts.push_back(sc.t0);
    for (const auto &o : spec.outer)
      for (const auto &sc : o.sectors) ts.push_back(sc.t0);
    for (double t : ts) {
      int si = in.sectors.empty() ? 0 : sectorAt(in, t);
      std::vector<double> zi, zo;
      bendsAt(in, si, t, zi);
      for (const auto &o : spec.outer) {
        if (o.zt < in.zb || o.zb > in.zt) continue;
        int so = o.sectors.empty() ? 0 : sectorAt(o, t);
        bendsAt(o, so, t, zo);
        zo.insert(zo.end(), zi.begin(), zi.end());
        for (double z : zo) {
          if (z < std::max(in.zb, o.zb) || z > std::min(in.zt, o.zt)) continue;
          Val a, b;
          if (valueAt(o, so, t, z, a) && valueAt(in, si, t, z, b)) w = std::min(w, a.r - b.r);
        }
      }
    }
  }
  return w;
}

double radialSupport(const RadialSpec &s, V3 d, V3 *at, bool *exact) {
  double h = trig::hypot(d.x, d.y), dz = d.z, len = norm(d);
  double td = h > 0 ? trig::atan2(d.y, d.x) : 0;
  if (td < 0) td += twoPi;
  bool across = std::fabs(dz) <= 1e-12 * len, along = h <= 1e-12 * len;
  std::vector<double> ts{td};
  if (!along) {
    for (const auto &zn : s.outer)
      for (const auto &sc : zn.sectors) ts.push_back(sc.t0);
    if (!across)
      for (int i = 0; i < 720; i++) ts.push_back(twoPi * i / 720);
  }
  double best = -INFINITY, reach = 0, pitch = 0;
  V3 where{0, 0, 0};
  for (const auto &zn : s.outer) {
    reach = std::max(reach, zn.reach);
    if (zn.threaded) pitch = std::max(pitch, zn.thread.p);
  }
  for (double t : ts) {
    double c = trig::cos(t - td);
    if (!along && c <= 0) continue;
    for (const auto &zn : s.outer) {
      int sector = zn.sectors.empty() ? 0 : sectorAt(zn, t);
      std::vector<double> zs;
      bendsAt(zn, sector, t, zs);
      for (double z : zs) {
        Val v;
        if (!valueAt(zn, sector, t, z, v)) continue;
        // Along the axis: a point of the end there (its rim's middle, clear of any socket or bore).
        double r = along ? 0.5 * v.r : v.r, g = r * h * c + z * dz;
        if (along) {
          for (const auto &in : s.inner)
            if (z >= in.zb && z <= in.zt) {
              Val w;
              if (valueAt(in, in.sectors.empty() ? 0 : sectorAt(in, t), t, z, w)) r = 0.5 * (v.r + w.r);
            }
          g = z * dz;
        }
        if (g > best) best = g, where = placeOf(t, r, z);
      }
    }
  }
  bool sure = across || along;
  if (!sure) best += (reach * h + std::fabs(dz) * pitch / twoPi) * (twoPi / 720);
  if (at) *at = where;
  if (exact) *exact = sure;
  return best;
}

bool radialMesh(const RadialSpec &spec, double deflection, Solid &out, std::string &why) {
  double d = std::isfinite(deflection) ? std::max(deflection, 1e-4) : 0.05;
  // A thin wall between inside and outside: chords fine enough not to cross it.
  if (!(spec.wall > 1e-4)) return why = "no wall left between the inside and the outside", false;
  d = std::min(d, spec.wall / 4);
  Body body;
  for (const auto &g : spec.faces) {
    Solid::Face f;
    f.geom = g;
    if (g.kind == FaceGeom::Flat) f.normal = g.pn;
    body.s.faces.push_back(f);
  }
  body.miss.assign(spec.faces.size(), 0);
  std::vector<std::unique_ptr<ZoneMesh>> outer, inner;
  for (size_t i = 0; i < spec.outer.size(); i++) {
    outer.emplace_back(new ZoneMesh(spec, spec.outer[i], false, (int)i, d, body));
    if (!outer.back()->run(why)) return false;
  }
  for (size_t i = 0; i < spec.inner.size(); i++) {
    inner.emplace_back(new ZoneMesh(spec, spec.inner[i], true, 128 + (int)i, d, body));
    if (!inner.back()->run(why)) return false;
  }
  // The inside keeps clear of the outside everywhere both are (by more than either's chords can stray).
  {
    auto zoneOf = [](const std::vector<Zone> &zs, double z) -> int {
      for (size_t i = 0; i < zs.size(); i++)
        if (z >= zs[i].zb && z <= zs[i].zt) return (int)i;
      return -1;
    };
    for (size_t i = 0; i < inner.size(); i++) {
      std::vector<std::array<double, 3>> pts;
      inner[i]->points(pts);
      for (auto &q : pts) {
        int o = zoneOf(spec.outer, q[1]);
        double r;
        if (o < 0 || !outer[o]->radius(q[0], q[1], r) || !(r - q[2] > 2 * d)) return why = "the inside comes through the outside", false;
      }
    }
    for (size_t o = 0; o < outer.size(); o++) {
      std::vector<std::array<double, 3>> pts;
      outer[o]->points(pts);
      for (auto &q : pts) {
        int i = zoneOf(spec.inner, q[1]);
        double r;
        if (i >= 0 && (!inner[i]->radius(q[0], q[1], r) || !(q[2] - r > 2 * d))) return why = "the inside comes through the outside", false;
      }
    }
  }
  // Flat ends and steps, where zones meet.
  std::map<double, std::array<int, 4>> level;  // outer below, outer above, inner below, inner above
  auto mark = [&](double z, int slot, int i) {
    auto it = level.find(z);
    if (it == level.end()) it = level.emplace(z, std::array<int, 4>{-1, -1, -1, -1}).first;
    it->second[slot] = i;
  };
  for (size_t i = 0; i < spec.outer.size(); i++) mark(spec.outer[i].zb, 1, (int)i), mark(spec.outer[i].zt, 0, (int)i);
  for (size_t i = 0; i < spec.inner.size(); i++) mark(spec.inner[i].zb, 3, (int)i), mark(spec.inner[i].zt, 2, (int)i);
  auto flatFace = [&](double z, bool up) {
    Solid::Face f;
    f.geom.kind = FaceGeom::Flat, f.geom.flat = true, f.geom.pn = {0, 0, up ? 1.0 : -1.0}, f.geom.pd = up ? z : -z;
    f.normal = f.geom.pn;
    body.s.faces.push_back(f);
    body.miss.push_back(0);
    return (int)body.s.faces.size() - 1;
  };
  for (auto &[z, at] : level) {
    int ob = at[0], oa = at[1], ib = at[2], ia = at[3];
    bool ok = true;
    if (ob >= 0 && oa >= 0) {
      if (ib >= 0 || ia >= 0) return why = "a step where the inside ends", false;
      const Ring &up = outer[oa]->bottom, &down = outer[ob]->top;
      if (up.lo > down.hi) ok = annulus(body, up, down, false, flatFace(z, false));
      else if (down.lo > up.hi) ok = annulus(body, down, up, true, flatFace(z, true));
      else return why = "a step whose sides cross", false;
    } else if (oa >= 0) {
      if (ib >= 0) return why = "the inside reaches below", false;
      const Ring &o = outer[oa]->bottom;
      if (ia >= 0) {
        const Ring &i = inner[ia]->bottom;
        if (!(o.lo > i.hi)) return why = "the inside comes through an end", false;
        ok = annulus(body, o, i, false, flatFace(z, false));
      } else {
        ok = disc(body, o, z, false, flatFace(z, false));
      }
    } else if (ob >= 0) {
      if (ia >= 0) return why = "the inside reaches above", false;
      const Ring &o = outer[ob]->top;
      if (ib >= 0) {
        const Ring &i = inner[ib]->top;
        if (!(o.lo > i.hi)) return why = "the inside comes through an end", false;
        ok = annulus(body, o, i, true, flatFace(z, true));
      } else {
        ok = disc(body, o, z, true, flatFace(z, true));
      }
    } else if (ia >= 0 && ib < 0) {
      ok = disc(body, inner[ia]->bottom, z, true, flatFace(z, true));  // a socket's floor
    } else if (ib >= 0 && ia < 0) {
      ok = disc(body, inner[ib]->top, z, false, flatFace(z, false));
    } else if (ia >= 0 && ib >= 0) {
      const Ring &up = inner[ia]->bottom, &down = inner[ib]->top;
      if (up.lo > down.hi) ok = annulus(body, up, down, true, flatFace(z, true));
      else if (down.lo > up.hi) ok = annulus(body, down, up, false, flatFace(z, false));
      else return why = "a step whose sides cross", false;
    }
    if (!ok) return why = "an end that can't be filled", false;
  }
  // What the chords miss: no more than the mesh's area at its chord error (a mesh off by more is wrong, not coarse); as
  // estimated face by face, scaled to make up the exact whole (spread by area where the estimates don't say).
  double meshed = body.s.meshVolume(), missed = 0, area = 0;
  std::vector<double> faceArea(body.s.faces.size(), 0);
  for (size_t t = 0; t < body.s.triFace.size(); t++) {
    double a = norm(cross(body.s.p[body.s.tri[3 * t + 1]] - body.s.p[body.s.tri[3 * t]], body.s.p[body.s.tri[3 * t + 2]] - body.s.p[body.s.tri[3 * t]])) / 2;
    area += a;
    if (spec.faces.size() > body.s.triFace[t] && spec.faces[body.s.triFace[t]].kind != FaceGeom::Flat) faceArea[body.s.triFace[t]] += a;
  }
  for (double m : body.miss) missed += m;
  double gap = spec.volume - meshed, curved = 0;
  for (double a : faceArea) curved += a;
  if (!(std::fabs(gap) <= 4 * area * d + 1e-9 * spec.volume)) return why = "the mesh is off its exact volume", false;
  double k = std::fabs(missed) > 1e-12 * spec.volume ? gap / missed : 0;
  for (size_t f = 0; f < body.s.faces.size(); f++)
    body.s.faces[f].deficit = k > 0.5 && k < 2 ? body.miss[f] * k : curved > 0 ? gap * faceArea[f] / curved : 0;
  out = std::move(body.s);
  // Two points of the sheet a hair apart may land on one point of the body: a triangle with two corners there has no
  // area, and its other two sides run both ways along one line, so leaving it out keeps the mesh as closed as it was.
  {
    std::vector<uint32_t> tri;
    std::vector<uint32_t> face;
    for (size_t t = 0; t < out.triFace.size(); t++) {
      V3 a = out.p[out.tri[3 * t]], b = out.p[out.tri[3 * t + 1]], c = out.p[out.tri[3 * t + 2]];
      if ((a.x == b.x && a.y == b.y && a.z == b.z) || (b.x == c.x && b.y == c.y && b.z == c.z) || (a.x == c.x && a.y == c.y && a.z == c.z)) continue;
      tri.insert(tri.end(), {out.tri[3 * t], out.tri[3 * t + 1], out.tri[3 * t + 2]});
      face.push_back(out.triFace[t]);
    }
    out.tri.swap(tri), out.triFace.swap(face);
  }
  finish(out, d);
  if (!shut(out)) return why = "the mesh didn't close", false;
  return true;
}

}  // namespace bce
