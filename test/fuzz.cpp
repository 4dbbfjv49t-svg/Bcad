// Bcad's engine shaken: random and extreme numbers through its C API. Nothing may crash, hang or do anything undefined
// (run it under the sanitizers); what's made must be sound. Sculpting: random strokes of every brush, undone and done
// again, the changed lists the app draws from always covering every change, everything undone giving back the start
// and done again the end. Figures: every number at the ends of its range and at random, made closed (and whole) with
// their box as worked out. Everything else: sizes, placements and tools from zero to huge, not numbers at all: refused,
// or made sound. BCAD_FUZZ_ITERS scales the random parts (default 1); BCAD_FUZZ_SEED changes them.
// c++ -std=c++17 -O1 -g -fsanitize=address,undefined,float-cast-overflow -I. test/fuzz.cpp Engine/*.cpp -o fuzz && ./fuzz
#include "BcadKernel.h"
#include "Engine/Math.hpp"
#include "Engine/Sculpt.hpp"

#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <tuple>
#include <unistd.h>
#include <vector>

using bce::V3;

static int failures = 0;
static std::string current;  // what's being done, for the watchdog
static void check(const char *name, bool ok, const std::string &note = "") {
  printf("%s %s %s\n", ok ? "✓" : "✗", name, note.c_str());
  fflush(stdout);
  if (!ok) failures++;
}
static void fail(const std::string &what) {
  static int shown = 0;
  if (shown++ < 40) printf("  ✗ %s\n", what.c_str());
  failures++;
}
// A case taking longer than this has hung.
static void watchdog(int) {
  static const char msg[] = "✗ hung: ";
  (void)!write(1, msg, sizeof msg - 1);
  (void)!write(1, current.c_str(), current.size());
  (void)!write(1, "\n", 1);
  _exit(3);
}
static const bool verbose = getenv("BCAD_FUZZ_VERBOSE");
static void doing(const std::string &what, unsigned seconds = 120) {
  current = what;
  if (verbose) printf("  %s\n", what.c_str()), fflush(stdout);
  alarm(seconds);
}

static uint64_t seed = 0x9e3779b97f4a7c15ull;
static uint64_t rnd() {
  seed ^= seed << 13, seed ^= seed >> 7, seed ^= seed << 17;
  return seed;
}
static double uni() { return (double)(rnd() >> 11) / 9007199254740992.0; }
template <class T, size_t N> static T pick(const T (&a)[N]) { return a[rnd() % N]; }
static std::string fmt(const char *f, double a, double b = 0, double c = 0) {
  char s[300];
  snprintf(s, sizeof s, f, a, b, c);
  return s;
}

static const double I[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};

// A mesh's closedness: every edge between two triangles run opposite ways (welded by position); `touching`: an edge may
// have four, two each way (a solid touching itself along a line).
static bool closed(const BKMesh *m, std::string &why, bool touching = true) {
  std::map<std::tuple<float, float, float>, int> weld;
  std::vector<int> id(m->vertexCount);
  for (int i = 0; i < m->vertexCount; i++) {
    auto key = std::make_tuple(m->positions[3 * i], m->positions[3 * i + 1], m->positions[3 * i + 2]);
    auto it = weld.find(key);
    id[i] = it == weld.end() ? (weld[key] = (int)weld.size()) : it->second;
  }
  std::map<std::pair<int, int>, int> directed;
  for (int t = 0; t < m->triangleCount; t++) {
    int v[3];
    for (int k = 0; k < 3; k++) v[k] = id[m->indices[3 * t + k]];
    if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) return why = "a triangle with two corners in one place", false;
    for (int k = 0; k < 3; k++) directed[{v[k], v[(k + 1) % 3]}]++;
  }
  for (auto &[e, n] : directed) {
    auto back = directed.find({e.second, e.first});
    if (back == directed.end()) return why = "an open edge", false;
    if (n != 1 && !(touching && back->second == n)) return why = "an edge run the same way twice", false;
  }
  return true;
}

// A made shape must be sound: its mesh valid and closed, its box finite.
static bool sound(const BKShape *s, std::string &why, double deflection = 0.05) {
  BKMesh *m = bk_mesh(s, deflection);
  bool ok = m && m->valid && m->triangleCount > 0 && closed(m, why);
  if (m && !m->valid) why = std::string("not valid: ") + bk_last_error();
  if (m && m->triangleCount == 0) why = "no triangles";
  for (int a = 0; ok && a < 6; a++) ok = std::isfinite(m->bbox[a]);
  bk_mesh_free(m);
  return ok;
}

// MARK: sculpting
// The app's floats (kept up to date only from the changed lists) against the sculptor's own state, slot by slot.
struct Twin {
  BKSculpt *api = nullptr;
  bce::Sculptor *ref = nullptr;
  std::string differs() const {
    const auto &p = ref->points(), &n = ref->normals();
    const auto &t = ref->triangles();
    if (bk_sculpt_vertex_count(api) != (int)p.size()) return fmt("%.0f points, the sculptor has %.0f", bk_sculpt_vertex_count(api), p.size());
    if (bk_sculpt_triangle_count(api) != (int)t.size() / 3) return fmt("%.0f triangle slots, the sculptor has %.0f", bk_sculpt_triangle_count(api), t.size() / 3);
    const float *ap = bk_sculpt_positions(api), *an = bk_sculpt_normals(api);
    for (size_t i = 0; i < p.size(); i++) {
      float q[6] = {(float)p[i].x, (float)p[i].y, (float)p[i].z, (float)n[i].x, (float)n[i].y, (float)n[i].z};
      if (memcmp(q, ap + 3 * i, 12) || memcmp(q + 3, an + 3 * i, 12)) return fmt("point %.0f not as the sculptor has it (left out of the changed list)", i);
    }
    const uint32_t *ai = bk_sculpt_indices(api);
    for (size_t k = 0; k < t.size() / 3; k++)
      for (int c = 0; c < 3; c++)
        if (ai[3 * k + c] != (ref->triangleAlive((uint32_t)k) ? t[3 * k + c] : 0)) return fmt("triangle slot %.0f not as the sculptor has it", k);
    return "";
  }
};

static std::vector<float> compactOf(const bce::Sculptor &s) {
  std::vector<V3> pts;
  std::vector<uint32_t> tris;
  s.compact(pts, tris);
  std::vector<float> out;
  for (V3 q : pts) out.insert(out.end(), {(float)q.x, (float)q.y, (float)q.z});
  for (uint32_t i : tris) out.push_back((float)i);
  return out;
}

static void sculptRun(int run, const BKShape *shape, double detail) {
  doing(fmt("sculpt run %.0f: remesh at %.2f", run, detail));
  BKSculptMesh *r = bk_remesh(shape, I, detail);
  if (!r) return fail(fmt("sculpt run %.0f: remesh refused: ", run) + bk_last_error());
  std::vector<V3> pts(r->vertexCount);
  for (int i = 0; i < r->vertexCount; i++) pts[i] = {r->positions[3 * i], r->positions[3 * i + 1], r->positions[3 * i + 2]};
  std::vector<uint32_t> tris(r->indices, r->indices + 3 * r->triangleCount);
  Twin w;
  w.api = bk_sculpt_new(r->positions, r->vertexCount, r->indices, r->triangleCount);
  w.ref = new bce::Sculptor(pts, tris);
  bk_sculpt_mesh_free(r);
  if (!w.api || !w.ref->ok()) return fail(fmt("sculpt run %.0f: not taken", run));
  std::vector<float> start = compactOf(*w.ref);

  auto surface = [&]() {
    double u[3] = {uni() * 2 - 1, uni() * 2 - 1, uni() * 2 - 1}, len = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (len < 1e-3) u[2] = len = 1;
    double o[3] = {100 * u[0] / len, 100 * u[1] / len, 100 * u[2] / len}, d[3] = {-u[0], -u[1], -u[2]}, at[3], nrm[3];
    if (bk_sculpt_ray(w.api, o, d, at, nrm)) return V3{at[0], at[1], at[2]};
    return V3{(uni() - 0.5) * 40, (uni() - 0.5) * 40, (uni() - 0.5) * 40};
  };
  static const double radii[] = {1e-9, 1e-3, 0.1, 0.5, 1, 2, 3, 5, 8, 20, 1e3, NAN, -1};
  static const double strengths[] = {0, 0.1, 0.5, 1, 2, -1, NAN};
  static const double pressures[] = {0, 0.02, 0.5, 1, 3, -2, NAN};
  static const double sizes[] = {0, 0.05, 0.5, 1, 4, -1, NAN};
  // (Not finer than the app allows: much finer, the strokes' history outgrows what's kept, and the oldest are forgotten.)
  static const double details[] = {0, 0, 0, 0.3, 0.6, 1, 2, -1, NAN, INFINITY};
  int strokes = 0;
  std::string where;
  for (int op = 0; op < 60; op++) {
    int what = (int)(rnd() % 20);
    if (what < 14) {
      int brush = (int)(rnd() % (BK_BRUSH_COUNT + 2)) - 1;  // -1 and BK_BRUSH_COUNT: not a brush (taken as draw)
      V3 at = surface();
      double radius = pick(radii), strength = pick(strengths);
      int mirror = (int)(rnd() % 10) - 1, invert = (int)(rnd() % 2);  // (-1, 8: bits beyond x, y and z)
      // Tips plain, ordinary, at their ends and past them, and not numbers.
      static const double hards[] = {0, 0, 0.3, 1, -1, 2, NAN}, ovals[] = {1, 1, 0.5, 0.05, 0, -1, 3, NAN}, angles[] = {0, 30, 90, 1e9, NAN},
                          tilts[] = {0, 0, 30, -85, 120, NAN};
      BKBrush b{};
      b.brush = brush, b.radius = radius, b.strength = strength, b.mirror = mirror, b.invert = invert;
      b.hardness = pick(hards), b.rigidity = pick(hards), b.oval = pick(ovals), b.angle = pick(angles), b.tilt = pick(tilts);
      for (double &v : b.across) v = rnd() % 4 ? uni() - 0.5 : pick(tilts);
      doing(fmt("sculpt run %.0f op %.0f: brush %.0f", run, op, brush) + fmt(" radius %g strength %g", radius, strength) +
            fmt(" mirror %.0f tip %g %g", mirror, b.hardness, b.rigidity) + fmt(" %g %g %g", b.oval, b.angle, b.tilt));
      double a[3] = {at.x, at.y, at.z};
      bool old = rnd() % 4 == 0;  // (the first call, still taken)
      if (old) bk_sculpt_begin(w.api, brush, a, radius, strength, mirror, invert);
      else bk_sculpt_begin_brush(w.api, &b, a);
      if (std::isfinite(radius) && std::isfinite(strength)) {
        if (old) w.ref->begin(brush, at, radius, strength, mirror != 0 ? BK_MIRROR_X : 0, invert != 0);
        else {
          bce::BrushTip tip;
          tip.hardness = b.hardness, tip.rigidity = b.rigidity, tip.oval = b.oval, tip.angle = b.angle, tip.tilt = b.tilt;
          w.ref->begin(brush, at, radius, strength, mirror, invert != 0, tip, V3{b.across[0], b.across[1], b.across[2]});
        }
      }
      V3 q = at;
      int dabs = (int)(rnd() % 8);
      for (int k = 0; k < dabs; k++) {
        double step = std::isfinite(radius) && radius > 0 && radius < 100 ? radius : 2;
        q = rnd() % 3 ? surface() : q + V3{(uni() - 0.5) * step, (uni() - 0.5) * step, (uni() - 0.5) * step};
        if (rnd() % 10 == 0) q = V3{q.x * 3, q.y * 3, q.z * 3};  // off the body
        double pressure = pick(pressures), size = pick(sizes), c[3] = {q.x, q.y, q.z}, tilt = rnd() % 3 ? NAN : pick(tilts);
        bk_sculpt_dab_tilted(w.api, c, pressure, size, tilt);
        if (std::isfinite(pressure) && std::isfinite(size)) w.ref->dab(q, pressure, size, tilt);
      }
      bk_sculpt_end(w.api), w.ref->end();
      strokes++;
    } else if (what < 17) {
      doing(fmt("sculpt run %.0f op %.0f: undo", run, op));
      int a = bk_sculpt_undo(w.api), b = w.ref->undo();
      if (a != b) fail(fmt("sculpt run %.0f op %.0f: undo told apart", run, op));
    } else if (what < 19) {
      doing(fmt("sculpt run %.0f op %.0f: redo", run, op));
      int a = bk_sculpt_redo(w.api), b = w.ref->redo();
      if (a != b) fail(fmt("sculpt run %.0f op %.0f: redo told apart", run, op));
    } else {
      double d = pick(details);
      bk_sculpt_set_detail(w.api, d), w.ref->setDetail(std::isfinite(d) ? d : 0);
    }
    bk_sculpt_sync(w.api);
    w.ref->takeChanged(), w.ref->takeChangedTriangles();
    if (verbose) printf("    %.0f triangle slots\n", (double)bk_sculpt_triangle_count(w.api)), fflush(stdout);
    std::string bad = bk_sculpt_check(w.api);
    if (bad.empty()) bad = w.ref->check();
    if (bad.empty()) bad = w.differs();
    if (!bad.empty()) {
      where = fmt("sculpt run %.0f op %.0f: ", run, op) + bad;
      break;
    }
  }
  if (where.empty()) {
    doing(fmt("sculpt run %.0f: undo everything", run));
    // (Strokes left undone done again first: everything undone and done again ends there.)
    while (w.ref->redo()) bk_sculpt_redo(w.api);
    bk_sculpt_sync(w.api);
    w.ref->takeChanged(), w.ref->takeChangedTriangles();
    std::vector<float> end = compactOf(*w.ref);
    int undone = 0;
    while (w.ref->undo()) bk_sculpt_undo(w.api), undone++;
    bk_sculpt_sync(w.api);
    w.ref->takeChanged(), w.ref->takeChangedTriangles();
    if (compactOf(*w.ref) != start) where = fmt("sculpt run %.0f: %.0f strokes undone don't give back the start", run, undone);
    else if (!w.differs().empty()) where = fmt("sculpt run %.0f, all undone: ", run) + w.differs();
    while (where.empty() && w.ref->redo()) bk_sculpt_redo(w.api);
    bk_sculpt_sync(w.api);
    w.ref->takeChanged(), w.ref->takeChangedTriangles();
    if (where.empty() && compactOf(*w.ref) != end) where = fmt("sculpt run %.0f: done again doesn't give back the end", run);
    else if (where.empty() && !w.differs().empty()) where = fmt("sculpt run %.0f, all done again: ", run) + w.differs();
    if (where.empty() && !std::string(bk_sculpt_check(w.api)).empty()) where = fmt("sculpt run %.0f at the end: ", run) + bk_sculpt_check(w.api);
  }
  if (!where.empty()) fail(where);
  bk_sculpt_free(w.api);
  delete w.ref;
}

// MARK: figures
static bool figureSound(const std::vector<double> &p, int draft, std::string &why) {
  BKShape *s = bk_figure(p.data(), (int)p.size(), draft);
  if (!s) return why = std::string("refused: ") + bk_last_error(), false;
  double ext[6], bb[6];
  bool ok = sound(s, why);
  if (ok && !(bk_figure_extent(p.data(), (int)p.size(), ext) && bk_bounds(s, I, bb) == 1)) ok = false, why = "no box";
  double off = 0;
  for (int a = 0; ok && a < 3; a++) off = std::max(off, std::fabs(ext[a] - (bb[3 + a] - bb[a])));
  if (ok && !(off <= 1e-9 * p[BK_FIG_HEIGHT])) ok = false, why = fmt("box off by %g", off);
  if (ok && !draft && bk_piece_count(s) != 1) ok = false, why = fmt("%.0f pieces", bk_piece_count(s));
  bk_free(s);
  return ok;
}

// Everything else the app calls, with numbers it would and wouldn't give: picks taken from a shape's own mesh (faces,
// edges, corners, and some off it), roundings, bevels and coves of them, hollows opened and walled, edges listed,
// distances from edges and faces, meshes given as bodies (broken ones too), STEP and print files, figures given fewer
// numbers or none, a sculpt's changed lists, its plain dabs and its mesh. Refused saying why, or made sound.
static int apiMade = 0, apiRefused = 0;
static void judged(BKShape *s, const std::string &what) {
  if (!s) {
    apiRefused++;
    if (!*bk_last_error()) fail(what + ": refused without saying why");
    return;
  }
  std::string why;
  if (sound(s, why)) apiMade++;
  else fail(what + ": made but not sound: " + why);
  bk_free(s);
}
static void apiRun(int k) {
  static const int kinds[] = {BK_BOX, BK_CYLINDER, BK_CONE, BK_SPHERE, BK_PRISM, BK_TORUS, BK_WEDGE, BK_PYRAMID, BK_HEMISPHERE, BK_BOWL, BK_RING,
                              BK_GLASS, BK_OVAL};
  int kind = pick(kinds);
  double p[5];
  for (double &v : p) v = 2 + uni() * 40;
  if (kind == BK_PRISM || kind == BK_PYRAMID) p[0] = 3 + (double)(rnd() % 10);
  if (kind == BK_TORUS) p[0] = 0, p[1] = 30 + uni() * 20, p[2] = 4 + uni() * 8;
  if (kind == BK_OVAL) p[2] = 5 + uni() * 170;
  std::string shape = fmt("api %.0f: kind %.0f", k, kind) + fmt(" %.17g %.17g %.17g", p[0], p[1], p[2]);
  if (verbose) printf("  %s\n", shape.c_str()), fflush(stdout);
  BKShape *s = bk_primitive(kind, p);
  if (!s) return;
  BKMesh *m = bk_mesh(s, 0.05);
  if (!m || m->faceCount == 0) {
    bk_mesh_free(m), bk_free(s);
    return;
  }
  // Picks: a face, an edge, a corner of its own (or a point beside it, now and then, or numbers that aren't).
  auto facePick = [&](double *out) {
    int f = (int)(rnd() % m->faceCount);
    for (int i = 0; i < 6; i++) out[i] = m->faceInfo[6 * f + i];
  };
  auto edgePick = [&](double *out) {
    if (m->edgeCount == 0) return facePick(out);
    int e = (int)(rnd() % m->edgeCount), a = (int)m->edgeStart[e], b = (int)m->edgeStart[e + 1];
    if (b <= a) return facePick(out);  // (a pole, where a turned face closes, has no points)
    int i = a + std::max(0, (b - a) / 2 - 1), j = std::min(b - 1, i + 1);
    for (int c = 0; c < 3; c++) out[c] = (m->edgePoints[3 * i + c] + m->edgePoints[3 * j + c]) / 2, out[3 + c] = m->edgePoints[3 * j + c] - m->edgePoints[3 * i + c];
  };
  auto cornerPick = [&](double *out) {
    if (m->cornerCount == 0) return facePick(out);
    int c = (int)(rnd() % m->cornerCount), f = (int)(rnd() % m->faceCount);
    for (int i = 0; i < 3; i++) out[i] = m->faceInfo[6 * f + i], out[3 + i] = m->corners[3 * c + i];
  };
  int n = 1 + (int)(rnd() % 4);
  std::vector<int> ks(n);
  std::vector<double> picks(6 * n);
  for (int i = 0; i < n; i++) {
    ks[i] = (int)(rnd() % 4);
    double *q = &picks[6 * i];
    if (ks[i] == BK_PICK_EDGE) edgePick(q);
    else if (ks[i] == BK_PICK_CORNER) cornerPick(q);
    else facePick(q);
    if (rnd() % 8 == 0) q[rnd() % 6] += (uni() - 0.5) * 20;
    if (rnd() % 40 == 0) q[rnd() % 6] = NAN;
  }
  static const double radii[] = {0.2, 0.5, 1, 2, 5, 1e-4, 50};
  double r = pick(radii), most = 0;
  int missing = 0;
  doing(shape + fmt(": fillet %g on %.0f picks", r, n), 120);
  judged(bk_fillet(s, ks.data(), picks.data(), n, r, &most, &missing), shape + fmt(": fillet %g", r));
  doing(shape + fmt(": chamfer %g", r), 120);
  judged(bk_chamfer(s, ks.data(), picks.data(), n, r, rnd() % 2 ? r : pick(radii), rnd() % 3 ? 0 : pick(radii), &missing), shape + fmt(": chamfer %g", r));
  doing(shape + fmt(": cove %g", r), 120);
  judged(bk_cove(s, ks.data(), picks.data(), n, r, &most, &missing), shape + fmt(": cove %g", r));
  // The edges they stand for, counted and listed (into too small a list too).
  doing(shape + ": pick edges", 60);
  int count = bk_pick_edges(s, ks.data(), picks.data(), n, nullptr, 0);
  std::vector<double> edges(6 * (size_t)std::max(1, count));
  int listed = bk_pick_edges(s, ks.data(), picks.data(), n, edges.data(), std::max(0, count / 2));
  if (count < 0 || listed != count) fail(shape + fmt(": pick edges counted %.0f, listed %.0f", count, listed));
  // Hollowed with an opening and a wall of its own.
  double open[6], wall[6], thick[1] = {pick(radii)};
  facePick(open), facePick(wall);
  int opens = (int)(rnd() % 2), walls = (int)(rnd() % 2);
  double t = pick(radii);
  doing(shape + fmt(": hollow %g, %.0f open", t, opens) + fmt(", %.0f walled", walls), 120);
  judged(bk_hollow(s, nullptr, 0, open, opens, wall, thick, walls, t, &missing), shape + fmt(": hollow %g", t));
  // Distances: from an edge and a face to a point and to another shape, by index (some past the end).
  doing(shape + ": distances", 60);
  double pt[3] = {(uni() - 0.5) * 100, (uni() - 0.5) * 100, (uni() - 0.5) * 100}, out[6], moved[12] = {1, 0, 0, 60, 0, 1, 0, 0, 0, 0, 1, 0};
  int ei = (int)(rnd() % (m->edgeCount + 2)) - 1, fi = (int)(rnd() % (m->faceCount + 2)) - 1;
  double d1 = bk_distance(s, I, BK_END_EDGE, ei, nullptr, nullptr, nullptr, BK_END_POINT, 0, pt, out);
  double d2 = bk_distance(s, I, BK_END_FACE, fi, nullptr, s, moved, BK_END_EDGE, ei, nullptr, out);
  if ((d1 != -1 && !(d1 >= 0)) || (d2 != -1 && !(d2 >= 0))) fail(shape + fmt(": a distance not a size %g, %g", d1, d2));
  // Merges with nothing; copies and print meshes; a remesh placed any way, its mesh a body, that body's own mesh back.
  if (bk_boolean((int)(rnd() % 3), s, nullptr) || bk_boolean(BK_UNION, nullptr, s)) fail(shape + ": merged with nothing");
  BKShape *c = bk_copy(s);
  BKPrintMesh *pm = bk_print_mesh(c);
  if (!pm || !pm->valid) fail(shape + ": its print mesh " + bk_last_error());
  bk_print_mesh_free(pm), bk_free(c);
  double placed[12];
  for (int i = 0; i < 12; i++) placed[i] = I[i];
  if (rnd() % 2) {
    double a = uni() * 6.28, sc = 0.5 + uni();
    placed[0] = std::cos(a) * sc, placed[1] = -std::sin(a) * sc, placed[4] = std::sin(a) * sc, placed[5] = std::cos(a) * sc, placed[10] = rnd() % 4 ? sc : -sc;
    placed[3] = (uni() - 0.5) * 100;
  }
  doing(shape + ": remesh placed", 120);
  BKSculptMesh *rm = bk_remesh(s, placed, 1 + uni() * 2);
  if (rm) {
    BKShape *body = bk_mesh_shape(rm->positions, rm->vertexCount, rm->indices, rm->triangleCount);
    if (!body) fail(shape + ": its remesh not taken as a body: " + bk_last_error());
    BKSculptMesh *own = body ? bk_mesh_body(body, placed) : nullptr;
    if (body && !own) fail(shape + ": a mesh body's own mesh not given back: " + bk_last_error());
    // Broken copies of it: a triangle gone (open), turned round (two the same way), a corner out of range, a point not a
    // number, all turned (inside out): each refused saying why.
    if (rm->triangleCount > 4) {
      std::vector<uint32_t> idx(rm->indices, rm->indices + 3 * rm->triangleCount);
      std::vector<float> pos(rm->positions, rm->positions + 3 * rm->vertexCount);
      int how = (int)(rnd() % 5);
      int tris = rm->triangleCount;
      if (how == 0) tris--;
      else if (how == 1) std::swap(idx[1], idx[2]);
      else if (how == 2) idx[rnd() % idx.size()] = (uint32_t)rm->vertexCount + 7;
      else if (how == 3) pos[rnd() % pos.size()] = NAN;
      else
        for (size_t q = 0; q + 2 < idx.size(); q += 3) std::swap(idx[q + 1], idx[q + 2]);
      BKShape *broken = bk_mesh_shape(pos.data(), rm->vertexCount, idx.data(), tris);
      if (broken) fail(shape + fmt(": a broken mesh (%.0f) taken", how));
      else if (!*bk_last_error()) fail(shape + ": a broken mesh refused without saying why");
      bk_free(broken);
    }
    bk_sculpt_mesh_free(own), bk_free(body);
    // Sculpted: plain dabs, the changed lists within the mesh's slots, the live count, its mesh read back.
    BKSculpt *sc = bk_sculpt_new(rm->positions, rm->vertexCount, rm->indices, rm->triangleCount);
    if (sc) {
      double at[3] = {rm->positions[0], rm->positions[1], rm->positions[2]};
      bk_sculpt_set_detail(sc, rnd() % 2 ? 0 : 1.5);
      bk_sculpt_begin(sc, (int)(rnd() % BK_BRUSH_COUNT), at, 2 + uni() * 5, uni(), (int)(rnd() % 2), (int)(rnd() % 2));
      for (int d = 0; d < 6; d++) {
        double q[3] = {at[0] + uni(), at[1] + uni(), at[2] + uni()};
        bk_sculpt_dab(sc, q, uni(), uni());
      }
      bk_sculpt_end(sc);
      int changed = bk_sculpt_sync(sc), tch = bk_sculpt_changed_triangle_count(sc), vc = bk_sculpt_vertex_count(sc), tc = bk_sculpt_triangle_count(sc);
      const uint32_t *cv = bk_sculpt_changed(sc), *ct = bk_sculpt_changed_triangles(sc);
      for (int i = 0; i < changed; i++)
        if (cv[i] >= (uint32_t)vc) fail(shape + ": a changed point past the slots");
      for (int i = 0; i < tch; i++)
        if (ct[i] >= (uint32_t)tc) fail(shape + ": a changed triangle past the slots");
      int live = bk_sculpt_live_triangle_count(sc);
      BKSculptMesh *back = bk_sculpt_mesh(sc);
      if (!back || back->triangleCount != live) fail(shape + ": a sculpt's mesh not its live triangles");
      bk_sculpt_mesh_free(back), bk_sculpt_free(sc);
    }
    bk_sculpt_mesh_free(rm);
  }
  // STEP: written into a temporary file (a name or none; a path that can't be written refused saying why).
  doing(shape + ": STEP", 120);
  char path[] = "/tmp/bcad-fuzz-XXXXXX";
  int fd = mkstemp(path);
  if (fd >= 0) {
    close(fd);
    const BKShape *list[1] = {s};
    const char *names[1] = {rnd() % 2 ? "Fuzzed ‘body’ \\ ü" : nullptr};
    if (!bk_export_step(list, rnd() % 2 ? names : nullptr, 1, path)) fail(shape + ": STEP not written: " + bk_last_error());
    unlink(path);
  }
  if (bk_export_step(nullptr, nullptr, 1, path) || !*bk_last_error()) fail(shape + ": STEP of nothing written");
  const BKShape *list[1] = {s};
  if (bk_export_step(list, nullptr, 1, "/nonexistent-dir/x.step") || !*bk_last_error()) fail(shape + ": STEP into nowhere written");
  bk_mesh_free(m), bk_free(s);
}

// MARK: scan files
// Files as other apps write them, then broken every way: every read refused saying why (in a way the app knows), or a
// mesh whose every corner is one of its points and whose parts cover its triangles in turn, the same each time.
struct Soup3 {
  std::vector<float> p;
  std::vector<uint32_t> t;
};
static Soup3 randomSoup(int points, int triangles) {
  Soup3 s;
  for (int i = 0; i < 3 * points; i++) {
    double r = uni();
    s.p.push_back(rnd() % 8 == 0 ? (float)(std::pow(10.0, 12 * r - 6) * (rnd() % 2 ? 1 : -1)) : (float)(200 * r - 100));
  }
  for (int i = 0; i < 3 * triangles; i++) s.t.push_back((uint32_t)(rnd() % points));
  return s;
}
static void put32(std::string &s, uint32_t v) {
  for (int k = 0; k < 4; k++) s += (char)(v >> 8 * k & 255);
}
static std::string asStl(const Soup3 &m, bool text) {
  std::string s;
  char b[200];
  if (text) s = "solid fuzz\n";
  else s = std::string(80, ' '), put32(s, (uint32_t)(m.t.size() / 3));
  for (size_t k = 0; k < m.t.size(); k += 3) {
    if (text) s += "facet normal 0 0 1\nouter loop\n";
    else s += std::string(12, '\0');
    for (int c = 0; c < 3; c++) {
      const float *q = &m.p[3 * m.t[k + c]];
      if (text) snprintf(b, sizeof b, "vertex %.9g %.9g %.9g\n", q[0], q[1], q[2]), s += b;
      else
        for (int i = 0; i < 3; i++) {
          uint32_t bits;
          memcpy(&bits, q + i, 4);
          put32(s, bits);
        }
    }
    s += text ? "endloop\nendfacet\n" : std::string(2, '\0');
  }
  return text ? s + "endsolid fuzz\n" : s;
}
static std::string asObj(const Soup3 &m) {
  std::string s = "o fuzz\n";
  char b[200];
  for (size_t i = 0; i < m.p.size(); i += 3) snprintf(b, sizeof b, "v %.9g %.9g %.9g\n", m.p[i], m.p[i + 1], m.p[i + 2]), s += b;
  for (size_t k = 0; k < m.t.size(); k += 3) {
    // Every way of naming a corner, counting back too.
    int n = (int)(m.p.size() / 3);
    s += "f";
    for (int c = 0; c < 3; c++) {
      int v = (int)m.t[k + c] + 1;
      snprintf(b, sizeof b, rnd() % 4 == 0 ? " %d/1" : rnd() % 3 == 0 ? " %d//2" : " %d", rnd() % 3 == 0 ? v - n - 1 : v);
      s += b;
    }
    s += "\n";
  }
  return s;
}
static std::string asPly(const Soup3 &m, int format) {
  std::string s = std::string("ply\nformat ") + (format == 0 ? "ascii" : format == 1 ? "binary_little_endian" : "binary_big_endian") +
                  " 1.0\nelement vertex " + std::to_string(m.p.size() / 3) + "\nproperty float x\nproperty float y\nproperty float z\n" +
                  "element face " + std::to_string(m.t.size() / 3) + "\nproperty list uchar int vertex_indices\nend_header\n";
  char b[200];
  auto raw = [&](uint32_t v, int size) {
    for (int k = 0; k < size; k++) s += (char)(v >> 8 * (format == 2 ? size - 1 - k : k) & 255);
  };
  for (size_t i = 0; i < m.p.size(); i += 3)
    if (format == 0) snprintf(b, sizeof b, "%.9g %.9g %.9g\n", m.p[i], m.p[i + 1], m.p[i + 2]), s += b;
    else
      for (int k = 0; k < 3; k++) {
        uint32_t bits;
        memcpy(&bits, &m.p[i + k], 4);
        raw(bits, 4);
      }
  for (size_t k = 0; k < m.t.size(); k += 3)
    if (format == 0) snprintf(b, sizeof b, "3 %u %u %u\n", m.t[k], m.t[k + 1], m.t[k + 2]), s += b;
    else raw(3, 1), raw(m.t[k], 4), raw(m.t[k + 1], 4), raw(m.t[k + 2], 4);
  return s;
}
static std::string as3mf(const Soup3 &m) {
  std::string s = "<?xml version=\"1.0\"?>\n<model unit=\"millimeter\"><resources><object id=\"1\" name=\"fuzz\"><mesh><vertices>";
  char b[200];
  for (size_t i = 0; i < m.p.size(); i += 3) snprintf(b, sizeof b, "<vertex x=\"%.9g\" y=\"%.9g\" z=\"%.9g\"/>", m.p[i], m.p[i + 1], m.p[i + 2]), s += b;
  s += "</vertices><triangles>";
  for (size_t k = 0; k < m.t.size(); k += 3) snprintf(b, sizeof b, "<triangle v1=\"%u\" v2=\"%u\" v3=\"%u\"/>", m.t[k], m.t[k + 1], m.t[k + 2]), s += b;
  return s + "</triangles></mesh></object><object id=\"2\"><components><component objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 0 0 0\"/>"
             "</components></object></resources><build><item objectid=\"2\"/></build></model>";
}
static BKScanSoup *readFile(const std::string &bytes, const char *ext) {
  if (std::string(ext) == "3mf") {
    const char *name = "3D/3dmodel.model";
    const uint8_t *data = (const uint8_t *)bytes.data();
    int64_t n = (int64_t)bytes.size();
    return bk_scan_read_3mf(&name, &data, &n, 1, nullptr);
  }
  return bk_scan_read((const uint8_t *)bytes.data(), (int64_t)bytes.size(), ext, nullptr);
}
// What's wrong with a soup as read ("" when nothing), and whether two reads are the same to the bit.
static std::string soupWrong(const BKScanSoup *s) {
  if (s->triangleCount < 1 || s->vertexCount < 1 || s->partCount < 1) return "empty";
  if (s->partStart[0] != 0 || s->partStart[s->partCount] != s->triangleCount) return "parts not covering the triangles";
  for (int k = 0; k < s->partCount; k++)
    if (s->partStart[k + 1] <= s->partStart[k] || !s->partNames[k]) return "an empty part";
  for (int i = 0; i < 3 * s->triangleCount; i++)
    if (s->indices[i] >= (uint32_t)s->vertexCount) return "a corner that isn't a point";
  if (!(s->scale > 0) || s->skipped < 0) return "a unit or count wrong";
  return "";
}
static bool sameSoup(const BKScanSoup *a, const BKScanSoup *b) {
  if (!a || !b) return !a && !b;
  if (a->vertexCount != b->vertexCount || a->triangleCount != b->triangleCount || a->partCount != b->partCount || a->scale != b->scale) return false;
  if (memcmp(a->positions, b->positions, 12 * (size_t)a->vertexCount) || memcmp(a->indices, b->indices, 12 * (size_t)a->triangleCount)) return false;
  for (int k = 0; k < a->partCount; k++)
    if (a->partStart[k] != b->partStart[k] || strcmp(a->partNames[k], b->partNames[k])) return false;
  return true;
}
static int scanRead = 0, scanRefused = 0;
static void readCase(const std::string &bytes, const char *ext, const std::string &what) {
  doing(what, 60);
  BKScanSoup *a = readFile(bytes, ext);
  std::string why = a ? "" : bk_last_error();
  BKScanSoup *b = readFile(bytes, ext);
  if (!sameSoup(a, b)) fail(what + ": read twice, not the same");
  if (a) {
    scanRead++;
    std::string wrong = soupWrong(a);
    if (!wrong.empty()) fail(what + ": " + wrong);
  } else {
    scanRefused++;
    static const char *known[] = {"stl: ", "obj: ", "ply: ", "3mf: ", "scan: ", "not enough memory"};
    bool ok = false;
    for (const char *k : known) ok = ok || why.compare(0, strlen(k), k) == 0;
    if (!ok) fail(what + ": refused without a reason the app knows: " + why);
  }
  bk_scan_soup_free(a), bk_scan_soup_free(b);
}
static std::string damaged(std::string s) {
  if (s.empty()) return s;
  switch (rnd() % 6) {
    case 0:  // bits flipped
      for (int k = 1 + (int)(rnd() % 8); k > 0; k--) s[rnd() % s.size()] ^= (char)(1 << rnd() % 8);
      break;
    case 1:  // cut short
      s.resize(rnd() % s.size());
      break;
    case 2: {  // a number made absurd
      static const char *absurd[] = {"4294967295", "99999999999999999999", "-1", "1e999", "nan", "-0", "1e-999", "0x10", "", "1.#INF"};
      size_t at = rnd() % s.size();
      size_t from = s.find_first_of("0123456789", at);
      if (from == std::string::npos) break;
      size_t to = s.find_first_not_of("0123456789.e-+", from);
      s.replace(from, (to == std::string::npos ? s.size() : to) - from, pick(absurd));
      break;
    }
    case 3: {  // a run of bytes repeated or taken out
      size_t at = rnd() % s.size(), n = std::min<size_t>(s.size() - at, 1 + rnd() % 64);
      if (rnd() % 2) s.insert(at, s.substr(at, n));
      else s.erase(at, n);
      break;
    }
    case 4:  // a binary count made huge
      if (s.size() >= 84) put32(s, 0), s.replace(80, 4, std::string("\xff\xff\xff\x7f", 4)), s.resize(s.size() - 4);
      break;
    default: {  // random bytes over a stretch
      size_t at = rnd() % s.size();
      for (size_t i = at; i < std::min(s.size(), at + 1 + rnd() % 32); i++) s[i] = (char)rnd();
    }
  }
  return s;
}
static void scanFiles(int iters) {
  static const char *exts[] = {"stl", "stl", "obj", "ply", "ply", "ply", "3mf"};
  auto write = [](const Soup3 &m, int f) {
    return f == 0 ? asStl(m, false) : f == 1 ? asStl(m, true) : f == 2 ? asObj(m) : f <= 5 ? asPly(m, f - 3) : as3mf(m);
  };
  // Read back as written: every format the same triangles (binary and 9-digit text both keep a float to the bit).
  for (int run = 0; run < 10 * iters; run++) {
    Soup3 m = randomSoup(3 + (int)(rnd() % 40), 1 + (int)(rnd() % 60));
    for (int f = 0; f < 7; f++) {
      std::string what = fmt("random soup %.0f as format %.0f", run, f);
      doing(what, 60);
      BKScanSoup *s = readFile(write(m, f), exts[f]);
      if (!s) {
        fail(what + ": refused: " + bk_last_error());
        continue;
      }
      std::vector<float> corners;
      double scale = s->unitGuessed ? s->scale : 1;
      for (int i = 0; i < 3 * s->triangleCount; i++)
        for (int k = 0; k < 3; k++) corners.push_back(s->positions[3 * s->indices[i] + k]);
      if (s->triangleCount != (int)m.t.size() / 3) fail(what + fmt(": %.0f triangles, not %.0f", s->triangleCount, m.t.size() / 3));
      else if (scale == 1)
        for (size_t i = 0; i < m.t.size(); i++)
          for (int k = 0; k < 3; k++)
            if (corners[3 * i + k] != m.p[3 * m.t[i] + k] + 0.0f) {
              fail(what + fmt(": corner %.0f not as written (%.9g)", i, corners[3 * i + k]));
              i = m.t.size();
              break;
            }
      bk_scan_soup_free(s);
    }
  }
  // Broken every way.
  Soup3 cube;
  for (int i = 0; i < 8; i++) cube.p.insert(cube.p.end(), {i & 1 ? 10.f : 0.f, i & 2 ? 10.f : 0.f, i & 4 ? 10.f : 0.f});
  cube.t = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
  for (int f = 0; f < 7; f++) {
    std::string file = write(cube, f);
    for (size_t cut = 0; cut < file.size(); cut += 1 + file.size() / 200) readCase(file.substr(0, cut), exts[f], fmt("cube as format %.0f cut at %.0f", f, cut));
    for (int k = 0; k < 150 * iters; k++) {
      std::string bad = file;
      for (int d = 1 + (int)(rnd() % 3); d > 0; d--) bad = damaged(bad);
      readCase(bad, exts[f], fmt("cube as format %.0f damaged (%.0f)", f, k));
    }
  }
  for (int k = 0; k < 40 * iters; k++) {
    std::string junk(rnd() % 2000, '\0');
    for (char &c : junk) c = rnd() % 3 ? (char)rnd() : " \n0123456789.-evfsolidplyx"[rnd() % 26];
    for (const char *ext : {"stl", "obj", "ply", "3mf"}) readCase(junk, ext, fmt("junk %.0f", k) + " as " + ext);
  }
}

int main() {
  signal(SIGALRM, watchdog);
  int iters = getenv("BCAD_FUZZ_ITERS") ? std::max(1, atoi(getenv("BCAD_FUZZ_ITERS"))) : 1;
  if (getenv("BCAD_FUZZ_SEED")) seed ^= strtoull(getenv("BCAD_FUZZ_SEED"), nullptr, 10) * 0x2545f4914f6cdd1dull;
  printf("fuzz: %d× (seed %llu)\n", iters, (unsigned long long)seed);
  // BCAD_FUZZ_ONLY: sculpting, figures, extremes, api or scans alone.
  std::string only = getenv("BCAD_FUZZ_ONLY") ? getenv("BCAD_FUZZ_ONLY") : "";

  // Sculpting.
  if (only.empty() || only == "sculpting") {
    printf("— sculpting\n");
    int before = failures;
    double b20[3] = {20, 20, 20}, d30[1] = {30}, t[3] = {0, 40, 12};
    BKShape *shapes[3] = {bk_primitive(BK_SPHERE, d30), bk_primitive(BK_BOX, b20), bk_primitive(BK_TORUS, t)};
    for (int run = 0; run < 6 * iters; run++) sculptRun(run, shapes[run % 3], run % 2 ? 1.5 : 0.8);
    for (BKShape *s : shapes) bk_free(s);
    check("sculpting: random strokes, undone and done again, the changed lists always as the sculptor's state; all undone the start, all done the end",
          failures == before, fmt("%.0f runs", 6 * iters));
  }

  // Figures at the ends of every range, and at random.
  if (only.empty() || only == "figures") {
    printf("— figures\n");
    int before = failures, made = 0;
    for (int sex = 0; sex < 2; sex++)
      for (int f = 0; f < BK_FIG_COUNT; f++)
        for (int end = 0; end < 2; end++) {
          std::vector<double> p(BK_FIG_COUNT);
          bk_figure_defaults(sex, p.data());
          double r[2];
          bk_figure_range(f, r);
          p[f] = r[end];
          int draft = (f + end + sex) % 4 != 0;
          doing(fmt("figure sex %.0f field %.0f at %g", sex, f, p[f]));
          std::string why;
          if (figureSound(p, draft, why)) made++;
          else fail(fmt("figure sex %.0f, number %.0f at %g", sex, f, p[f]) + fmt(" (draft %.0f): ", draft) + why);
        }
    for (int k = 0; k < 6 * iters; k++) {
      std::vector<double> p(BK_FIG_COUNT);
      for (int f = 0; f < BK_FIG_COUNT; f++) {
        double r[2];
        bk_figure_range(f, r);
        p[f] = r[0] + (r[1] - r[0]) * (rnd() % 4 ? uni() : (double)(rnd() % 2));
      }
      if (rnd() % 3 == 0) p[BK_FIG_HEIGHT] = 100;
      int draft = k % 3 != 0;
      doing(fmt("random figure %.0f", k));
      std::string why;
      if (figureSound(p, draft, why)) made++;
      else {
        std::string nums;
        for (double v : p) nums += fmt(" %.17g", v);
        fail(fmt("random figure %.0f (draft %.0f): ", k, draft) + why + " ·" + nums);
      }
    }
    // Numbers that aren't a figure's: refused, never made.
    static const double bad[] = {NAN, INFINITY, -INFINITY, -1e300, 1e300};
    for (int k = 0; k < 20; k++) {
      std::vector<double> p(BK_FIG_COUNT);
      bk_figure_defaults(0, p.data());
      p[rnd() % BK_FIG_COUNT] = pick(bad);
      double ext[6];
      BKShape *s = bk_figure(p.data(), BK_FIG_COUNT, 1);
      if (s || bk_figure_extent(p.data(), BK_FIG_COUNT, ext)) fail("a figure made of numbers out of range");
      bk_free(s);
    }
    check("figures: every number at both ends of its range and at random: closed, one piece, its box as worked out; numbers out of range refused",
          failures == before, fmt("%.0f made", made));
  }

  // Everything else, from zero to huge and not numbers at all.
  if (only.empty() || only == "extremes") {
    printf("— extremes\n");
    int before = failures, made = 0, refused = 0;
    // Sizes as the app takes them (up to 10 m), and wild ones (zero, tiny, huge, not numbers): a shape of wild numbers
    // must be refused, or made; only those of the app's sizes are worked on further (a 100 m shape meshed to a hundredth
    // of a millimetre would take billions of triangles).
    static const double sizes[] = {0, 1e-9, 1e-4, 0.001, 0.01, 1, 3, 10, 50, 500, 5000, 1e5, 1e9, -1, NAN, INFINITY};
    static const double sides[] = {3, 4, 6, 8, 24, 64, 2.4, 65, 1e4, 1e12, -5, NAN};
    static const int tubes[] = {0, 3, 6};
    static const double normals[] = {0, 1, -1, 1e-300, NAN}, clearances[] = {0, 0.2, -1, NAN, 1e3};
    auto size = [&]() { return rnd() % 3 ? std::exp(uni() * 9 - 2) : pick(sizes); };
    std::string last;  // the last primitive's numbers
    double largest = 0, smallest = INFINITY;
    bool wild = false;
    auto primitive = [&](int kind) {
      double p[5];
      for (double &v : p) v = size();
      if (kind == BK_PRISM || kind == BK_PYRAMID) p[0] = pick(sides);
      if (kind == BK_TORUS || kind == BK_OVAL_TORUS) p[0] = rnd() % 4 ? (double)pick(tubes) : pick(sides);
      if (kind == BK_OVAL) p[2] = rnd() % 4 ? 5 + uni() * 170 : pick(sizes);
      if (kind == BK_OVAL_TORUS) p[3] = rnd() % 4 ? 5 + uni() * 170 : pick(sizes);
      last = fmt("kind %.0f: %.17g", kind, p[0]) + fmt(" %.17g %.17g", p[1], p[2]) + fmt(" %.17g %.17g", p[3], p[4]);
      largest = 0, smallest = INFINITY, wild = false;
      for (double v : p) {
        wild = wild || !(v >= 0 && v <= 1e4), largest = std::max(largest, std::fabs(v));
        if (v > 0) smallest = std::min(smallest, v);
      }
      return bk_primitive(kind, p);
    };
    auto judge = [&](BKShape *s, const std::string &what, bool deep = true) {
      if (!s) {
        refused++;
        if (!*bk_last_error()) fail(what + ": refused without saying why");
        return;
      }
      std::string why;
      if (!deep) {
        double bb[6];
        bk_bounds(s, I, bb);
        made++;
      } else if (sound(s, why)) made++;
      else fail(what + ": made but not sound: " + why);
    };
    for (int k = 0; k < 120 * iters; k++) {
      int kind = (int)(rnd() % 16) - 1;
      doing(fmt("extremes %.0f: primitive %.0f", k, kind), 60);
      BKShape *s = primitive(kind);
      std::string shape = last;
      if (verbose) printf("  %s\n", shape.c_str()), fflush(stdout);
      bool tame = !wild;
      judge(s, fmt("primitive %.0f (case %.0f)", kind, k), tame);
      if (!s || !tame) {
        bk_free(s);
        continue;
      }
      // Placed: turned, stretched, moved, any way (wild placements refused or made, the rest worked on).
      double m[12];
      for (double &v : m) v = (uni() - 0.5) * 2;
      static const double scales[] = {1e-3, 0.1, 1, 3, 1e-9, 1e6, 0, NAN}, moves[] = {0, 1, 1e4, -1e4, 1e9, NAN};
      double sc = pick(scales), mv = pick(moves);
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) m[4 * i + j] *= sc;
      m[3] = mv;
      // (A mesh's points are floats: moved far, a part smaller than a float's step there can't be told apart from its
      // neighbour, and its triangles come out with two corners in one place.)
      bool placedTame = sc >= 1e-3 && sc <= 3 && largest * sc <= 1e4 && std::fabs(mv) <= 1e4 && smallest * sc >= 1e-6 * std::fabs(mv);
      doing(fmt("extremes %.0f: placed (scale %g, moved %g)", k, sc, mv), 60);
      BKShape *t = bk_transform(s, m);
      judge(t, fmt("primitive %.0f placed", kind) + fmt(" at scale %g, moved %g", sc, mv) + fmt(" (case %.0f)", k), placedTame);
      // Merged with another.
      BKShape *o = primitive((int)(rnd() % 14));
      if (verbose) printf("  and %s\n", last.c_str());
      if (o && !wild) {
        int op = (int)(rnd() % 3);
        doing(fmt("extremes %.0f: boolean %.0f", k, op), 120);
        BKShape *b = bk_boolean(op, s, o);
        if (b) {
          std::string why;
          if (!sound(b, why)) {
            // (Merging may leave nothing: an empty result is fine.)
            BKMesh *mm = bk_mesh(b, 0.05);
            bool empty = mm && mm->triangleCount == 0;
            bk_mesh_free(mm);
            if (!empty) fail(fmt("boolean %.0f of primitive %.0f", op, kind) + fmt(" and another (case %.0f): ", k) + why);
          }
        }
        bk_free(b);
      }
      bk_free(o);
      // Meshed coarse to fine (finer than a thousandth taken as that).
      static const double deflections[] = {1e-9, 0.01, 0.05, 1, 100, -1, NAN, INFINITY};
      double d = largest <= 100 ? pick(deflections) : pick(deflections) + 0.01;
      doing(fmt("extremes %.0f: mesh at %g", k, d), 60);
      BKMesh *mm = bk_mesh(s, d);
      if (mm && mm->valid && mm->triangleCount == 0) fail(fmt("primitive %.0f meshed at %g: valid but empty", kind, d));
      bk_mesh_free(mm);
      // Rounded, bevelled, coved, hollowed, remeshed, split, cut across, measured.
      static const double tools[] = {-1, 0, 1e-9, 1e-3, 0.1, 0.5, 2, 1e3, NAN};
      int kinds[1] = {BK_PICK_BODY};
      double picks[6] = {0, 0, 0, 0, 0, 0}, maxR = 0;
      int missing = 0;
      double rr = pick(tools);
      doing(fmt("extremes %.0f: fillet %g", k, rr), 120);
      BKShape *f = bk_fillet(s, kinds, picks, 1, rr, &maxR, &missing);
      judge(f, fmt("fillet %g of primitive %.0f", rr, kind) + fmt(" (case %.0f)", k));
      bk_free(f);
      double la = pick(tools), lb = rnd() % 2 ? la : pick(tools);
      doing(fmt("extremes %.0f: chamfer %g %g", k, la, lb), 120);
      f = bk_chamfer(s, kinds, picks, 1, la, lb, rnd() % 4 ? 0 : pick(tools), &missing);
      judge(f, fmt("chamfer %g %g of primitive", la, lb) + fmt(" %.0f (case %.0f)", kind, k));
      bk_free(f);
      rr = pick(tools);
      doing(fmt("extremes %.0f: cove %g", k, rr), 120);
      f = bk_cove(s, kinds, picks, 1, rr, &maxR, &missing);
      judge(f, fmt("cove %g of primitive %.0f", rr, kind) + fmt(" (case %.0f)", k));
      bk_free(f);
      double th = pick(tools);
      doing(fmt("extremes %.0f: hollow %g", k, th), 120);
      f = bk_hollow(s, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, th, &missing);
      judge(f, fmt("hollow %g of primitive %.0f", th, kind) + fmt(" (case %.0f)", k));
      bk_free(f);
      static const double details[] = {-1, 0, 1e-9, 1e-3, 0.3, 1, 5, 1e3, NAN};
      double dt = pick(details);
      doing(fmt("extremes %.0f: remesh %g", k, dt), 120);
      BKSculptMesh *rm = bk_remesh(s, I, dt);
      if (rm && (rm->vertexCount <= 0 || rm->triangleCount <= 0)) fail(fmt("remesh at %g of primitive %.0f: empty", dt, kind));
      bk_sculpt_mesh_free(rm);
      double at[3] = {(uni() - 0.5) * 2, (uni() - 0.5) * 2, (uni() - 0.5) * 2};
      double n[3] = {pick(normals), uni() - 0.5, uni() - 0.5};
      if (rnd() % 4 == 0) n[1] = n[2] = 0;
      doing(fmt("extremes %.0f: split", k), 120);
      f = bk_split(s, at, n, (int)(rnd() % 2));
      if (f) {
        std::string why;
        BKMesh *sm = bk_mesh(f, 0.05);
        bool empty = sm && sm->triangleCount == 0;
        bk_mesh_free(sm);
        if (!empty && !sound(f, why)) fail(fmt("split of primitive %.0f (case %.0f): ", kind, k) + why);
      }
      bk_free(f);
      doing(fmt("extremes %.0f: section", k), 60);
      int sk = (int)(rnd() % 4);
      double sp[6] = {uni() - 0.5, uni() - 0.5, uni() - 0.5, uni() - 0.5, uni() - 0.5, uni() - 0.5};
      BKSection *sec = bk_section(s, sk, sp, pick(tools));
      bk_section_free(sec);
      doing(fmt("extremes %.0f: distance", k), 60);
      double pa[3] = {pick(sizes), 0, 0}, out[6];
      bk_distance(s, I, BK_END_FACE, (int)(rnd() % 5) - 1, nullptr, s, m, BK_END_POINT, 0, pa, out);
      doing(fmt("extremes %.0f: bounds and pieces", k), 60);
      double bb[6];
      bk_bounds(s, m, bb);
      bk_piece_count(s);
      bk_free(t), bk_free(s);
    }
    // Bolts and nuts, sizes out of their ranges.
    static const double fsizes[] = {-1, 0, 1e-6, 0.5, 2, 10, 30, 200, 1e6, NAN, INFINITY};
    for (int k = 0; k < 60 * iters; k++) {
      BKFastener f;
      f.kind = (int)(rnd() % 18) - 1, f.size = (int)(rnd() % (bk_thread_count() + 2)) - 1;
      doing(fmt("fastener %.0f kind %.0f", k, f.kind), 60);
      if (f.kind >= 0 && f.kind <= BK_CONE_NUT && f.size >= 0 && f.size < bk_thread_count()) bk_fastener_defaults(&f, 1);
      else f.length = f.width = f.height = f.angle = f.seat = f.drive = f.recess = f.depth = 1;
      double *fields[] = {&f.length, &f.width, &f.height, &f.angle, &f.seat, &f.drive, &f.recess, &f.depth};
      if (rnd() % 2) *fields[rnd() % 8] = pick(fsizes);
      double range[2], ext[3];
      bk_fastener_range(&f, (int)(rnd() % 10) - 1, (int)(rnd() % 2), range);
      if (rnd() % 3 == 0) bk_fastener_fit(&f);
      if (rnd() % 5 == 0) bk_fastener_drive(&f, pick(fsizes));
      bk_fastener_extent(&f, pick(clearances), ext);
      double clearance = pick(clearances);
      if (verbose)
        printf("  size %d: length %.17g width %.17g height %.17g angle %.17g seat %.17g drive %.17g recess %.17g depth %.17g, clearance %.17g\n", f.size,
               f.length, f.width, f.height, f.angle, f.seat, f.drive, f.recess, f.depth, clearance);
      BKShape *s = bk_fastener(&f, clearance);
      judge(s, fmt("fastener kind %.0f size %.0f (case %.0f)", f.kind, f.size, k));
      bk_free(s);
    }
    bk_thread_name(-1), bk_thread_name(9999), bk_torx_number(-1), bk_torx_number(9999), bk_fastener_fields(-1), bk_fastener_fields(99);
    double r[2];
    bk_figure_range(-1, r), bk_figure_range(999, r);
    double pose[BK_FIG_COUNT];
    bk_figure_defaults(0, pose), bk_figure_pose(-1, pose), bk_figure_pose(999, pose);
    check("extremes: shapes, placements, merges, meshes, roundings, bevels, coves, hollows, remeshes, splits, sections, distances and fasteners: "
          "refused (saying why) or made sound",
          failures == before, fmt("%.0f made, %.0f refused", made, refused));
  }
  // The rest of the API.
  if (only.empty() || only == "api") {
    printf("— the rest of the API\n");
    int before = failures;
    for (int k = 0; k < 30 * iters; k++) apiRun(k);
    // Figures from fewer numbers (the rest standard), none, or none at all; their pose named.
    std::vector<double> few(BK_FIG_COUNT);
    bk_figure_defaults(1, few.data());
    for (int count : {0, 1, 5, 12, BK_FIG_COUNT - 1}) {
      doing(fmt("figure of %.0f numbers", count), 60);
      BKShape *f = bk_figure(few.data(), count, 1);
      std::string why;
      if (f && !sound(f, why)) fail(fmt("a figure of %.0f numbers: ", count) + why);
      bk_free(f);
      bk_figure_pose_of(few.data(), count);
    }
    if (bk_figure(nullptr, BK_FIG_COUNT, 1) || bk_figure_pose_of(nullptr, 3) != -1) fail("a figure of no numbers");
    if (bk_torx_count() <= 0) fail("no Torx sizes");
    check("the rest of the API: picks of its own faces, edges and corners rounded, bevelled, coved; hollows opened and walled; edges "
          "listed; distances; broken meshes; mesh bodies; STEP; figures of fewer numbers; a sculpt's changed lists: refused saying why, or "
          "made sound",
          failures == before, fmt("%.0f made, %.0f refused", apiMade, apiRefused));
  }
  // Files from other apps and scanners.
  if (only.empty() || only == "scans") {
    printf("— scans\n");
    int before = failures;
    scanFiles(iters);
    check("scan files as written, cut short, damaged and junk: read back as written, or refused saying why; the same every time",
          failures == before, fmt("%.0f read, %.0f refused", scanRead, scanRefused));
  }
  alarm(0);
  printf(failures ? "%d FAILED\n" : "ALL OK\n", failures);
  return failures ? 1 : 0;
}
