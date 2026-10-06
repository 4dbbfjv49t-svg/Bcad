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

int main() {
  signal(SIGALRM, watchdog);
  int iters = getenv("BCAD_FUZZ_ITERS") ? std::max(1, atoi(getenv("BCAD_FUZZ_ITERS"))) : 1;
  if (getenv("BCAD_FUZZ_SEED")) seed ^= strtoull(getenv("BCAD_FUZZ_SEED"), nullptr, 10) * 0x2545f4914f6cdd1dull;
  printf("fuzz: %d× (seed %llu)\n", iters, (unsigned long long)seed);
  // BCAD_FUZZ_ONLY: sculpting, figures or extremes alone.
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
      if (verbose) printf("  %s\n", shape.c_str());
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
  alarm(0);
  printf(failures ? "%d FAILED\n" : "ALL OK\n", failures);
  return failures ? 1 : 0;
}
