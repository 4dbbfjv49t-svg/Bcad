// Bcad's own geometry engine against exact maths: volumes and boxes from formulas, closed outward meshes, the faces,
// edges, corners and circles the app works with, placements, merging and splitting (again and again), distances, exact
// orientation, and how fast it all is.
// c++ -std=c++17 -O2 -I. test/engine.cpp Engine/*.cpp -o engine-test && ./engine-test
#include "BcadKernel.h"
#include "Engine/Bolts.hpp"
#include "Engine/Fasteners.hpp"
#include "Engine/Implicit.hpp"
#include "Engine/Math.hpp"
#include "Engine/Model.hpp"
#include "Engine/Print.hpp"
#include "Engine/Step.hpp"
#include "Engine/Treat.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <random>
#include <string>
#include <tuple>
#include <vector>

static int failures = 0;
// Speeds are checked unless BCAD_UNTIMED is set (under the sanitizers, everything is several times slower).
static const bool timed = !getenv("BCAD_UNTIMED");
static void check(const char *name, bool ok, const std::string &note = "") {
  printf("%s %s %s\n", ok ? "✓" : "✗", name, note.c_str());
  if (!ok) failures++;
}
static std::string fmt(const char *f, double a, double b = 0, double c = 0) {
  char s[200];
  snprintf(s, sizeof s, f, a, b, c);
  return s;
}
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
static double ms(std::chrono::steady_clock::time_point t) { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count(); }

static const double PI = M_PI;
static const double I[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};

// The perimeter of an ellipse by the Gauss–Kummer series (a different road from the engine's).
static double perimeter(double a, double b) {
  double h = (a - b) * (a - b) / ((a + b) * (a + b)), sum = 1, c = 1;
  for (int n = 1; n < 60; n++) {
    c *= (0.5 - (n - 1)) / n;  // binomial(1/2, n)
    sum += c * c * std::pow(h, n);
  }
  return PI * (a + b) * sum;
}

// A mesh's closedness: every edge between two triangles, run opposite ways (welded by position), and its signed volume.
// `touching`: a shape may touch itself along a line (a merge of shapes meeting there), so an edge may have four
// triangles, two each way.
static bool closed(const BKMesh *m, double &signedVolume, std::string &why, bool touching = false) {
  std::map<std::tuple<float, float, float>, int> weld;
  std::vector<int> id(m->vertexCount);
  for (int i = 0; i < m->vertexCount; i++) {
    auto key = std::make_tuple(m->positions[3 * i], m->positions[3 * i + 1], m->positions[3 * i + 2]);
    auto it = weld.find(key);
    id[i] = it == weld.end() ? (weld[key] = (int)weld.size()) : it->second;
  }
  std::map<std::pair<int, int>, int> directed;
  signedVolume = 0;
  for (int t = 0; t < m->triangleCount; t++) {
    int v[3];
    double p[3][3];
    for (int k = 0; k < 3; k++) {
      v[k] = id[m->indices[3 * t + k]];
      for (int c = 0; c < 3; c++) p[k][c] = m->positions[3 * m->indices[3 * t + k] + c];
    }
    if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) {
      why = "a triangle with two corners in one place";
      return false;
    }
    signedVolume += (p[0][0] * (p[1][1] * p[2][2] - p[1][2] * p[2][1]) - p[0][1] * (p[1][0] * p[2][2] - p[1][2] * p[2][0]) +
                     p[0][2] * (p[1][0] * p[2][1] - p[1][1] * p[2][0])) / 6;
    for (int k = 0; k < 3; k++) directed[{v[k], v[(k + 1) % 3]}]++;
  }
  for (auto &[e, n] : directed) {
    auto back = directed.find({e.second, e.first});
    if (n != 1 && !(touching && back != directed.end() && back->second == n)) {
      why = "an edge run the same way twice";
      return false;
    }
    if (back == directed.end()) {
      why = "an open edge";
      return false;
    }
  }
  return true;
}

struct Case {
  const char *name;
  int kind;
  std::vector<double> p;
  double volume;
  double lo[3], hi[3];
  int faces, edges, corners, circles;  // as the app expects them for the shape
};

int main() {
  // MARK: shapes
  double r3 = std::sqrt(3.0), h4 = 4 * r3 / 2, h3 = 3 * r3 / 2;
  double prism5R = 10, c5 = std::cos(PI / 5);
  // Oval with conjugate semi-diameters 10 and 7 at 60°: the box from the support function by hand.
  double ox = std::sqrt(100 + 49 * 0.25), oy = 7 * std::sin(PI / 3);
  // Oval torus 30 × 20 at 90°, tube 6: an ellipse of 12 × 7 swept with a tube of radius 3.
  double ovalLine = perimeter(12, 7);
  std::vector<Case> cases = {
      {"box", BK_BOX, {20, 30, 10}, 6000, {-10, -15, -5}, {10, 15, 5}, 6, 12, 8, 0},
      {"cylinder", BK_CYLINDER, {20, 20}, PI * 100 * 20, {-10, -10, -10}, {10, 10, 10}, 3, 3, 2, 2},
      {"cone", BK_CONE, {20, 0, 20}, PI * 100 * 20 / 3, {-10, -10, -10}, {10, 10, 10}, 2, 3, 2, 1},
      {"cut cone", BK_CONE, {20, 10, 20}, PI * 20 * (100 + 50 + 25) / 3, {-10, -10, -10}, {10, 10, 10}, 3, 3, 2, 2},
      {"cone upside down", BK_CONE, {0, 20, 20}, PI * 100 * 20 / 3, {-10, -10, -10}, {10, 10, 10}, 2, 3, 2, 1},
      {"sphere", BK_SPHERE, {20}, 4 * PI * 1000 / 3, {-10, -10, -10}, {10, 10, 10}, 1, 3, 2, 1},
      {"5-sided prism", BK_PRISM, {5, 20, 20}, 2.5 * 100 * std::sin(2 * PI / 5) * 20, {-prism5R * std::sin(2 * PI / 5), -prism5R * (1 + c5) / 2, -10}, {prism5R * std::sin(2 * PI / 5), prism5R * (1 + c5) / 2, 10}, 7, 15, 10, 0},
      {"6-sided prism", BK_PRISM, {6, 20, 20}, 3 * 100 * std::sin(PI / 3) * 20, {-10, -10 * std::sin(PI / 3), -10}, {10, 10 * std::sin(PI / 3), 10}, 8, 18, 12, 0},
      {"torus", BK_TORUS, {0, 30, 8}, 2 * PI * PI * 11 * 16, {-15, -15, -4}, {15, 15, 4}, 1, 2, 1, 2},
      {"triangle torus", BK_TORUS, {3, 30, 8}, 2 * PI * 11 * r3 * 16, {-15, -15, -h4}, {15, 15, h4}, 3, 5, 3, 3},
      {"hexagon torus", BK_TORUS, {6, 30, 8}, 2 * PI * 11 * 1.5 * r3 * 16, {-15, -15, -h4}, {15, 15, h4}, 6, 10, 6, 6},
      {"wedge", BK_WEDGE, {30, 20, 10}, 3000, {-15, -10, -5}, {15, 10, 5}, 5, 9, 6, 0},
      {"pyramid", BK_PYRAMID, {4, 20, 20}, 200.0 * 20 / 3, {-10 / std::sqrt(2.0), -10 / std::sqrt(2.0), -10}, {10 / std::sqrt(2.0), 10 / std::sqrt(2.0), 10}, 5, 8, 5, 0},
      {"3-sided pyramid", BK_PYRAMID, {3, 20, 20}, 0.75 * r3 * 100 * 20 / 3, {-5 * r3, -7.5, -10}, {5 * r3, 7.5, 10}, 4, 6, 4, 0},
      {"half sphere", BK_HEMISPHERE, {20}, 2 * PI * 1000 / 3, {-10, -10, -5}, {10, 10, 5}, 2, 3, 2, 1},
      {"bowl", BK_BOWL, {40, 2}, 2 * PI * (8000 - 18 * 18 * 18) / 3, {-20, -20, -10}, {20, 20, 10}, 3, 6, 4, 2},
      {"ring", BK_RING, {30, 20, 5}, PI * (225 - 100) * 5, {-15, -15, -2.5}, {15, 15, 2.5}, 4, 6, 4, 4},
      {"ring without a hole", BK_RING, {30, 0, 5}, PI * 225 * 5, {-15, -15, -2.5}, {15, 15, 2.5}, 3, 3, 2, 2},
      {"glass", BK_GLASS, {30, 40, 2, 3}, PI * (225 * 40 - 169 * 37), {-15, -15, -20}, {15, 15, 20}, 5, 6, 4, 4},
      {"oval", BK_OVAL, {20, 14, 90, 20}, PI * 10 * 7 * 20, {-10, -7, -10}, {10, 7, 10}, 3, 3, 2, 0},
      {"slanted oval", BK_OVAL, {20, 14, 60, 20}, PI * 10 * 7 * std::sin(PI / 3) * 20, {-ox, -oy, -10}, {ox, oy, 10}, 3, 3, 2, 0},
      {"round oval", BK_OVAL, {20, 20, 90, 20}, PI * 100 * 20, {-10, -10, -10}, {10, 10, 10}, 3, 3, 2, 2},
      {"oval torus", BK_OVAL_TORUS, {0, 30, 20, 90, 6}, PI * 9 * ovalLine, {-15, -10, -3}, {15, 10, 3}, 1, 2, 1, 0},
      {"triangle oval torus", BK_OVAL_TORUS, {3, 30, 20, 90, 6}, r3 * 9 * ovalLine, {-15, -10, -h3}, {15, 10, h3}, 3, 6, 3, 0},
  };
  for (const auto &c : cases) {
    BKShape *s = bk_primitive(c.kind, c.p.data());
    if (!s) {
      check(c.name, false, bk_last_error());
      continue;
    }
    auto t0 = std::chrono::steady_clock::now();
    BKMesh *m = bk_mesh(s, 0.05);
    double took = ms(t0);
    double sv;
    std::string why;
    bool shut = closed(m, sv, why);
    bool box = true;
    for (int k = 0; k < 3; k++) box = box && near(m->bbox[k], c.lo[k], 1e-9) && near(m->bbox[3 + k], c.hi[k], 1e-9);
    // The mesh's own box lies within the exact one, short of it by no more than the chord error.
    double vlo[3] = {1e9, 1e9, 1e9}, vhi[3] = {-1e9, -1e9, -1e9};
    for (int i = 0; i < m->vertexCount; i++)
      for (int k = 0; k < 3; k++) vlo[k] = std::min(vlo[k], (double)m->positions[3 * i + k]), vhi[k] = std::max(vhi[k], (double)m->positions[3 * i + k]);
    bool inside = true;
    for (int k = 0; k < 3; k++) inside = inside && vlo[k] >= c.lo[k] - 1e-4 && vhi[k] <= c.hi[k] + 1e-4 && vlo[k] - c.lo[k] <= 0.06 && c.hi[k] - vhi[k] <= 0.06;
    bool topo = m->faceCount == c.faces && m->edgeCount == c.edges && m->cornerCount == c.corners && m->circleCount == c.circles;
    char note[400];
    snprintf(note, sizeof note, "volume %.6f (%.6f) · mesh %.2f%% · %d triangles in %.2f ms · faces %d edges %d corners %d circles %d %s", m->volume, c.volume,
             100 * (sv - c.volume) / c.volume, m->triangleCount, took, m->faceCount, m->edgeCount, m->cornerCount, m->circleCount, shut ? "" : why.c_str());
    bool vol = near(m->volume, c.volume, 1e-9 * c.volume);
        // (The mesh is in floats: its volume may come out a hair over.)
    bool meshVol = sv > 0 && sv <= c.volume * (1 + 1e-6) && sv >= c.volume * 0.96;
    check(c.name, vol && box && shut && meshVol && inside && topo, note);
    // Faces numbered on every triangle, each face's centroid and normal finite.
    bool numbered = true;
    for (int t = 0; t < m->triangleCount; t++) numbered = numbered && (int)m->triangleFace[t] < m->faceCount;
    for (int f = 0; f < 6 * m->faceCount; f++) numbered = numbered && std::isfinite(m->faceInfo[f]);
    for (int e = 0; e < m->edgeCount; e++) numbered = numbered && m->edgeFaces[2 * e] >= 0 && m->edgeFaces[2 * e] < m->faceCount && m->edgeFaces[2 * e + 1] < m->faceCount;
    if (!numbered) check((std::string(c.name) + " faces and edges numbered").c_str(), false);
    // Finer meshes close in on the exact volume.
    BKMesh *fine = bk_mesh(s, 0.002);
    double fv;
    closed(fine, fv, why);
    if (!(fv <= c.volume * (1 + 1e-6) && fv >= c.volume * (1 - 2e-3))) check((std::string(c.name) + " fine mesh").c_str(), false, fmt("%.4f of %.4f", fv, c.volume));
    bk_mesh_free(fine);
    bk_mesh_free(m);
    bk_free(s);
  }
  double z[2] = {20, 0};
  bool refused = !bk_primitive(BK_CYLINDER, z);
  std::string said = bk_last_error();
  check("a cylinder of no height is refused", refused && said.find("above zero") != std::string::npos, said);
  double fat[3] = {0, 30, 30};
  refused = !bk_primitive(BK_TORUS, fat);
  check("a tube too thick for its torus is refused", refused, bk_last_error());

  // MARK: circles and corners where they belong
  {
    double cyl[2] = {20, 20};
    BKShape *s = bk_primitive(BK_CYLINDER, cyl);
    BKMesh *m = bk_mesh(s, 0.05);
    bool ok = m->circleCount == 2;
    for (int i = 0; i < m->circleCount; i++) {
      double *c = m->circles + 7 * i;
      ok = ok && near(c[0], 0, 1e-12) && near(c[1], 0, 1e-12) && near(std::fabs(c[2]), 10, 1e-12) && near(c[5], 1, 1e-12) && near(c[6], 10, 1e-12);
    }
    check("a cylinder's rims are circles round its axis", ok);
    bk_mesh_free(m);
    bk_free(s);
  }

  // MARK: placements
  {
    double box[3] = {20, 30, 10};
    BKShape *s = bk_primitive(BK_BOX, box);
    double c = std::cos(PI / 4), sn = std::sin(PI / 4);
    double turn[12] = {c, -sn, 0, 5, sn, c, 0, -3, 0, 0, 1, 7};
    BKShape *t = bk_transform(s, turn);
    BKMesh *m = bk_mesh(t, 0.05);
    double half = (10 + 15) * c;
    check("turned and moved: same volume, the box it fills", near(m->volume, 6000, 1e-9) && near(m->bbox[0], 5 - half, 1e-9) && near(m->bbox[3], 5 + half, 1e-9) &&
                                                               near(m->bbox[2], 2, 1e-9) && near(m->bbox[5], 12, 1e-9),
          fmt("x %.6f … %.6f", m->bbox[0], m->bbox[3]));
    bk_mesh_free(m);
    double stretch[12] = {2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 3, 0};
    double cyl[2] = {20, 20};
    BKShape *k = bk_primitive(BK_CYLINDER, cyl);
    BKShape *ks = bk_transform(k, stretch);
    BKMesh *km = bk_mesh(ks, 0.05);
    double sv;
    std::string why;
    bool shut = closed(km, sv, why);
    check("stretched unevenly: volume times the stretch, no circles left, still closed", near(km->volume, PI * 100 * 20 * 6, 1e-6) && km->circleCount == 0 && shut &&
                                                                                        near(km->bbox[3], 20, 1e-9) && near(km->bbox[5], 30, 1e-9),
          fmt("%.4f", km->volume));
    double mirror[12] = {-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    BKShape *km2 = bk_transform(k, mirror);
    BKMesh *mm = bk_mesh(km2, 0.05);
    shut = closed(mm, sv, why);
    check("mirrored: still facing outwards", shut && sv > 0, fmt("%.2f", sv));
    double flat[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
    bool flatRefused = !bk_transform(k, flat);
    check("a placement that flattens is refused", flatRefused, bk_last_error());
    bk_mesh_free(mm), bk_mesh_free(km);
    bk_free(km2), bk_free(ks), bk_free(k), bk_free(t), bk_free(s);
  }

  // MARK: boxes of turned and stretched shapes (where a turned shape settles, what it lines up with)
  {
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> U(0, 1);
    auto randomPlace = [&](bool stretch) {
      double a = U(rng) * 2 * PI, b = U(rng) * PI, c = U(rng) * 2 * PI;
      double ca = std::cos(a), sa = std::sin(a), cb = std::cos(b), sb = std::sin(b), cc = std::cos(c), sc = std::sin(c);
      double R[9] = {ca * cc - sa * cb * sc, -ca * sc - sa * cb * cc, sa * sb, sa * cc + ca * cb * sc, -sa * sc + ca * cb * cc, -ca * sb, sb * sc, sb * cc, cb};
      double k[3] = {1, 1, 1};
      if (stretch)
        for (auto &v : k) v = 0.5 + 1.5 * U(rng);
      std::vector<double> m(12);
      for (int r = 0; r < 3; r++) {
        for (int col = 0; col < 3; col++) m[4 * r + col] = R[3 * r + col] * k[col];
        m[4 * r + 3] = (U(rng) - 0.5) * 40;
      }
      return m;
    };
    // The box from a far finer mesh of the placed shape: inside the true box by no more than its chord error.
    auto meshBox = [](BKShape *s, const double *m, double out[6]) {
      BKShape *t = bk_transform(s, m);
      BKMesh *f = bk_mesh(t, 0.002);
      for (int k = 0; k < 3; k++) out[k] = INFINITY, out[3 + k] = -INFINITY;
      for (int i = 0; i < f->vertexCount; i++)
        for (int k = 0; k < 3; k++) out[k] = std::min(out[k], (double)f->positions[3 * i + k]), out[3 + k] = std::max(out[3 + k], (double)f->positions[3 * i + k]);
      bk_mesh_free(f);
      bk_free(t);
    };
    int wrong = 0, tried = 0;
    double worst = 0;
    std::string where;
    for (const auto &c : cases) {
      BKShape *s = bk_primitive(c.kind, c.p.data());
      for (int it = 0; it < 6; it++) {
        auto m = randomPlace(it % 2 == 1);
        double box[6], fine[6];
        int exact = bk_bounds(s, m.data(), box);
        meshBox(s, m.data(), fine);
        tried++;
        bool ok = exact == 1;
        for (int k = 0; k < 3; k++) {
          // Floats on the fine mesh: a hair outside is rounding.
          ok = ok && fine[k] >= box[k] - 1e-4 && fine[3 + k] <= box[3 + k] + 1e-4 && fine[k] - box[k] < 0.01 && box[3 + k] - fine[3 + k] < 0.01;
          worst = std::max({worst, fine[k] - box[k], box[3 + k] - fine[3 + k]});
        }
        if (!ok) wrong++, where += std::string(c.name) + " ";
      }
      bk_free(s);
    }
    check("every shape's box, turned and stretched any way, is exact", wrong == 0, fmt("%.0f of %.0f wrong; the finer mesh within %.4f mm", wrong, tried, worst) + " " + where);
    // Merged and split shapes: never smaller than they are, and close.
    double b20[3] = {20, 20, 20}, cyl[2] = {10, 40}, sph[1] = {24}, p0[3] = {0, 0, 0}, tilt[3] = {1, 2, 3};
    double shift[12] = {1, 0, 0, 8, 0, 1, 0, 4, 0, 0, 1, 2};
    BKShape *box = bk_primitive(BK_BOX, b20), *c = bk_primitive(BK_CYLINDER, cyl), *ball = bk_primitive(BK_SPHERE, sph);
    BKShape *moved = bk_transform(ball, shift);
    BKShape *trees[4] = {bk_boolean(BK_UNION, box, moved), bk_boolean(BK_SUBTRACT, ball, c), bk_boolean(BK_INTERSECT, box, moved), bk_split(ball, p0, tilt, 0)};
    wrong = 0, tried = 0, worst = 0;
    for (BKShape *t : trees)
      for (int it = 0; it < 6; it++) {
        auto m = randomPlace(it % 3 == 2);
        double bx[6], fine[6];
        int exact = bk_bounds(t, m.data(), bx);
        meshBox(t, m.data(), fine);
        tried++;
        // Sides that can't be told exactly come from the display mesh's points (0.05 mm chords; where two curved surfaces
        // cross, the crossing's chords too).
        bool ok = exact >= 0;
        for (int k = 0; k < 3; k++) {
          ok = ok && std::fabs(fine[k] - bx[k]) < 0.1 && std::fabs(bx[3 + k] - fine[3 + k]) < 0.1;
          if (exact == 1) ok = ok && fine[k] >= bx[k] - 1e-4 && fine[3 + k] <= bx[3 + k] + 1e-4 && fine[k] - bx[k] < 0.01 && bx[3 + k] - fine[3 + k] < 0.01;
          worst = std::max({worst, std::fabs(fine[k] - bx[k]), std::fabs(bx[3 + k] - fine[3 + k])});
        }
        if (!ok) wrong++;
      }
    check("merged and split shapes' boxes, turned and stretched: exact where told, otherwise as close as their meshes", wrong == 0,
          fmt("%.0f of %.0f wrong; within %.4f mm", wrong, tried, worst));
    // Asked as a shape turns under a drag: quick.
    std::vector<double> m = randomPlace(false);
    double bx[6];
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 1000; i++) bk_bounds(box, m.data(), bx);
    double prim = ms(t0) / 1000;
    for (BKShape *t : trees) bk_bounds(t, m.data(), bx);  // their display meshes made once
    t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; i++)
      for (BKShape *t : trees) bk_bounds(t, m.data(), bx);
    double tree = ms(t0) / 400;
    printf("  a turned shape's box in %.4f ms, a merged or split one's in %.3f ms\n", prim, tree);
    check("turned shapes' boxes are quick", !timed || (prim < 0.05 && tree < 2));
    for (BKShape *t : trees) bk_free(t);
    bk_free(moved), bk_free(ball), bk_free(c), bk_free(box);
  }

  // MARK: distances (as the ruler measures)
  {
    double box[3] = {10, 10, 10}, cyl[2] = {10, 20}, sph[1] = {10};
    BKShape *b = bk_primitive(BK_BOX, box), *c = bk_primitive(BK_CYLINDER, cyl), *s = bk_primitive(BK_SPHERE, sph);
    BKMesh *bm = bk_mesh(b, 0.05), *cm = bk_mesh(c, 0.05);
    int px = -1, nx = -1, pz = -1, side = -1, top = -1, rim = -1;
    for (int f = 0; f < bm->faceCount; f++) {
      double *i = bm->faceInfo + 6 * f;
      if (i[0] > .9) px = f;
      if (i[0] < -.9) nx = f;
      if (i[2] > .9) pz = f;
    }
    for (int f = 0; f < cm->faceCount; f++) {
      double *i = cm->faceInfo + 6 * f;
      if (std::fabs(i[2]) < .1) side = f;
      if (i[2] > .9) top = f;
    }
    for (int e = 0; e < cm->edgeCount; e++)
      if (cm->edgeFaces[2 * e] == side && cm->edgeFaces[2 * e + 1] == top) rim = e;
    auto at = [](double x, double sx = 1, double y = 0, double z = 0) {
      std::vector<double> m = {sx, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z};
      return m;
    };
    double out[6], p5[3] = {5, 5, 5};
    struct D {
      const char *name;
      double got, want, tol;
    };
    auto t0 = std::chrono::steady_clock::now();
    std::vector<D> ds = {
        {"face to face", bk_distance(b, I, BK_END_FACE, px, nullptr, b, at(15).data(), BK_END_FACE, nx, nullptr, out), 5, 1e-9},
        {"corner to face", bk_distance(nullptr, nullptr, BK_END_POINT, 0, p5, b, at(15).data(), BK_END_FACE, nx, nullptr, out), 5, 1e-9},
        {"a point on the face", bk_distance(nullptr, nullptr, BK_END_POINT, 0, p5, b, I, BK_END_FACE, pz, nullptr, out), 0, 1e-9},
        {"edge to edge", bk_distance(b, I, BK_END_EDGE, 0, nullptr, b, at(0, 1, 30).data(), BK_END_EDGE, 0, nullptr, out), 30, 1e-9},
        {"stretched face to face", bk_distance(b, at(0, 2).data(), BK_END_FACE, px, nullptr, b, at(25).data(), BK_END_FACE, nx, nullptr, out), 10, 1e-9},
        {"box face to cylinder side", bk_distance(b, I, BK_END_FACE, px, nullptr, c, at(20).data(), BK_END_FACE, side, nullptr, out), 10, 1e-9},
        {"cylinder sides, turned", bk_distance(c, at(0, 1, 0.3).data(), BK_END_FACE, side, nullptr, c, at(17.3, 1, 1.1).data(), BK_END_FACE, side, nullptr, out),
         std::hypot(17.3, 0.8) - 10, 1e-9},
        {"sphere to sphere", bk_distance(s, I, BK_END_FACE, 0, nullptr, s, at(7, 1, 8, 9).data(), BK_END_FACE, 0, nullptr, out), std::sqrt(49 + 64 + 81) - 10, 1e-9},
        {"point to sphere", bk_distance(nullptr, nullptr, BK_END_POINT, 0, p5, s, I, BK_END_FACE, 0, nullptr, out), std::sqrt(75) - 5, 1e-9},
        {"rim to a face above", bk_distance(c, I, BK_END_EDGE, rim, nullptr, b, at(0, 1, 0, 20).data(), BK_END_FACE, 0, nullptr, out), -1, 0},
        {"a stretched cylinder's side (meshed finely)", bk_distance(c, at(0, 2).data(), BK_END_FACE, side, nullptr, b, at(30).data(), BK_END_FACE, nx, nullptr, out), 15, 3e-4},
    };
    double took = ms(t0);
    // The rim (z = 10, radius 5) to the box above it (its faces at z = 15 … 25): the nearest face found by name.
    {
      double best = 1e9;
      for (int f = 0; f < bm->faceCount; f++) best = std::min(best, bk_distance(c, I, BK_END_EDGE, rim, nullptr, b, at(0, 1, 0, 20).data(), BK_END_FACE, f, nullptr, out));
      ds[9].got = best, ds[9].want = 5, ds[9].tol = 1e-9;
    }
    for (auto &d : ds) check((std::string("distance: ") + d.name).c_str(), near(d.got, d.want, d.tol), fmt("%.12f (%.12f)", d.got, d.want));
    double none = bk_distance(b, I, BK_END_FACE, 99, nullptr, b, I, BK_END_FACE, 0, nullptr, out);
    check("distance: a face that isn't there", none == -1, bk_last_error());
    printf("  %zu distances in %.1f ms\n", ds.size(), took);
    bk_mesh_free(bm), bk_mesh_free(cm);
    bk_free(b), bk_free(c), bk_free(s);
  }

  // MARK: exact orientation
  {
    std::mt19937_64 rng(7);
    int wrong = 0, tested = 0;
    auto exact = [](long long a[3], long long b[3], long long c[3], long long d[3]) {
      __int128 m[3][3];
      long long *p[3] = {a, b, c};
      for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++) m[i][k] = (__int128)p[i][k] - d[k];
      __int128 det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
      return det > 0 ? 1 : det < 0 ? -1 : 0;
    };
    for (int it = 0; it < 200000; it++) {
      long long q[4][3];
      // Nearly flat on purpose: the fourth point close to the plane of the others, coordinates large and offset.
      long long base = (long long)(rng() % 1000000) - 500000;
      for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++) q[i][k] = base + (long long)(rng() % 4000001) - 2000000;
      long long u = rng() % 7, v = rng() % 7;
      for (int k = 0; k < 3; k++) q[3][k] = q[0][k] + u * (q[1][k] - q[0][k]) + v * (q[2][k] - q[0][k]) + (long long)(rng() % 3) - 1;
      // Scaled by a power of two and moved far off: still exact as doubles, the differences no longer are.
      double scale = std::ldexp(1.0, -20), shift = 1e9;
      bce::V3 p[4];
      for (int i = 0; i < 4; i++) p[i] = {q[i][0] * scale + shift, q[i][1] * scale - shift, q[i][2] * scale + shift / 3};
      int want = exact(q[0], q[1], q[2], q[3]);
      // Shifting by a non-dyadic amount changes the points themselves; only the unshifted copy is compared exactly.
      bce::V3 e[4];
      for (int i = 0; i < 4; i++) e[i] = {q[i][0] * scale, q[i][1] * scale, q[i][2] * scale};
      if (bce::orient3d(e[0], e[1], e[2], e[3]) != want) wrong++;
      int s1 = bce::orient3d(p[0], p[1], p[2], p[3]), s2 = bce::orient3d(p[1], p[0], p[2], p[3]);
      if (s1 != -s2) wrong++;
      tested++;
    }
    check("exact orientation of nearly flat tetrahedra", wrong == 0, fmt("%.0f wrong of %.0f", wrong, tested));
  }

  // MARK: exact points where meshes cross
  // A point where an edge meets a plane, kept as the two and the three it's made from: on its plane and its line exactly,
  // the same point made another way one point with it, and which side of a plane or another point it lies on as worked
  // out by hand in whole numbers (small ones, so that points fall on one another's planes and lines often).
  {
    std::mt19937_64 rng(5);
    using I3 = std::array<long long, 3>;
    auto det = [](I3 a, I3 b, I3 c, I3 d) {
      __int128 m[3][3];
      I3 p[3] = {a, b, c};
      for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++) m[i][k] = (__int128)p[i][k] - d[k];
      return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    auto sign = [](__int128 v) { return v > 0 ? 1 : v < 0 ? -1 : 0; };
    int tested = 0, off = 0, apart = 0, wrongSide = 0, wrongOrder = 0, unequal = 0;
    for (int round = 0; round < 400; round++) {
      long long range = round % 2 ? 6 : 900;
      double step = std::ldexp(1.0, -7);
      bce::ExactPoints E(step);
      std::vector<I3> at;
      auto grid = [&]() {
        I3 q{(long long)(rng() % (2 * range + 1)) - range, (long long)(rng() % (2 * range + 1)) - range, (long long)(rng() % (2 * range + 1)) - range};
        at.push_back(q);
        return E.grid({q[0] * step, q[1] * step, q[2] * step});
      };
      for (int i = 0; i < 12; i++) grid();
      auto pick = [&]() { return (uint32_t)(rng() % at.size()); };
      for (int k = 0; k < 40; k++) {
        uint32_t a = pick(), b = pick(), c = pick(), p = pick(), q = pick();
        __int128 dp = det(at[a], at[b], at[c], at[p]), dq = det(at[a], at[b], at[c], at[q]);
        if (dp == dq) continue;
        uint32_t x = E.line(p, q, {a, b, c});
        tested++;
        // On its plane, on its line (seen along each axis), and the same point from q to p and the plane's corners turned.
        if (E.orient3d(a, b, c, x) != 0) off++;
        for (int axis = 0; axis < 3; axis++)
          if (E.orient2d(axis, false, p, q, x) != 0) off++;
        if (E.lex(x, E.line(q, p, {b, c, a})) != 0) apart++;
        // Where two planes through its line meet the plane: the same point again.
        uint32_t r = pick(), s = pick();
        if (det(at[p], at[q], at[r], at[s]) != 0 && E.lex(x, E.planes({a, b, c}, {p, q, r}, {p, q, s})) != 0) apart++;
        // Its side of another plane: (dp·f(q) − dq·f(p)) / (dp − dq), f the plane's orient3d at a point.
        uint32_t u = pick(), v = pick(), w = pick();
        int want = sign(dp * det(at[u], at[v], at[w], at[q]) - dq * det(at[u], at[v], at[w], at[p])) * sign(dp - dq);
        if (E.orient3d(u, v, w, x) != want || E.orient3d(v, u, w, x) != -want) wrongSide++;
        // Its place against a grid point's, axis by axis.
        uint32_t g = pick();
        for (int i = 0; i < 3; i++) {
          int cmp = sign(dp * at[q][i] - dq * at[p][i] - (__int128)at[g][i] * (dp - dq)) * sign(dp - dq);
          if (E.compare(i, x, g) != cmp || E.compare(i, g, x) != -cmp) wrongOrder++;
        }
        // The middle of three points on the plane is on it too.
        uint32_t p2 = pick(), q2 = pick();
        __int128 dp2 = det(at[a], at[b], at[c], at[p2]), dq2 = det(at[a], at[b], at[c], at[q2]);
        if (dp2 != dq2 && E.orient3d(a, b, c, E.middle(x, E.line(p2, q2, {a, b, c}), a)) != 0) unequal++;
      }
    }
    check("exact points: on their planes and lines, one point however made, sides and order as worked out by hand",
          off + apart + wrongSide + wrongOrder + unequal == 0 && tested > 5000,
          fmt("%.0f points: %.0f off, %.0f apart, ", tested, off, apart) + fmt("%.0f wrong sides, %.0f out of order, %.0f middles off", wrongSide, wrongOrder, unequal));
  }

  // MARK: exact merging
  // Pairs of shapes (boxes, cylinders, balls, cones, bolts), half lined up on whole millimetres (faces on faces, edges
  // along edges, touching) and half turned any way: every merge, subtract and intersect closed, A ∪ B and A ∩ B adding up
  // to A and B and A − B to A less A ∩ B to the last digits, the same to the last bit when made again, and nothing the
  // exact arithmetic couldn't decide.
  {
    std::mt19937 rng(23);
    std::uniform_real_distribution<double> U(0, 1);
    auto made = [&](bool square) {
      bce::Shape s;
      std::string why;
      auto len = [&](double lo, double hi) { return square ? (double)(int)(lo + (hi - lo) * U(rng)) : lo + (hi - lo) * U(rng); };
      int kind = rng() % 5;
      if (kind == 4) {
        BKFastener f{};
        f.kind = (int)(rng() % 3) == 0 ? BK_HEX_NUT : BK_HEX, f.size = 2 + rng() % 4;
        bk_fastener_defaults(&f, 1);
        bce::fastener(f, 0.2, s, why);
      } else {
        double p[3] = {len(6, 20), len(6, 20), len(6, 20)};
        if (kind == 3) p[1] = len(0, 4);
        bce::primitive(kind == 0 ? BK_BOX : kind == 1 ? BK_CYLINDER : kind == 2 ? BK_SPHERE : BK_CONE, p, s, why);
      }
      bce::Solid m;
      bce::mesh(s, 0.1, m);
      double a = U(rng) * 2 * PI, b = U(rng) * PI, c = U(rng) * 2 * PI;
      if (square) a = (rng() % 4) * PI / 2, b = (rng() % 2) * PI, c = (rng() % 4) * PI / 2;
      double ca = std::cos(a), sa = std::sin(a), cb = std::cos(b), sb = std::sin(b), cc = std::cos(c), sc = std::sin(c);
      if (square) ca = std::round(ca), sa = std::round(sa), cb = std::round(cb), sb = std::round(sb), cc = std::round(cc), sc = std::round(sc);
      double t[3] = {(U(rng) - 0.5) * 16, (U(rng) - 0.5) * 16, (U(rng) - 0.5) * 16};
      if (square)
        for (auto &v : t) v = std::round(v);
      double r[12] = {ca * cc - sa * cb * sc, -ca * sc - sa * cb * cc, sa * sb, t[0], sa * cc + ca * cb * sc, -sa * sc + ca * cb * cc, -ca * sb, t[1], sb * sc, sb * cc, cb, t[2]};
      m.transform(bce::Affine::from(r));
      return m;
    };
    auto same = [](const bce::Solid &x, const bce::Solid &y) {
      return x.p.size() == y.p.size() && x.tri == y.tri && std::memcmp(x.p.data(), y.p.data(), sizeof(bce::V3) * x.p.size()) == 0;
    };
    bce::combineReport = {};
    int pairs = 300, open = 0, unequal = 0, differ = 0;
    double worst = 0;
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < pairs; i++) {
      bool square = i % 2 == 0;
      bce::Solid a = made(square), b = made(square);
      double va = a.meshVolume(), vb = b.meshVolume(), v[3];
      for (int op = 0; op < 3; op++) {
        bce::Solid r = bce::combine(a, b, op);
        if (!bce::shut(r)) open++;
        v[op] = r.meshVolume();
        if (i % 5 == 0 && !same(r, bce::combine(a, b, op))) differ++;
      }
      double e = std::max(std::fabs(v[0] + v[2] - va - vb), std::fabs(v[1] + v[2] - va)) / std::max(va, vb);
      worst = std::max(worst, e);
      if (e > 1e-9) unequal++;
    }
    // A shape with several taken away and several added in one merge: as the same merges one after another.
    int manyOpen = 0, manyOff = 0;
    double manyWorst = 0;
    for (int i = 0; i < 40; i++) {
      bool square = i % 2 == 0;
      bce::Solid base = made(square);
      std::vector<bce::Solid> take, add;
      for (int k = 0, n = 1 + rng() % 3; k < n; k++) take.push_back(made(square));
      for (int k = 0, n = rng() % 3; k < n; k++) add.push_back(made(square));
      bce::Solid once = bce::combine(base, take, add), steps = base;
      for (const auto &s : take) steps = bce::combine(steps, s, BK_SUBTRACT);
      for (const auto &s : add) steps = bce::combine(steps, s, BK_UNION);
      if (!bce::shut(once)) manyOpen++;
      double e = std::fabs(once.meshVolume() - steps.meshVolume()) / std::max(1.0, base.meshVolume());
      manyWorst = std::max(manyWorst, e);
      if (e > 1e-9) manyOff++;
    }
    check("exact merging: many taken away and added at once, as one after another", manyOpen == 0 && manyOff == 0,
          fmt("%.0f open, worst %.2g (%.0f off)", manyOpen, manyWorst, manyOff));
    const auto &cr = bce::combineReport;
    check("exact merging: lined up and turned any way, all closed", open == 0, fmt("%.0f open of %.0f", open, 3 * pairs));
    check("exact merging: volumes add up to the last digits", unequal == 0, fmt("worst %.2g (%.0f off)", worst, unequal));
    check("exact merging: the same to the last bit made again", differ == 0, fmt("%.0f differ", differ));
    check("exact merging: nothing left undecided", cr.misses == 0 && cr.keepsFailed == 0 && cr.segsDropped == 0 && cr.overflows == 0 && cr.open == 0 && cr.unsure == 0,
          fmt("misses %.0f, keeps failed %.0f, ", cr.misses, cr.keepsFailed) + fmt("dropped %.0f, overflows %.0f, ", cr.segsDropped, cr.overflows) +
              fmt("open %.0f, unsure %.0f", cr.open, cr.unsure));
    printf("  %ld merges in %.0f ms (%ld made again)\n", cr.calls, ms(t0), cr.again);
  }

  // MARK: merging and splitting
  {
    auto at = [](BKShape *s, double x, double y = 0, double z = 0) {
      double m[12] = {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z};
      return bk_transform(s, m);
    };
    struct Got {
      double volume, sv;
      bool shut;
      std::string why;
      int faces, edges, corners, circles, pieces;
      double lo[3], hi[3];
    };
    auto look = [&](BKShape *s, double deflection = 0.05, bool touching = false) {
      Got g{};
      BKMesh *m = bk_mesh(s, deflection);
      g.volume = m->volume, g.shut = closed(m, g.sv, g.why, touching);
      g.faces = m->faceCount, g.edges = m->edgeCount, g.corners = m->cornerCount, g.circles = m->circleCount, g.pieces = bk_piece_count(s);
      for (int k = 0; k < 3; k++) g.lo[k] = m->bbox[k], g.hi[k] = m->bbox[3 + k];
      bk_mesh_free(m);
      return g;
    };
    auto says = [](const Got &g) {
      char s[300];
      snprintf(s, sizeof s, "volume %.6f · faces %d edges %d corners %d circles %d pieces %d %s", g.volume, g.faces, g.edges, g.corners, g.circles, g.pieces,
               g.shut ? "" : g.why.c_str());
      return std::string(s);
    };
    auto boxIs = [](const Got &g, std::vector<double> lo, std::vector<double> hi) {
      bool ok = true;
      for (int k = 0; k < 3; k++) ok = ok && near(g.lo[k], lo[k], 1e-9) && near(g.hi[k], hi[k], 1e-9);
      return ok;
    };
    double b20[3] = {20, 20, 20}, hole[2] = {10, 40}, flush[2] = {10, 20}, ball[1] = {20}, small[1] = {10}, slab[3] = {10, 10, 40};
    BKShape *box = bk_primitive(BK_BOX, b20), *box2 = at(box, 10, 10, 10), *cyl = bk_primitive(BK_CYLINDER, hole);
    double p0[3] = {0, 0, 0}, nx[3] = {1, 0, 0}, ny[3] = {0, 1, 0}, nz[3] = {0, 0, 1}, tilt[3] = {1, 1, 1};
    std::vector<BKShape *> made;
    auto keep = [&](BKShape *s) { return made.push_back(s), s; };

    // Merging: two cubes overlapping corner to corner, face to face, touching, apart.
    Got g = look(keep(bk_boolean(BK_UNION, box, box2)));
    check("merge: overlapping cubes", g.shut && near(g.volume, 15000, 1e-9) && g.faces == 12 && g.edges == 30 && g.corners == 20 && g.pieces == 1 &&
                                          boxIs(g, {-10, -10, -10}, {20, 20, 20}), says(g));
    g = look(keep(bk_boolean(BK_UNION, box, keep(at(box, 10)))));
    check("merge: side by side, faces on one plane joined", g.shut && near(g.volume, 12000, 1e-9) && g.faces == 6 && g.edges == 12 && g.corners == 8, says(g));
    g = look(keep(bk_boolean(BK_UNION, box, keep(at(box, 20)))));
    check("merge: touching cubes are one piece", g.shut && near(g.volume, 16000, 1e-9) && g.faces == 6 && g.pieces == 1, says(g));
    g = look(keep(bk_boolean(BK_UNION, box, keep(at(box, 30)))));
    check("merge: cubes apart are two pieces", g.shut && near(g.volume, 16000, 1e-9) && g.pieces == 2, says(g));
    // Touching only along an edge, or at one corner: one piece all the same (held together there, as one shape).
    {
      double m[12] = {1, 0, 0, 20, 0, 1, 0, 20, 0, 0, 1, 0}, c[12] = {1, 0, 0, 20, 0, 1, 0, 20, 0, 0, 1, 20};
      BKShape *e = keep(bk_transform(box, m)), *k = keep(bk_transform(box, c));
      g = look(keep(bk_boolean(BK_UNION, box, e)));
      check("merge: cubes touching along an edge are one piece", near(g.volume, 16000, 1e-9) && g.pieces == 1, says(g));
      g = look(keep(bk_boolean(BK_UNION, box, k)));
      check("merge: cubes touching at a corner are one piece", near(g.volume, 16000, 1e-9) && g.pieces == 1, says(g));
    }
    g = look(keep(bk_boolean(BK_SUBTRACT, box, box2)));
    check("subtract: a corner taken out", g.shut && near(g.volume, 7000, 1e-9) && g.faces == 9 && g.pieces == 1 && boxIs(g, {-10, -10, -10}, {10, 10, 10}), says(g));
    g = look(keep(bk_boolean(BK_INTERSECT, box, box2)));
    check("intersect: the shared corner", g.shut && near(g.volume, 1000, 1e-9) && g.faces == 6 && boxIs(g, {0, 0, 0}, {10, 10, 10}), says(g));

    // Merges as one piece: faces of parts on one surface are one face, with no line between; parts on one axis meet point
    // for point however they were turned about it; a merge of a merge stays on its grid.
    {
      auto spun = [](BKShape *s, double a, double z, bool flip = false) {
        double c = std::cos(a), n = std::sin(a), f = flip ? -1 : 1;
        double m[12] = {c, -n * f, 0, 0, n, c * f, 0, 0, 0, 0, f, z};
        return bk_transform(s, m);
      };
      double c20[2] = {20, 10}, c20b[2] = {20, 6}, c14[2] = {14, 10}, hemi[1] = {20}, ball20[1] = {20}, tall[2] = {20, 20}, cone[3] = {20, 0, 10};
      double ring[3] = {30, 14, 10}, slab[3] = {10, 10, 20};
      BKShape *cylA = keep(bk_primitive(BK_CYLINDER, c20)), *cylB = keep(bk_primitive(BK_CYLINDER, c20b));
      g = look(keep(bk_boolean(BK_UNION, cylA, keep(at(cylB, 0, 0, 8)))));
      check("merge: cylinders stacked on one axis are one side, no line between",
            g.shut && near(g.volume, PI * 100 * 16, 1e-6) && g.faces == 3 && g.edges == 2 && g.circles == 2 && g.pieces == 1, says(g));
      g = look(keep(bk_boolean(BK_UNION, keep(spun(cylA, 0.3, 0)), keep(spun(cylB, 1.234, 8, true)))));
      check("merge: one turned about its own axis and upside down, still one side",
            g.shut && near(g.volume, PI * 100 * 16, 1e-6) && g.faces == 3 && g.edges == 2 && g.pieces == 1, says(g));
      BKShape *two = keep(bk_boolean(BK_UNION, keep(spun(cylA, 0.7, 0)), keep(at(cylB, 0, 0, 8))));
      g = look(keep(bk_boolean(BK_UNION, two, keep(spun(cylA, 2.1, -10)))));
      check("merge: a merge of a merge, three cylinders stacked, is one piece",
            g.shut && near(g.volume, PI * 100 * 26, 1e-6) && g.faces == 3 && g.edges == 2 && g.pieces == 1, says(g));
      BKShape *top = keep(at(keep(bk_primitive(BK_HEMISPHERE, hemi)), 0, 0, 5)), *bottom = keep(spun(keep(bk_primitive(BK_HEMISPHERE, hemi)), 0.4, -5, true));
      g = look(keep(bk_boolean(BK_UNION, top, bottom)));
      check("merge: two half balls are one ball, one face", g.shut && near(g.volume, 4 * PI * 1000 / 3, 1e-6) && g.faces == 1 && g.edges == 0, says(g));
      g = look(keep(bk_boolean(BK_UNION, keep(bk_primitive(BK_SPHERE, ball20)), keep(at(keep(bk_primitive(BK_CYLINDER, tall)), 0, 0, -10)))));
      check("merge: a ball on a cylinder's end, as wide (half a capsule)",
            g.shut && near(g.volume, PI * 100 * 20 + 2 * PI * 1000 / 3, 1e-6) && g.faces == 3 && g.edges == 2 && g.circles == 2, says(g));
      g = look(keep(bk_boolean(BK_UNION, cylA, keep(at(keep(bk_primitive(BK_CONE, cone)), 0, 0, 10)))));
      check("merge: a cone running on from a cylinder", g.shut && near(g.volume, PI * 100 * 10 * 4 / 3, 1e-6) && g.faces == 3 && g.edges == 2, says(g));
      BKShape *rg = keep(bk_primitive(BK_RING, ring)), *plug = keep(spun(keep(bk_primitive(BK_CYLINDER, c14)), 0.9, 0));
      g = look(keep(bk_boolean(BK_UNION, rg, plug)));
      check("merge: a ring with its hole filled is a cylinder", g.shut && near(g.volume, PI * 225 * 10, 1e-6) && g.faces == 3 && g.edges == 2 && g.circles == 2, says(g));
      g = look(keep(bk_boolean(BK_SUBTRACT, rg, plug)));
      check("subtract: the cylinder filling a ring's hole leaves the ring", g.shut && near(g.volume, PI * 176 * 10, 1e-6) && g.faces == 4 && g.edges == 4, says(g));
      g = look(keep(bk_boolean(BK_INTERSECT, rg, plug)));
      check("intersect: a ring and the cylinder filling its hole have nothing in common", g.faces == 0 && std::fabs(g.volume) < 1e-9 && g.pieces == 0, says(g));
      g = look(keep(bk_boolean(BK_UNION, box, keep(at(keep(bk_primitive(BK_BOX, slab)), 15, -5)))));
      check("merge: blocks of other sizes flush, one face per plane", g.shut && near(g.volume, 10000, 1e-9) && g.faces == 8 && g.edges == 18 && g.corners == 12, says(g));
    }

    // Holes: through, flush with both faces; a hidden hollow; a sphere cut by a face.
    BKShape *drilled = keep(bk_boolean(BK_SUBTRACT, box, cyl));
    g = look(drilled);
    {
      BKMesh *m = bk_mesh(drilled, 0.05);
      bool rims = m->circleCount == 2;
      for (int i = 0; i < m->circleCount; i++) {
        double *c = m->circles + 7 * i;
        rims = rims && near(c[0], 0, 1e-9) && near(c[1], 0, 1e-9) && near(std::fabs(c[2]), 10, 1e-9) && near(std::fabs(c[5]), 1, 1e-9) && near(c[6], 5, 1e-9);
      }
      check("subtract: a hole through, its rims circles of its radius", g.shut && near(g.volume, 8000 - PI * 25 * 20, 1e-9 * 8000) && g.faces == 7 && g.edges == 14 &&
                                                                              g.corners == 8 && rims && boxIs(g, {-10, -10, -10}, {10, 10, 10}), says(g));
      bk_mesh_free(m);
    }
    g = look(keep(bk_boolean(BK_SUBTRACT, box, keep(bk_primitive(BK_CYLINDER, flush)))));
    check("subtract: a hole flush with both faces leaves no skin", g.shut && near(g.volume, 8000 - PI * 25 * 20, 1e-9 * 8000) && g.faces == 7 && g.circles == 2, says(g));
    g = look(keep(bk_boolean(BK_SUBTRACT, box, keep(bk_primitive(BK_SPHERE, small)))));
    check("subtract: a hollow inside is still one piece", g.shut && near(g.volume, 8000 - 4 * PI * 125 / 3, 1e-9 * 8000) && g.pieces == 1 && g.faces == 7, says(g));
    BKShape *dome = keep(bk_boolean(BK_INTERSECT, keep(bk_primitive(BK_SPHERE, ball)), keep(at(box, 10))));
    g = look(dome);
    check("intersect: a sphere cut by a face is half a sphere, its rim a circle", g.shut && near(g.volume, 2 * PI * 1000 / 3, 1e-6 * 2094) && g.faces == 2 && g.circles == 1 &&
                                                                                       boxIs(g, {0, -10, -10}, {10, 10, 10}), says(g));
    // A merge, then a subtract: a 30 × 20 × 20 block with a 10 × 10 slot through it.
    g = look(keep(bk_boolean(BK_SUBTRACT, keep(bk_boolean(BK_UNION, box, keep(at(box, 10)))), keep(at(keep(bk_primitive(BK_BOX, slab)), 5)))));
    check("merge, then subtract: one more level down the tree", g.shut && near(g.volume, 12000 - 2000, 1e-9) && g.pieces == 1, says(g));
    // Two equal cylinders crossing at right angles (their sides touch where they cross): 16 r³ / 3.
    {
      double c[2] = {20, 60}, turn[12] = {1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0};
      BKShape *a = keep(bk_primitive(BK_CYLINDER, c)), *b = keep(bk_transform(a, turn));
      bool ok = true;
      std::string note;
      for (double d : {0.2, 0.1, 0.05, 0.02}) {
        g = look(keep(bk_boolean(BK_INTERSECT, a, b)), d);
        ok = ok && g.shut && near(g.volume, 16000.0 / 3, 2e-4 * 5333);
        note += fmt("%.4f ", g.volume);
      }
      check("intersect: crossing cylinders (sides touching), at every detail", ok, note);
    }

    // Splitting: straight, tilted, across a cylinder, through a ring's hole, beside the shape.
    g = look(keep(bk_split(box, p0, nz, 0)));
    Got other = look(keep(bk_split(box, p0, nz, 1)));
    check("split: a cube in two halves", g.shut && other.shut && near(g.volume, 4000, 1e-9) && near(other.volume, 4000, 1e-9) && g.faces == 6 &&
                                            boxIs(g, {-10, -10, 0}, {10, 10, 10}) && boxIs(other, {-10, -10, -10}, {10, 10, 0}),
          says(g));
    g = look(keep(bk_split(box, p0, tilt, 0)));
    other = look(keep(bk_split(box, p0, tilt, 1)));
    check("split: tilted through the middle, a six-sided cut", g.shut && near(g.volume, 4000, 1e-9) && near(other.volume, 4000, 1e-9) && g.faces == 7 && g.corners == 10,
          says(g));
    double c20[2] = {20, 20};
    BKShape *round = keep(bk_primitive(BK_CYLINDER, c20));
    g = look(keep(bk_split(round, p0, nz, 0)));
    check("split: a cylinder across its axis, the cut's rim a circle", g.shut && near(g.volume, PI * 100 * 10, 1e-9 * 3142) && g.faces == 3 && g.circles == 2, says(g));
    double ringSize[3] = {30, 20, 5};
    BKShape *ring = keep(bk_primitive(BK_RING, ringSize));
    g = look(keep(bk_split(ring, p0, nx, 0)));
    other = look(keep(bk_split(ring, p0, nx, 1)));
    double half = PI * (225 - 100) * 5 / 2;
    check("split: a ring through its hole, two separate cut faces", g.shut && other.shut && near(g.volume, half, 1e-4 * half) && near(g.volume + other.volume, 2 * half, 1e-9 * half) &&
                                                                      g.faces == 6 && g.pieces == 1 && boxIs(g, {0, -15, -2.5}, {15, 15, 2.5}),
          says(g));
    double far[3] = {50, 0, 0};
    BKShape *beside = keep(bk_split(box, far, nx, 0));
    g = look(beside);
    check("split: beside the shape leaves nothing", g.pieces == 0 && g.volume == 0 && g.faces == 0, says(g));

    // Splitting again and again: a split piece split, a merge split, a drilled block split, each time on what's left.
    {
      BKShape *s = box;
      double planes[5][6] = {{-5, 0, 0, 1, 0, 0}, {5, 0, 0, -1, 0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 3, 0, 0, -1}, {0, 7, 0, 0, -1, 0}};
      bool ok = true;
      std::string note;
      for (auto &q : planes) {
        s = keep(bk_split(s, q, q + 3, 0));
        g = look(s);
        ok = ok && g.shut && g.pieces == 1 && g.faces == 6;
        note += fmt("%.1f ", g.volume);
      }
      check("split five times over, each piece split again", ok && near(g.volume, 10 * 7 * 13, 1e-9) && boxIs(g, {-5, 0, -10}, {5, 7, 3}), note);
    }
    g = look(keep(bk_split(keep(bk_boolean(BK_UNION, box, box2)), p0, tilt, 0)));
    other = look(keep(bk_split(keep(bk_boolean(BK_UNION, box, box2)), p0, tilt, 1)));
    check("split: a merged shape, both sides adding up", g.shut && other.shut && near(g.volume + other.volume, 15000, 1e-9 * 15000), says(g));
    {
      BKShape *top = keep(bk_split(drilled, p0, nz, 0)), *quarter = keep(bk_split(top, p0, nx, 0)), *other2 = keep(bk_split(top, p0, nx, 1));
      Got t = look(top), q = look(quarter), o = look(other2);
      double whole = 8000 - PI * 25 * 20;
      check("split: a drilled block, then its half again", t.shut && q.shut && o.shut && near(t.volume, whole / 2, 1e-9 * whole) && near(q.volume, whole / 4, 1e-6 * whole) &&
                                                            near(q.volume + o.volume, t.volume, 1e-9 * whole) && q.circles == 2,
            says(q));
    }
    {
      // A split sphere split across, and across again: an eighth of the sphere.
      BKShape *s = keep(bk_primitive(BK_SPHERE, ball));
      BKShape *e = keep(bk_split(keep(bk_split(keep(bk_split(s, p0, nz, 0)), p0, nx, 0)), p0, ny, 0));
      g = look(e);
      double eighth = 4 * PI * 1000 / 3 / 8;
      check("split: a sphere split three times, an eighth left", g.shut && near(g.volume, eighth, 1e-6 * eighth) && g.faces == 4 && boxIs(g, {0, 0, 0}, {10, 10, 10}),
            says(g));
    }

    // Measuring on what's left: the drilled hole's side and the split cylinder's cut face, exactly.
    {
      BKMesh *m = bk_mesh(drilled, 0.05);
      double out[6], q[3] = {0, 0, 0}, best = 1e9;
      for (int f = 0; f < m->faceCount; f++) best = std::min(best, bk_distance(nullptr, nullptr, BK_END_POINT, 0, q, drilled, I, BK_END_FACE, f, nullptr, out));
      check("distance: a point on the axis to the drilled hole's side", near(best, 5, 1e-9), fmt("%.12f", best));
      bk_mesh_free(m);
      BKShape *cut = keep(bk_split(round, p0, nz, 1));
      m = bk_mesh(cut, 0.05);
      double above[3] = {3, 4, 100};
      best = 1e9;
      for (int f = 0; f < m->faceCount; f++) best = std::min(best, bk_distance(nullptr, nullptr, BK_END_POINT, 0, above, cut, I, BK_END_FACE, f, nullptr, out));
      check("distance: a point above to a split cylinder's cut", near(best, 100, 1e-9), fmt("%.12f", best));
      bk_mesh_free(m);
    }

    // Many shapes at random, each pair merged, subtracted and intersected: always closed and facing out, and the volumes
    // agreeing with one another (A ∪ B and A ∩ B add up to A and B; A − B is A less A ∩ B) and with a far finer mesh.
    {
      std::mt19937 rng(11);
      std::uniform_real_distribution<double> U(0, 1);
      auto shape = [&]() -> BKShape * {
        double s = 8 + 14 * U(rng);
        switch (rng() % 6) {
          case 0: {
            double p[3] = {s, s * (0.6 + U(rng)), s * (0.6 + U(rng))};
            return bk_primitive(BK_BOX, p);
          }
          case 1: {
            double p[2] = {s, s};
            return bk_primitive(BK_CYLINDER, p);
          }
          case 2: {
            double p[1] = {s};
            return bk_primitive(BK_SPHERE, p);
          }
          case 3: {
            double p[3] = {s, s * 0.3, s};
            return bk_primitive(BK_CONE, p);
          }
          case 4: {
            double p[3] = {0, s * 1.5, s * 0.4};
            return bk_primitive(BK_TORUS, p);
          }
          default: {
            double p[3] = {(double)(3 + rng() % 5), s, s};
            return bk_primitive(BK_PRISM, p);
          }
        }
      };
      // Half turned any way, half lined up on whole millimetres (faces lying on faces, edges along edges).
      auto place = [&](BKShape *s, bool square) {
        double a = U(rng) * 2 * PI, b = U(rng) * PI, c = U(rng) * 2 * PI;
        if (square) a = (rng() % 4) * PI / 2, b = (rng() % 2) * PI / 2, c = 0;
        double ca = std::cos(a), sa = std::sin(a), cb = std::cos(b), sb = std::sin(b), cc = std::cos(c), sc = std::sin(c);
        double t[3] = {(U(rng) - 0.5) * 16, (U(rng) - 0.5) * 16, (U(rng) - 0.5) * 16};
        if (square)
          for (auto &v : t) v = std::round(v);
        double m[12] = {ca * cc - sa * cb * sc, -ca * sc - sa * cb * cc, sa * sb, t[0], sa * cc + ca * cb * sc, -sa * sc + ca * cb * cc, -ca * sb, t[1], sb * sc, sb * cc, cb, t[2]};
        BKShape *r = bk_transform(s, m);
        bk_free(s);
        return r;
      };
      int pairs = 120, open = 0, unequal = 0, off = 0;
      double worst = 0, worstFine = 0, took = 0, slowest = 0;
      for (int i = 0; i < pairs; i++) {
        bool square = i % 2 == 1;
        BKShape *a = place(shape(), square), *b = place(shape(), square);
        double va = look(a).volume, vb = look(b).volume, v[3];
        for (int op = 0; op < 3; op++) {
          auto t0 = std::chrono::steady_clock::now();
          BKShape *r = bk_boolean(op, a, b);
          // Lined-up shapes may touch along a line (a torus resting on a face): closed all the same.
          Got q = look(r, 0.05, square);
          double t = ms(t0);
          took += t, slowest = std::max(slowest, t);
          if (!q.shut) open++;
          v[op] = q.volume;
          // Against a far finer mesh's.
          if (i % 4 == 0) {
            Got fine = look(r, 0.003, square);
            double e = std::fabs(q.volume - fine.volume) / std::min(va, vb);
            worstFine = std::max(worstFine, e);
            if (e > 1e-3) off++;
          }
          bk_free(r);
        }
        double e1 = std::fabs(v[0] + v[2] - va - vb) / std::min(va, vb), e2 = std::fabs(v[1] - (va - v[2])) / std::min(va, vb);
        worst = std::max({worst, e1, e2});
        if (e1 > 1e-6 || e2 > 1e-6) unequal++;
        bk_free(a), bk_free(b);
      }
      check("random merges, subtracts and intersects: all closed", open == 0, fmt("%.0f open of %.0f", open, 3 * pairs));
      check("random: volumes add up", unequal == 0, fmt("worst %.2g (%.0f off)", worst, unequal));
      check("random: volumes as a far finer mesh's", off == 0, fmt("worst %.2g of the smaller shape", worstFine));
      printf("  %d merges, subtracts and intersects in %.2f ms each (slowest %.1f ms)\n", 3 * pairs, took / (3 * pairs), slowest);
    }

    // Speed: the same merge again is kept (no work), a new one is quick.
    {
      double sph[1] = {40};
      BKShape *s = keep(bk_primitive(BK_SPHERE, sph)), *u = keep(bk_boolean(BK_UNION, s, keep(at(s, 25, 5, 3))));
      auto t0 = std::chrono::steady_clock::now();
      BKMesh *m = bk_mesh(u, 0.05);
      double first = ms(t0);
      int tris = m->triangleCount;
      bk_mesh_free(m);
      t0 = std::chrono::steady_clock::now();
      m = bk_mesh(u, 0.05);
      double again = ms(t0);
      bk_mesh_free(m);
      t0 = std::chrono::steady_clock::now();
      BKShape *sp = keep(bk_split(u, p0, tilt, 0));
      m = bk_mesh(sp, 0.05);
      double split = ms(t0);
      bk_mesh_free(m);
      printf("  two 40 mm spheres merged in %.2f ms (%d triangles), again in %.3f ms; that split in %.2f ms\n", first, tris, again, split);
      check("merging and splitting are quick", !timed || (first < 50 && again < first / 5 && split < 50));
    }
    for (BKShape *s : made) bk_free(s);
    bk_free(box), bk_free(box2), bk_free(cyl);
  }

  // MARK: edges and hollows
  // Sections across edges; roundings, inward roundings and bevels against exact volumes (and reference figures where
  // a corner is a matter of convention), closed and as many faces; hollows with openings and walls of their own.
  {
    double b20[3] = {20, 20, 20}, body[6] = {0}, mr;
    int miss, ke = BK_PICK_EDGE, kf = BK_PICK_FACE, kb = BK_PICK_BODY, kc = BK_PICK_CORNER;
    BKShape *box = bk_primitive(BK_BOX, b20);
    std::vector<BKShape *> made;
    auto keep = [&](BKShape *s) { return made.push_back(s), s; };
    auto at = [](BKShape *s, double x, double y, double z) {
      double m[12] = {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z};
      return bk_transform(s, m);
    };
    struct Got {
      double volume = -1;
      int faces = 0;
      bool shut = false;
      std::string why;
    };
    auto look = [&](BKShape *s, double deflection = 0.05) {
      Got g;
      if (!s) return g.why = bk_last_error(), g;
      BKMesh *m = bk_mesh(s, deflection);
      double sv;
      g.volume = m->volume, g.faces = m->faceCount, g.shut = closed(m, sv, g.why) && bk_piece_count(s) == 1;
      bk_mesh_free(m);
      return g;
    };
    auto says = [](const Got &g) { return fmt("volume %.4f · faces %.0f ", g.volume, g.faces) + g.why; };
    auto is = [&](const char *name, BKShape *s, double want, double tol, int faces = -1) {
      Got g = look(keep(s));
      check(name, g.shut && near(g.volume, want, tol) && (faces < 0 || g.faces == faces), says(g) + fmt(" (want %.4f)", want));
    };
    double edge[6] = {0, -10, 10, 1, 0, 0}, top[6] = {0, 0, 1, 0, 0, 10}, bottom[6] = {0, 0, -1, 0, 0, -10}, corner[6] = {0, 0, 1, 10, 10, 10};
    auto t0 = std::chrono::steady_clock::now();

    BKSection *sec = bk_section(box, ke, edge, 10);
    check("section: across a box's edge, 90° and one outline", sec && near(sec->angle, 90, 1e-9) && sec->loopCount == 1, sec ? fmt("%.4f° · %.0f loops", sec->angle, sec->loopCount) : bk_last_error());
    if (sec) bk_section_free(sec);
    // The edges picks stand for, one edge pick each: a box's every edge, its top's four, one edge itself.
    {
      double got[12 * 6];
      int all = bk_pick_edges(box, &kb, body, 1, got, 12), fourTop = bk_pick_edges(box, &kf, top, 1, nullptr, 0), one = bk_pick_edges(box, &ke, edge, 1, got, 1);
      check("pick edges: a box's 12, its top's 4, one edge's 1 (at its middle)", all == 12 && fourTop == 4 && one == 1 && near(got[1], -10, 1e-9) && near(got[2], 10, 1e-9) && near(got[0], 0, 1e-9),
            fmt("%.0f, %.0f, %.0f", all, fourTop, one));
    }

    double one = 8000 - (4 - PI) * 20;
    is("round: one edge", bk_fillet(box, &ke, edge, 1, 2, &mr, &miss), one, 1e-6, 7);
    is("round: a corner pick takes its edge", bk_fillet(box, &kc, corner, 1, 2, &mr, &miss), one, 1e-6, 7);
    // The top face's edges: each edge's rounding, less where two meet at a corner (the cylinders' common part counted once).
    is("round: a face's edges", bk_fillet(box, &kf, top, 1, 2, &mr, &miss), 8000 - 4 * (4 - PI) * 20 + 4 * (16 - 8.0 / 3 - 4 * PI), 0.01, 10);
    is("round: every edge (balls at the corners)", bk_fillet(box, &kb, body, 1, 2, &mr, &miss), 4096 + 3072 + 192 * PI + 32 * PI / 3, 1e-6, 26);
    double w20[3] = {20, 20, 20};
    {
      // A rounded polyhedron is its corners' inner one grown by the radius (Steiner): inner volume, area, edges' turns, ball.
      double rho = 20 * (2 - std::sqrt(2.0)) / 2, k = (rho - 1) / rho, leg = 20 * k, hyp = leg * std::sqrt(2.0);
      double inner = leg * leg / 2 * 18, area = leg * leg + (2 * leg + hyp) * 18, turns = 18 * 2 * PI + 2 * (2 * leg + hyp) * PI / 2;
      is("round: every edge of a wedge", bk_fillet(keep(bk_primitive(BK_WEDGE, w20)), &kb, body, 1, 1, &mr, &miss), inner + area + turns / 2 + 4 * PI / 3, 1e-4, 20);
    }
    BKShape *big = bk_fillet(box, &ke, edge, 1, 40, &mr, &miss);
    check("round: too large is refused, saying the most that fits", !big && mr > 19 && mr < 20, fmt("most %.2f · ", mr) + bk_last_error());
    double cy[2] = {20, 20}, hc[2] = {8, 30};
    BKShape *cyl = keep(bk_primitive(BK_CYLINDER, cy)), *holed = keep(bk_boolean(BK_SUBTRACT, box, keep(bk_primitive(BK_CYLINDER, hc))));
    double rim[6] = {10, 0, 10, 0, 1, 0}, hrim[6] = {4, 0, 10, 0, 1, 0};
    {
      // A rim's rounding by Pappus: the section's area turned round the axis at its centroid.
      double A = 4 * (1 - PI / 4), rc = (4 * 9 - PI * (8 + 8 / (3 * PI))) / A;
      is("round: a cylinder's rim, in step with its circle", bk_fillet(cyl, &ke, rim, 1, 2, &mr, &miss), PI * 100 * 20 - 2 * PI * rc * A, 1e-3, 4);
      double a = 1 - PI / 4, rh = (4.5 - PI / 4 * (5 - 4 / (3 * PI))) / a;
      is("round: a hole's rim", bk_fillet(holed, &ke, hrim, 1, 1, &mr, &miss), 8000 - PI * 16 * 20 - 2 * PI * rh * a, 1e-3, 8);
    }
    BKShape *L = keep(bk_boolean(BK_UNION, box, keep(at(box, 20, 0, -10))));
    double inside[6] = {10, 0, 0, 0, 1, 0};
    is("round: an inside corner is filled", bk_fillet(L, &ke, inside, 1, 2, &mr, &miss), 16000 + (4 - PI) * 20, 1e-6, 11);
    // An inside corner rounded wider than the narrow face beside it is across (c1044): what it fills past that face is cut
    // off by the face beyond (reference 3466.8184 mm³).
    {
      double box1044[3] = {8.17306, 9.76692, 22.3589}, five1044[3] = {5, 12.2671, 20.3772}, edge1044[6] = {3.74027, 4.88346, 7.8643, -1, 0, 0};
      BKShape *joined = keep(bk_boolean(BK_UNION, keep(bk_primitive(BK_BOX, box1044)), keep(at(keep(bk_primitive(BK_PRISM, five1044)), 7.92584, 2.62818, -2.3243))));
      is("round: an inside corner wider than the narrow face beside it", bk_fillet(joined, &ke, edge1044, 1, 0.419373, &mr, &miss), 3466.8184, 1e-3, 14);
    }
    is("cove: one edge", bk_cove(box, &ke, edge, 1, 3, &mr, &miss), 8000 - PI * 9 / 4 * 20, 0.01, 7);
    // Every edge: each cylinder's quarter, less where two (Steinmetz) and three (the tricylinder) meet at a corner.
    for (double r : {1.0, 3.0})
      is(r == 1 ? "cove: every edge, 1 mm" : "cove: every edge, 3 mm", bk_cove(box, &kb, body, 1, r, &mr, &miss),
         8000 - 12 * r * r * PI / 4 * 20 + 8 * (3 * 2.0 / 3 - (2 - std::sqrt(2.0))) * r * r * r, 0.5, 18);
    // Every edge of a five-sided prism at 0.5 mm, made or refused, in moments (once, tools going on being merged into a
    // mesh already broken took 46 s to refuse it).
    {
      double five[3] = {5, 14.0654, 21.614};
      auto t1 = std::chrono::steady_clock::now();
      BKShape *coved = keep(bk_cove(keep(bk_primitive(BK_PRISM, five)), &kb, body, 1, 0.5, &mr, &miss));
      double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
      check("cove: every edge of a prism, made or refused in moments", !timed || took < 5, fmt("%.1f s, ", took) + (coved ? "made" : bk_last_error()));
      // Every edge of a three-sided prism and of a wedge: where three meet at a corner two cylinders only touch, which
      // merging takes well only in some orders (reference 261.0555 and 2656.6523 mm³).
      double three[3] = {3, 11.6451, 6.32203}, wedge406[3] = {20.7767, 15.8337, 16.3498};
      is("cove: every edge of a three-sided prism", bk_cove(keep(bk_primitive(BK_PRISM, three)), &kb, body, 1, 0.579835, &mr, &miss), 261.0555, 0.01, 14);
      is("cove: every edge of a wedge", bk_cove(keep(bk_primitive(BK_WEDGE, wedge406)), &kb, body, 1, 0.525139, &mr, &miss), 2656.6523, 0.01, 14);
    }
    is("bevel: 2 × 4 mm", bk_chamfer(box, &ke, edge, 1, 2, 4, 0, &miss), 8000 - 80, 1e-6, 7);
    // Softened: each of the bevel's edges (135°) rounded, ρ²(cot(φ/2) − (π − φ)/2) per edge.
    is("bevel: softened", bk_chamfer(box, &ke, edge, 1, 2, 2, 0.5, &miss), 8000 - 40 - 2 * 0.25 * (1 / std::tan(3 * PI / 8) - PI / 8) * 20, 1e-3, 9);
    // Every edge: three bevels at each corner cut it off flat (a tetrahedron of 2/3 mm³ more).
    is("bevel: every edge, the corners cut off flat", bk_chamfer(box, &kb, body, 1, 2, 2, 0, &miss), 8000 - 12 * 40 + 8 * (8 - 2) - 8 * 2.0 / 3, 1e-6, 26);
    {
      // A leg longer than the 1 mm plate is thick: the bevel runs on past the front, cutting into the bottom (exact: 7.5 mm³
      // off). Two bevels whose legs on the front add up to more than it: their flats meet in a ridge (exact: 6.8108 off).
      double plate1[3] = {20, 20, 1}, top1[6] = {0, -10, 0.5, 1, 0, 0}, both1[12] = {0, -10, 0.5, 1, 0, 0, 0, -10, -0.5, 1, 0, 0};
      int ke2[2] = {ke, ke};
      BKShape *thin = keep(bk_primitive(BK_BOX, plate1));
      is("bevel: a leg past the face beside it, running on", bk_chamfer(thin, &ke, top1, 1, 0.5, 2, 0, &miss), 392.5, 1e-6, 6);
      is("bevel: two across a narrow face, meeting in a ridge", bk_chamfer(thin, ke2, both1, 2, 0.5, 0.7, 0, &miss), 393.189189, 1e-5, 7);
      // Every edge of a block with a slot (c578): the fills of its inside corners merge only in some orders (reference
      // 4928.0541 mm³; inside corners are finished a little differently).
      double block578[3] = {14.5673, 23.7556, 16.9797}, slot578[3] = {7.50662, 18.9013, 16.2504};
      BKShape *slotted = keep(bk_boolean(BK_SUBTRACT, keep(bk_primitive(BK_BOX, block578)), keep(at(keep(bk_primitive(BK_BOX, slot578)), -0.519449, -6.57891, 8.40813))));
      is("bevel: every edge of a block with a slot", bk_chamfer(slotted, &kb, body, 1, 0.744838, 0.494177, 0, &miss), 4928.0541, 0.6);
    }

    // Runs of edges meeting smoothly: a box's sides rounded, then its top.
    double ups[24];
    int k4[4] = {ke, ke, ke, ke};
    double cs[4][2] = {{-10, -10}, {-10, 10}, {10, -10}, {10, 10}};
    for (int i = 0; i < 4; i++) {
      double q[6] = {cs[i][0], cs[i][1], 0, 0, 0, 1};
      for (int j = 0; j < 6; j++) ups[6 * i + j] = q[j];
    }
    BKShape *up2 = keep(bk_fillet(box, k4, ups, 4, 2, &mr, &miss)), *up1 = keep(bk_fillet(box, k4, ups, 4, 1, &mr, &miss));
    {
      // 1 mm round a top whose corners are rounded 2 mm: straight parts, and at each corner the section turned a quarter.
      double sec1 = 1.5 - PI / 4 * (1 + 4 / (3 * PI));
      is("round: a top round its rounded corners", bk_fillet(up2, &kf, top, 1, 1, &mr, &miss), 8000 - 4 * (4 - PI) * 20 - 4 * (1 - PI / 4) * 16 - 4 * PI / 2 * sec1, 0.1, 18);
    }
    // As wide as the corners: a ball's eighth there.
    is("round: a top as wide as its rounded corners", bk_fillet(up2, &kf, top, 1, 2, &mr, &miss), 8000 - 4 * (4 - PI) * 20 - 4 * (4 - PI) * 16 - 4 * (PI * 4 / 4 * 2 - 4 * PI / 3 * 8 / 8), 0.5);
    // Wider than the corners (reference 7917.562 mm³; the section's circle cut off at the corner's axis here).
    is("round: a top wider than its rounded corners", bk_fillet(up1, &kf, top, 1, 2, &mr, &miss), 7917.562, 1);
    printf("  sections, roundings, coves and bevels in %.0f ms\n", ms(t0));

    t0 = std::chrono::steady_clock::now();
    auto hollow = [&](BKShape *s, const double *open, int openCount, const double *walls, const double *thick, int wallCount, double t, const BKShape *sharp = nullptr) {
      const BKShape *sh[1] = {sharp};
      return bk_hollow(s, sharp ? sh : nullptr, sharp ? 1 : 0, open, openCount, walls, thick, wallCount, t, &miss);
    };
    double five = 5, four = 4, side[6] = {1, 0, 0, 10, 0, 0};
    is("hollow: shut", hollow(box, nullptr, 0, nullptr, nullptr, 0, 2), 8000 - 4096, 1e-6, 12);
    is("hollow: the top open", hollow(box, top, 1, nullptr, nullptr, 0, 2), 8000 - 16 * 16 * 18, 1e-6, 11);
    is("hollow: the top open, a 5 mm bottom", hollow(box, top, 1, bottom, &five, 1, 2), 8000 - 16 * 16 * 15, 1e-6, 11);
    is("hollow: a cylinder open at the top", hollow(cyl, top, 1, nullptr, nullptr, 0, 1.5), PI * (100 * 20 - 8.5 * 8.5 * 18.5), 1e-3, 5);
    double s20[1] = {20};
    is("hollow: a ball", hollow(keep(bk_primitive(BK_SPHERE, s20)), nullptr, 0, nullptr, nullptr, 0, 2), 4 * PI / 3 * (1000 - 512), 1e-3, 2);
    // Merged shapes hollowed as one: two boxes side by side, no wall left between them; a cylinder standing on a box, open at
    // its top (its bore runs down through the box's top wall into the box's void: 1408 + 29π); a box's face hidden in
    // another (as c299: inside the other's top wall, its void mustn't reach into that wall; exact 4851.178).
    {
      double half[3] = {10, 20, 20}, plate[3] = {20, 20, 10}, post[2] = {10, 10}, postTop[6] = {0, 0, 1, 0, 0, 10};
      BKShape *pair = keep(bk_boolean(BK_UNION, keep(at(keep(bk_primitive(BK_BOX, half)), -5, 0, 0)), keep(at(keep(bk_primitive(BK_BOX, half)), 5, 0, 0))));
      is("hollow: two boxes side by side, one void", hollow(pair, nullptr, 0, nullptr, nullptr, 0, 2), 8000 - 4096, 1e-6, 12);
      BKShape *standing = keep(bk_boolean(BK_UNION, keep(bk_primitive(BK_BOX, plate)), keep(at(keep(bk_primitive(BK_CYLINDER, post)), 0, 0, 5))));
      is("hollow: a cylinder standing on a box, open at its top", hollow(standing, postTop, 1, nullptr, nullptr, 0, 1), 1408 + 29 * PI, 1e-3);
      double a299[3] = {24.6375, 22.7407, 20.0771}, b299[3] = {6.53949, 13.7818, 20.0672};
      BKShape *poked = keep(bk_boolean(BK_UNION, keep(bk_primitive(BK_BOX, a299)), keep(at(keep(bk_primitive(BK_BOX, b299)), -2.5173, -0.958031, -2.00733))));
      is("hollow: a box's face hidden near another's top wall keeps that wall whole", hollow(poked, nullptr, 0, nullptr, nullptr, 0, 1.85386), 4851.178, 0.01, 22);
    }
    // A ball's one face is its whole surface: opened, nothing would be left, so it stays shut (a round torus's alike).
    double ballFace[6] = {0, 0, 1, 0, 0, 10}, ring[3] = {0, 30, 8}, ringFace[6] = {0, 0, 1, 11, 0, 4};
    is("hollow: a ball's face picked to open stays shut", hollow(keep(bk_primitive(BK_SPHERE, s20)), ballFace, 1, nullptr, nullptr, 0, 2), 4 * PI / 3 * (1000 - 512), 1e-3, 2);
    is("hollow: a round torus's face picked to open stays shut", hollow(keep(bk_primitive(BK_TORUS, ring)), ringFace, 1, nullptr, nullptr, 0, 1),
       2 * PI * PI * 11 * (16 - 9), 0.05, 2);
    // A glass whose side is thinner than two walls: hollowed only in its base, its side left solid (as the walls ask).
    {
      double glass[4] = {28.2715, 21.3845, 2.67956, 4}, r = 28.2715 / 2, h = 21.3845, w = 2.67956, b = 4, t = 1.6;
      double whole = PI * r * r * b + PI * (r * r - (r - w) * (r - w)) * (h - b);
      is("hollow: a glass thinner at its side than two walls, hollowed in its base", hollow(keep(bk_primitive(BK_GLASS, glass)), nullptr, 0, nullptr, nullptr, 0, t),
         whole - PI * (r - t) * (r - t) * (b - 2 * t), 1e-3);
      double thin[4] = {28.2715, 21.3845, 2.67956, 2.5};
      BKShape *none = hollow(keep(bk_primitive(BK_GLASS, thin)), nullptr, 0, nullptr, nullptr, 0, t);
      check("hollow: a glass thinner everywhere than two walls is refused", !none, none ? "made" : bk_last_error());
      if (none) bk_free(none);
    }
    // A turn given to six digits (its columns a hair off square) is taken as the turn it stands for: a cone stays a cone,
    // its volume exact.
    {
      double cone[3] = {12.3279, 0, 10.0594};
      double m[12] = {0.141008, -0.409537, 0.90133, -2.88387, -0.657371, 0.642019, 0.394556, -1.33004, -0.740256, -0.648144, -0.178688, 1.30816};
      is("a cone turned by a turn given to six digits keeps its exact volume", bk_transform(keep(bk_primitive(BK_CONE, cone)), m), PI / 3 * 6.16395 * 6.16395 * 10.0594, 1e-3, 2);
    }
    BKShape *thick = hollow(box, top, 1, nullptr, nullptr, 0, 12);
    check("hollow: walls too thick are refused", !thick, thick ? "made" : bk_last_error());
    BKShape *roundAll = keep(bk_fillet(box, &kb, body, 1, 2, &mr, &miss));
    double roundVolume = 4096 + 3072 + 192 * PI + 32 * PI / 3;
    is("hollow: a rounded cube (walls as thick as its rounding: a sharp void)", hollow(roundAll, nullptr, 0, nullptr, nullptr, 0, 2), roundVolume - 4096, 1e-3);
    is("hollow: a rounded cube, its top open", hollow(roundAll, top, 1, nullptr, nullptr, 0, 2), roundVolume - 16 * 16 * 18, 1e-3);
    is("hollow: a rounded cube, a thicker wall of its own", hollow(roundAll, nullptr, 0, side, &four, 1, 2), roundVolume - 14 * 16 * 16, 1e-3);
    {
      // Thinner walls than the rounding: the void rounded by the rest (round the same middles).
      double rho = 1, inner = 18;
      double v = inner * inner * inner - (12 * (1 - PI / 4) * rho * rho * (inner - 2 * rho) + 8 * (1 - PI / 6) * rho * rho * rho);
      is("hollow: a rounded cube, walls thinner than its rounding", hollow(roundAll, nullptr, 0, nullptr, nullptr, 0, 1), roundVolume - v, 0.05);
    }
    // Reference 3309.562 mm³: the cube rounded 1 mm up its sides, 2 mm round its top, its top open.
    is("hollow: sides and top rounded differently, the top open", hollow(keep(bk_fillet(up1, &kf, top, 1, 2, &mr, &miss)), top, 1, nullptr, nullptr, 0, 2, box), 3309.562, 1);
    printf("  hollows in %.0f ms\n", ms(t0));

    // Found by running random shapes through this engine and a reference kernel side by side.
    {
      // Where three smooth edges meet: no run walked twice (it read past an empty one).
      double ball[1] = {21.0713}, p[3] = {-1.25366, -0.358263, -1.36067}, n[3] = {-0.965991, -0.691866, -0.395964};
      double pick[6] = {-4.14527, -2.43209, 9.31728, -0.576288, 0.816962, -0.0215704};
      BKShape *half = keep(bk_split(keep(bk_primitive(BK_SPHERE, ball)), p, n, 0));
      BKShape *rounded = keep(bk_fillet(half, &ke, pick, 1, 1.64806, &mr, &miss));
      BKShape *bevelled = rounded ? bk_chamfer(rounded, &kb, body, 1, 1.38121, 0.93435, 0, &miss) : nullptr;
      check("a rounded half ball bevelled all round: made or said no, never a crash", rounded != nullptr, bevelled ? "made" : bk_last_error());
      if (bevelled) bk_free(bevelled);
      // A rounding's tool touching a face along a line: its crossings there come out loosely, and are made one.
      double cone[3] = {24.7353, 18.4702, 11.8902};
      is("round: a cone's rims at a radius whose crossings come out a hair apart (reference 4357.4627)", bk_fillet(keep(bk_primitive(BK_CONE, cone)), &kb, body, 1, 0.937602, &mr, &miss), 4357.4627, 0.01, 5);
      // A wedge's edges: how much they may take, from its faces shrunk by the roundings (the end's in-circle), not halves.
      double wedge[3] = {21.7653, 9.62249, 13.3893};
      BKShape *w = keep(bk_primitive(BK_WEDGE, wedge));
      is("round: a wedge all round, close to the most it takes (reference 493.5294)", bk_fillet(w, &kb, body, 1, 3.79, &mr, &miss), 493.5294, 1e-3);
      BKShape *over = bk_fillet(w, &kb, body, 1, 3.85, &mr, &miss);
      check("round: a wedge's ends hold no more than their in-circle", !over && near(mr, 3.79, 0.011), over ? "made" : fmt("most %.2f", mr));
      if (over) bk_free(over);
      // A ring's face curved in its cut (a half ball's dome): the rounding meets the dome, not its tangent at the rim.
      double dome[1] = {21.8554}, R = dome[0] / 2, r = 2.02357, cx = std::sqrt((R - r) * (R - r) - r * r), tz = r * R / std::hypot(cx, r);
      double cut = 0;
      for (int i = 0, n = 200000; i < n; i++) {
        double z = (i + 0.5) * tz / n, xs = std::sqrt(std::max(R * R - z * z, 0.0)), xf = cx + std::sqrt(std::max(r * r - (z - r) * (z - r), 0.0));
        if (xs > xf) cut += PI * (xs * xs - xf * xf) * tz / n;
      }
      is("round: a half ball's rim", bk_fillet(keep(bk_primitive(BK_HEMISPHERE, dome)), &kb, body, 1, r, &mr, &miss), 2 * PI / 3 * R * R * R - cut, 0.05, 3);
      // Walls moved in by their thickness along the face's own normal (a dome's or a cone's slant too).
      double h21[1] = {21.4096}, Rh = h21[0] / 2, th = 2.403, cap = Rh - 2 * th;
      is("hollow: a half ball", hollow(keep(bk_primitive(BK_HEMISPHERE, h21)), nullptr, 0, nullptr, nullptr, 0, th),
         2 * PI / 3 * Rh * Rh * Rh - PI * cap * cap * (3 * (Rh - th) - cap) / 3, 1e-3);
      // An oval's side moved in: no oval, but its exact area all the same (the oval's, less the wall times its length round,
      // plus π times the wall squared).
      double ov[4] = {17.8737, 10.0376, 90, 17.6743}, a = ov[0] / 2, b = ov[1] / 2, h = ov[3], t = 0.910856, round = 0;
      for (int k = 0, n = 100000; k < n; k++) round += std::hypot(a * std::sin(2 * PI * (k + 0.5) / n), b * std::cos(2 * PI * (k + 0.5) / n)) * 2 * PI / n;
      is("hollow: an oval", hollow(keep(bk_primitive(BK_OVAL, ov)), nullptr, 0, nullptr, nullptr, 0, t), PI * a * b * h - (PI * a * b - t * round + PI * t * t) * (h - 2 * t), 1e-3);
      // A face both opened and given a wall of its own: open.
      double cy[2] = {25.0234, 25.4184}, cyTop[6] = {0, 0, 1, 0, 0, 12.7092}, three = 3.03;
      is("hollow: a face both opened and walled is open (reference 2680.0055)", hollow(keep(bk_primitive(BK_CYLINDER, cy)), cyTop, 1, cyTop, &three, 1, 1.15996), 2680.0055, 1e-3);
      // A bevel wider than a small disc's edge reaches past the axis: its tool cut off there (reference 1652.8131).
      double cone2[3] = {19.5542, 5.66193, 12.1836}, rim[6] = {-2.83097, 0, 6.0918, 0.0980171, -0.995185, 0};
      is("bevel: a narrow cone's top rim, wide on the top", bk_chamfer(keep(bk_primitive(BK_CONE, cone2)), &ke, rim, 1, 2.14342, 1.59471, 0, &miss), 1652.8131, 0.05);
      // A face whose outline was walked from part way along a side: its two parts one side (reference 942.7708).
      double slab[3] = {29.2304, 26.1938, 11.7463}, peg[2] = {12.8745, 13.187};
      BKShape *pegged = keep(bk_boolean(BK_INTERSECT, keep(bk_primitive(BK_BOX, slab)), keep(at(keep(bk_primitive(BK_CYLINDER, peg)), -0.532917, 3.19933, 5.03263))));
      is("round: a box and a cylinder's common part all round", bk_fillet(pegged, &kb, body, 1, 1.22653, &mr, &miss), 942.7708, 0.01);
      // An opening beside a curved face, walls elsewhere thick: out only as far as it must go.
      double base[6] = {0, 0, -1, 0, 0, -3.14088}, d12[1] = {12.5635}, Ro = d12[0] / 2, wallBase = 2.48502;
      is("hollow: a half ball open below, a thick wall picked there too", hollow(keep(bk_primitive(BK_HEMISPHERE, d12)), base, 1, base, &wallBase, 1, 1.60785),
         2 * PI / 3 * (Ro * Ro * Ro - (Ro - 1.60785) * (Ro - 1.60785) * (Ro - 1.60785)), 0.01);
      // An edge where a turned face meets a flat one: its angle where they meet exactly, not at the mesh's point a chord's sag
      // off (reference 20.8083°).
      double slab2[3] = {27.4638, 29.2663, 29.211}, hole2[2] = {8.74376, 19.6135}, seam[6] = {13.7319, -2.48913, -0.867683, 0, 0, 1};
      BKShape *bitten = keep(bk_boolean(BK_SUBTRACT, keep(bk_primitive(BK_BOX, slab2)), keep(at(keep(bk_primitive(BK_CYLINDER, hole2)), 9.64518, -3.96658, -0.867683))));
      BKSection *across = bk_section(bitten, ke, seam, 20);
      check("section: where a cylinder's side meets a flat face, at their exact angle", across && near(across->angle, 20.8083, 1e-3), across ? fmt("%.4f°", across->angle) : bk_last_error());
      if (across) bk_section_free(across);
      // Rounded all round, then bevelled all round: nothing left to bevel (the roundings meet their faces smoothly, a bowl's
      // inside a hair off its mesh), so as it was.
      double bowl[2] = {28.7095, 2.81855};
      BKShape *rb = keep(bk_fillet(keep(bk_primitive(BK_BOWL, bowl)), &kb, body, 1, 0.402626, &mr, &miss));
      Got before = look(rb);
      is("bevel: a rounded bowl, every edge (none left sharp)", bk_chamfer(rb, &kb, body, 1, 0.383957, 1.12174, 0, &miss), before.volume, 1e-6);
      // A rim rounded wider than half its disc's radius: the rounding's circle crosses the axis, its arc doesn't (reference
      // 1296.4767; by Pappus, the corner's area turned round at its middle's distance from the axis).
      double rod[2] = {8.32971, 24.226}, Rr = rod[0] / 2, rr = 2.18641, rodRim[6] = {-Rr, 0, rod[1] / 2, 0, -1, 0};
      double spandrel = (1 - PI / 4) * rr * rr, inset = rr * (10 - 3 * PI) / (12 - 3 * PI);
      is("round: a narrow cylinder's top rim, wide", bk_fillet(keep(bk_primitive(BK_CYLINDER, rod)), &ke, rodRim, 1, rr, &mr, &miss),
         PI * Rr * Rr * rod[1] - spandrel * 2 * PI * (Rr - inset), 1e-3);
      // Two edges rounded where they meet at a corner, then the sharp edge ending there bevelled: the bevel runs on up the
      // seam the two roundings meet along, to where they meet the top smoothly (reference 7914.7425: its legs on the
      // roundings measured as chords, so its bevel stays a hair wider to the end; not run on, 7916.83).
      double cube20[3] = {20, 20, 20}, twoTop[12] = {0, 10, 10, 1, 0, 0, 10, 0, 10, 0, 1, 0}, upright[6] = {10, 10, 0, 0, 0, 1};
      int ke2[2] = {BK_PICK_EDGE, BK_PICK_EDGE};
      BKShape *corner = keep(bk_fillet(keep(bk_primitive(BK_BOX, cube20)), ke2, twoTop, 2, 3, &mr, &miss));
      is("bevel: an edge running on into the seam between two roundings", bk_chamfer(corner, &ke, upright, 1, 1, 1, 0, &miss), 7914.7425, 0.4);
      // A D: a disc cut by a flat. Its two edges rounded where the circle truly touches the round side, not a mesh point
      // a chord's sag off (exact: 2521.7830).
      double disc[2] = {20, 10}, slab3[3] = {20, 20, 10}, dEdges[12] = {5, 8.66025, 0, 0, 0, 1, 5, -8.66025, 0, 0, 0, 1};
      BKShape *dee = keep(bk_boolean(BK_INTERSECT, keep(bk_primitive(BK_CYLINDER, disc)), keep(at(keep(bk_primitive(BK_BOX, slab3)), -5, 0, 0))));
      is("round: a D's two edges, beside its round side", bk_fillet(dee, ke2, dEdges, 2, 2, &mr, &miss), 2521.7830, 0.01);
      // A plate bevelled on one edge, leaving a strip narrower than the rounding then asked for on the edge below: the
      // rounding runs on over the strip to the bevel (exact: 2212.534).
      double plate[3] = {20, 20, 6}, topFront[6] = {0, -10, 3, 1, 0, 0}, bottomFront[6] = {0, -10, -3, 1, 0, 0};
      BKShape *bevelled1 = keep(bk_chamfer(keep(bk_primitive(BK_BOX, plate)), &ke, topFront, 1, 4.5, 3, 0, &miss));
      is("round: wider than the strip beside it, running on over it", bk_fillet(bevelled1, &ke, bottomFront, 1, 3.5, &mr, &miss), 2212.534, 0.02, 7);
      // A wall thinner than the rounding on its top's edge: the rounding runs on over the top, down the inside to where the
      // circle meets it (exact: 724 less 0.214435 a unit of length).
      double blockA[3] = {10, 20, 20}, blockB[3] = {10.1, 30, 20}, wallTop[6] = {5, 0, 10, 0, 1, 0};
      BKShape *wall = keep(bk_boolean(BK_SUBTRACT, keep(bk_primitive(BK_BOX, blockA)), keep(at(keep(bk_primitive(BK_BOX, blockB)), -0.95, 0, 2))));
      is("round: a thin wall's top edge, wider than the wall", bk_fillet(wall, &ke, wallTop, 1, 1, &mr, &miss), 724 - 0.214435 * 20, 0.01);
      // A top rounded, then bevelled all round: each upright edge's bevel runs on up its seam (reference 6303.9067; its
      // four alike corners come out up to 1.2 apart, by which of its faces takes which leg).
      double b30[3] = {13.2815, 20.7227, 23.1038}, top30[6] = {0, 0, 1, 0, 0, b30[2] / 2};
      BKShape *roundTop = keep(bk_fillet(keep(bk_primitive(BK_BOX, b30)), &kf, top30, 1, 1.08193, &mr, &miss));
      is("bevel: a box with its top rounded, all round", bk_chamfer(roundTop, &kb, body, 1, 0.786019, 0.658416, 0, &miss), 6303.9067, 2.5);
      // One edge rounded, then bevelled all round: the bevel runs on round the rounding's ends, each run one tool (cut piece
      // by piece, its straight part and its arc left a face between them). Reference 6092.0459.
      double b1509[3] = {11.3016, 19.6324, 27.8478}, side[6] = {-b1509[0] / 2, 0, b1509[2] / 2, 0, 1, 0};
      BKShape *oneEdge = keep(bk_fillet(keep(bk_primitive(BK_BOX, b1509)), &ke, side, 1, 1.40145, &mr, &miss));
      is("bevel: a box with one edge rounded, all round", bk_chamfer(oneEdge, &kb, body, 1, 1.06377, 0.715423, 0, &miss), 6092.0459, 0.5);

      // A ball cut off-centre by a slanted plane: its rim a circle round the plane's normal (a ball is turned round any line
      // through its middle); three faces (bits of the tool's outer faces left a hair inside the ball's mesh
      // taken into the faces beside them). Exact volumes by turning the cut's outline (ρ, z) round that normal; kept: the side the normal
      // points to (0) or away from (1).
      auto turned = [](const std::vector<std::pair<double, double>> &loop) {
        double v = 0;
        for (size_t i = 0; i + 1 < loop.size(); i++) {
          auto [x1, y1] = loop[i];
          auto [x2, y2] = loop[i + 1];
          v += (x1 * x1 + x1 * x2 + x2 * x2) / 3 * (y2 - y1);
        }
        return std::fabs(PI * v);
      };
      auto arcTo = [](std::vector<std::pair<double, double>> &loop, double cx, double cz, double rad, double a0, double a1) {
        for (int i = 1, n = 20000; i <= n; i++) {
          double a = a0 + (a1 - a0) * i / n;
          loop.push_back({cx + rad * std::cos(a), cz + rad * std::sin(a)});
        }
      };
      // The cap above z = h of a ball of radius R, its rim rounded r.
      auto capRounded = [&](double R, double h, double r) {
        double cap = PI * (R - h) * (R - h) * (2 * R + h) / 3, zc = h + r, rc = std::sqrt((R - r) * (R - r) - zc * zc);
        std::vector<std::pair<double, double>> loop{{rc, h}, {std::sqrt(R * R - h * h), h}};
        double ts = std::atan2(zc, rc);
        arcTo(loop, 0, 0, R, std::atan2(h, std::sqrt(R * R - h * h)), ts);
        arcTo(loop, rc, zc, r, ts, -PI / 2);
        return cap - turned(loop);
      };
      double ball774[1] = {17.5967}, cutAt[3] = {-0.5319, -1.11591, -1.111}, cutN[3] = {0.318081, 0.797479, 0.814279};
      double h774 = (cutAt[0] * cutN[0] + cutAt[1] * cutN[1] + cutAt[2] * cutN[2]) / std::hypot(cutN[0], cutN[1], cutN[2]);
      BKShape *capped = keep(bk_split(keep(bk_primitive(BK_SPHERE, ball774)), cutAt, cutN, 1));
      is("round: a ball cut aslant, its rim (reference 901.9012)", bk_fillet(capped, &kb, body, 1, 2.11565, &mr, &miss), capRounded(ball774[0] / 2, -h774, 2.11565), 0.15, 3);
      // Hollowed: the void a ball cut and rounded as well, each smaller by the wall (it was left sharp: a cut shape's void came
      // without its edges). Reference 423.5560.
      double ball85[1] = {17.3662}, at85[3] = {1.01316, 1.55386, 0.529631}, n85[3] = {0.382081, 0.566047, -0.504127}, wall85 = 0.890721;
      double h85 = (at85[0] * n85[0] + at85[1] * n85[1] + at85[2] * n85[2]) / std::hypot(n85[0], n85[1], n85[2]), R85 = ball85[0] / 2;
      BKShape *rounded85 = keep(bk_fillet(keep(bk_split(keep(bk_primitive(BK_SPHERE, ball85)), at85, n85, 0)), &kb, body, 1, 1.9793, &mr, &miss));
      is("hollow: a ball cut aslant and rounded", hollow(rounded85, nullptr, 0, nullptr, nullptr, 0, wall85),
         capRounded(R85, h85, 1.9793) - capRounded(R85 - wall85, h85 + wall85, 1.9793 - wall85), 0.1);
      // A ball cut flat, its rim bevelled: the leg on the ball a straight line to a point on it, not along its tangent
      // (reference 1301.5833; the cut ball itself comes out 0.03 mm³ over, its curved face's missed volume shared out
      // by area).
      double slab470[3] = {20.0164, 24.817, 24.8619}, ball470[1] = {14.8439}, rim470[6] = {0, 1, 0, 2.4948, 12.4085, -1.54868};
      BKShape *dome470 = keep(bk_boolean(BK_INTERSECT, keep(bk_primitive(BK_BOX, slab470)), keep(at(keep(bk_primitive(BK_SPHERE, ball470)), 2.4948, 9.16612, -1.54868))));
      double R470 = ball470[0] / 2, h470 = slab470[1] / 2 - 9.16612, e470 = std::sqrt(R470 * R470 - h470 * h470), leg = 2.02392;
      double a470 = std::acos(h470 / R470) + 2 * std::asin(leg / (2 * R470));
      std::vector<std::pair<double, double>> bev{{e470 - leg, h470}, {e470, h470}};
      arcTo(bev, 0, 0, R470, std::atan2(h470, e470), PI / 2 - a470);
      bev.push_back({e470 - leg, h470});
      // Two boxes' common part, open at the top and at one side: that side the second box's face, with the first box's own
      // face just past it (not in the shape, so no wall). Reference 641.7472.
      double big79[3] = {29.1248, 28.4093, 13.6934}, small79[3] = {15.4352, 8.28922, 11.1492}, wall79 = 1.59593;
      BKShape *common79 = keep(bk_boolean(BK_INTERSECT, keep(bk_primitive(BK_BOX, big79)), keep(at(keep(bk_primitive(BK_BOX, small79)), -6.91055, 9.93607, -0.716893))));
      double open79[12] = {0, 0, 1, -6.87767, 9.93607, 4.85771, 0, 1, 0, -6.87767, 14.0807, -0.716893};
      double x79 = (-6.91055 + small79[0] / 2) + big79[0] / 2, y79 = small79[1], z79 = small79[2];
      is("hollow: two boxes' common part, open at the top and a side", hollow(common79, open79, 2, nullptr, nullptr, 0, wall79),
         x79 * y79 * z79 - (x79 - 2 * wall79) * (y79 - wall79) * (z79 - wall79), 1e-3);
      // Rounded all round, then hollowed: the void rounded too, narrower by the wall, each edge picked on it at its middle by
      // length (one of an edge's points by count could lie near its end, past the void's corner). Reference 347.6992.
      double big250[3] = {13.4032, 29.4925, 10.5391}, small250[3] = {11.2809, 16.3106, 20.0836}, r250 = 2.18804, t250 = 0.887828;
      BKShape *common250 = keep(bk_boolean(BK_INTERSECT, keep(bk_primitive(BK_BOX, big250)), keep(at(keep(bk_primitive(BK_BOX, small250)), -6.93709, 5.51232, -5.8938))));
      double l250[3] = {(-6.93709 + small250[0] / 2) + big250[0] / 2, small250[1], (-5.8938 + small250[2] / 2) + big250[2] / 2};
      auto roundedBox = [](const double *l, double r, double in) {
        double a = l[0] - 2 * in - 2 * r, b = l[1] - 2 * in - 2 * r, c = l[2] - 2 * in - 2 * r;
        return a * b * c + 2 * r * (a * b + b * c + a * c) + PI * r * r * (a + b + c) + 4 * PI / 3 * r * r * r;
      };
      is("hollow: two boxes' common part rounded all round", hollow(keep(bk_fillet(common250, &kb, body, 1, r250, &mr, &miss)), nullptr, 0, nullptr, nullptr, 0, t250),
         roundedBox(l250, r250, 0) - roundedBox(l250, r250 - t250, t250), 1e-3);
      // Inward roundings wider than a face beside them is across: cut, and kept while every face beside them is left and
      // none away from their edges is touched (reference 3264.7941).
      double slab741[3] = {25.3699, 11.1879, 14.3455}, notch741[3] = {8.05339, 15.9867, 18.1881}, face741[6] = {0, 1, 0, 2.91354, 5.59395, -0.473284};
      BKShape *notched = keep(bk_boolean(BK_SUBTRACT, keep(bk_primitive(BK_BOX, slab741)), keep(at(keep(bk_primitive(BK_BOX, notch741)), -8.43071, 7.58588, 4.66032))));
      is("cove: wider than a face beside it is across", bk_cove(notched, &kf, face741, 1, 2.13207, &mr, &miss), 3264.7941, 0.05);
      // A ring cut aslant, bevelled all round: the cut's line of mesh points zigzags across the ring's facets; each cross-
      // section is square to where the faces meet, not to that zigzag (else neighbours cross and the tool folds onto
      // itself); the bevel's face one face along the whole cut, five faces in all. Reference 1099.5708 (its legs on
      // the curved face measured to points on it). Looked at on a finer mesh: on the usual one the leg on the faceted face
      // moves by about a millimetre³ with how the facets beside the cut are split.
      double ring623[3] = {0, 37.9599, 5.10573}, at623[3] = {1.75048, 0.957164, 1.17883}, n623[3] = {0.481551, 0.64596, -0.246432};
      BKShape *cutRing = keep(bk_split(keep(bk_primitive(BK_TORUS, ring623)), at623, n623, 1));
      Got g623 = look(keep(bk_chamfer(cutRing, &kb, body, 1, 0.87252, 1.4573, 0, &miss)), 0.02);
      check("bevel: a ring cut aslant, all round", g623.shut && near(g623.volume, 1099.5708, 0.5) && g623.faces == 5, says(g623) + " (want 1099.5708)");
      is("bevel: a ball cut flat, its rim", bk_chamfer(dome470, &kf, rim470, 1, leg, leg, 0, &miss),
         4 * PI / 3 * R470 * R470 * R470 - PI * (R470 - h470) * (R470 - h470) * (2 * R470 + h470) / 3 - turned(bev), 0.05);
      // Every edge bevelled after every edge rounded (c501): the hair-thin remnants of faces where roundings met are left as
      // they are, the rest bevelled (reference 4567.990 mm³).
      double can501[2] = {18.3527, 27.7191}, at501[3] = {-0.950215, -1.85683, -0.327635}, n501[3] = {-0.738288, -0.877353, 0.235435};
      BKShape *cut501 = keep(bk_split(keep(bk_primitive(BK_CYLINDER, can501)), at501, n501, 1));
      BKShape *round501 = keep(bk_fillet(cut501, &kb, body, 1, 1.54905, &mr, &miss));
      is("bevel: every edge of a cut cylinder rounded all round", round501 ? bk_chamfer(round501, &kb, body, 1, 0.401328, 1.12913, 0, &miss) : nullptr,
         4567.990, 5);
    }
    for (BKShape *s : made) bk_free(s);
    bk_free(box);
  }

  // MARK: bolts and nuts
  {
    printf("— bolts and nuts\n");
    static const char *names[] = {"rod", "hex", "hex cone", "socket", "socket cone", "12-point", "12-point cone", "Torx", "Torx cone",
                                  "PH hex", "PH hex cone", "PH countersunk", "sleeve", "square nut", "hex nut", "cone nut"};
    // One built and looked at: one closed piece, its box exactly the size known beforehand (centred), its mesh short of
    // its exact volume by no more than its chords miss; the same mesh to the last bit when built again.
    std::string wrong;
    int built = 0, bad = 0;
    auto one = [&](const BKFastener &f, double c) {
      built++;
      char what[96];
      snprintf(what, sizeof what, "%s %s c%.2g L%.3g", names[f.kind], bk_thread_name(f.size), c, f.length);
      auto fault = [&](const std::string &why) {
        if (bad++ < 4) wrong += std::string(what) + ": " + why + "; ";
      };
      // (Every fourth one built twice, to see it come out the same.)
      BKShape *s = bk_fastener(&f, c), *again = built % 4 == 1 ? bk_fastener(&f, c) : bk_copy(s);
      if (!s || !again) {
        fault(bk_last_error());
        bk_free(s), bk_free(again);
        return;
      }
      double ext[3];
      bk_fastener_extent(&f, c, ext);
      BKMesh *m = bk_mesh(s, 0.05), *m2 = bk_mesh(again, 0.05);
      double off = 0;
      for (int i = 0; i < 3; i++) off = std::max({off, std::fabs(m->bbox[3 + i] - m->bbox[i] - ext[i]), std::fabs(m->bbox[3 + i] + m->bbox[i])});
      double sv;
      std::string why;
      bool shut = closed(m, sv, why);
      if (!shut || m->valid != 1) fault("not closed " + why);
      else if (bk_piece_count(s) != 1) fault("not one piece");
      else if (off > 1e-9) fault(fmt("box off by %.3g", off));
      else if (!(sv < m->volume * 1.0001 + 1e-9 ? sv > m->volume * 0.96 : sv < m->volume * 1.04)) fault(fmt("mesh %.4f against %.4f", sv, m->volume));
      else if (m->vertexCount != m2->vertexCount || memcmp(m->positions, m2->positions, sizeof(float) * 3 * m->vertexCount) != 0) fault("not the same twice");
      bk_mesh_free(m), bk_mesh_free(m2), bk_free(s), bk_free(again);
    };
    auto t0 = std::chrono::steady_clock::now();
    for (int kind = BK_ROD; kind <= BK_CONE_NUT; kind++)
      for (int size = 0; size < bk_thread_count(); size++)
        for (double c : {0.0, 0.2, 0.5}) {
          // Every thread at the usual clearance; the smallest and largest with none and with a lot.
          if (c != 0.2 && size != 0 && size != bk_thread_count() - 1) continue;
          BKFastener f{};
          f.kind = kind, f.size = size;
          bk_fastener_defaults(&f, 1);
          one(f, c);
        }
    check("every bolt and nut at every thread: one closed piece of the size known beforehand, the same twice", bad == 0,
          fmt("%.0f built in %.1f s ", built, ms(t0) / 1000) + wrong);
    // Sizes at random within what each allows (fitted as the app fits them after each change).
    std::mt19937 rng(20261004);
    built = bad = 0, wrong.clear();
    for (int kind = BK_ROD; kind <= BK_CONE_NUT; kind++)
      for (int n = 0; n < 8; n++) {
        BKFastener f{};
        f.kind = kind, f.size = (int)(rng() % bk_thread_count());
        bk_fastener_defaults(&f, 1);
        int fields = bk_fastener_fields(kind);
        for (int field = BK_LENGTH; field <= BK_DEPTH; field++) {
          if (!(fields & (1 << field))) continue;
          double r[2];
          bk_fastener_range(&f, field, 0, r);
          if (!(r[0] <= r[1])) continue;
          double u = (rng() % 1000) / 999.0, v = r[0] + u * (std::min(r[1], field == BK_LENGTH ? r[0] + 60 : r[1]) - r[0]);
          double *at[] = {&f.length, &f.width, &f.height, &f.angle, &f.seat, &f.drive, &f.recess, &f.depth};
          if (field == BK_DRIVE) bk_fastener_drive(&f, torxDrive(kind) ? bk_torx_number((int)(rng() % bk_torx_count())) : phillipsDrive(kind) ? 1 + rng() % 4 : v);
          else *at[field] = v, bk_fastener_fit(&f);
        }
        one(f, (rng() % 6) * 0.1);
      }
    check("bolts and nuts of sizes picked at random within their ranges", bad == 0, fmt("%.0f built ", built) + wrong);

    // Its exact volume, which a mesh comes ever nearer to as it's made finer; and what the mesh misses made up face by face.
    BKFastener m8{};
    m8.kind = BK_HEX, m8.size = 4;
    bk_fastener_defaults(&m8, 1);
    BKShape *bolt = bk_fastener(&m8, 0.2);
    {
      BKMesh *coarse = bk_mesh(bolt, 0.05), *fine = bk_mesh(bolt, 0.002);
      double vc, vf;
      std::string why;
      closed(coarse, vc, why), closed(fine, vf, why);
      double exact = coarse->volume;
      check("a bolt's mesh comes ever nearer its exact volume as it's made finer",
            exact > vf && vf > vc && (exact - vf) < 0.06 * (exact - vc), fmt("%.4f, %.4f → %.4f mm³", vc, vf, exact));
      bk_mesh_free(coarse), bk_mesh_free(fine);
    }
    // Merges and cuts with them: a threaded hole, a bolt through a plate, a nut on a bolt, a bolt cut aslant (which keeps
    // all of it).
    {
      double box[3] = {30, 30, 20}, plate[3] = {40, 40, 6};
      BKFastener rod{};
      rod.kind = BK_ROD, rod.size = 4;
      bk_fastener_defaults(&rod, 0);
      rod.length = 30;
      BKShape *b = bk_primitive(BK_BOX, box), *r = bk_fastener(&rod, 0.2), *hole = bk_boolean(BK_SUBTRACT, b, r);
      BKMesh *hm = bk_mesh(hole, 0.05);
      check("a threaded hole: a box less a rod through it", hm->valid == 1 && bk_piece_count(hole) == 1 && hm->volume < 18000 - 600 && hm->volume > 18000 - 1000,
            fmt("%.3f mm³", hm->volume));
      double away[12] = {1, 0, 0, 0.3, 0, 1, 0, -0.2, 0, 0, 1, 0.7};
      BKShape *r2 = bk_transform(r, away), *hole2 = bk_boolean(BK_SUBTRACT, b, r2);
      BKMesh *hm2 = bk_mesh(hole2, 0.05);
      check("the same hole moved within the box takes the same out", hm2->valid == 1 && near(hm2->volume, hm->volume, 1e-6 * hm->volume),
            fmt("%.6f, %.6f mm³", hm2->volume, hm->volume));
      BKFastener m12{}, nut{};
      m12.kind = BK_HEX, m12.size = 6;
      bk_fastener_defaults(&m12, 0);
      m12.length = 30;
      nut.kind = BK_HEX_NUT, nut.size = 4;
      bk_fastener_defaults(&nut, 1);
      double down[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -3}, lower[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -5};
      BKShape *p = bk_primitive(BK_BOX, plate), *m12s = bk_fastener(&m12, 0.2), *m12d = bk_transform(m12s, down), *through = bk_boolean(BK_SUBTRACT, p, m12d);
      BKShape *n = bk_fastener(&nut, 0.2), *nl = bk_transform(n, lower), *onBolt = bk_boolean(BK_UNION, bolt, nl);
      BKMesh *tm = bk_mesh(through, 0.05), *om = bk_mesh(onBolt, 0.05);
      check("a bolt through a plate, and a nut on a bolt", tm->valid == 1 && bk_piece_count(through) == 1 && om->valid == 1 && bk_piece_count(onBolt) == 1,
            fmt("%.3f, %.3f mm³", tm->volume, om->volume));
      double at[3] = {1, 2, 3}, aslant[3] = {0.3, -0.2, 1};
      BKShape *up = bk_split(bolt, at, aslant, 0), *dn = bk_split(bolt, at, aslant, 1);
      BKMesh *um = bk_mesh(up, 0.05), *dm = bk_mesh(dn, 0.05), *wm = bk_mesh(bolt, 0.05);
      check("a bolt cut aslant keeps all of it", um->valid == 1 && dm->valid == 1 && near(um->volume + dm->volume, wm->volume, 1e-5 * wm->volume),
            fmt("%.4f + %.4f = %.4f", um->volume, dm->volume, wm->volume));
      for (BKMesh *x : {hm, hm2, tm, om, um, dm, wm}) bk_mesh_free(x);
      for (BKShape *x : {b, r, hole, r2, hole2, p, m12s, m12d, through, n, nl, onBolt, up, dn}) bk_free(x);
    }
    // Rounded or bevelled like any other shape, and its sizes refused when they don't fit (said as a nut's or a bolt's).
    {
      int kf = BK_PICK_FACE, miss;
      BKMesh *m = bk_mesh(bolt, 0.05);
      double top[6] = {0, 0, 1, 0, 0, m->bbox[5]}, mr;
      BKShape *rounded = bk_fillet(bolt, &kf, top, 1, 0.5, &mr, &miss), *bevelled = bk_chamfer(bolt, &kf, top, 1, 0.3, 0.3, 0, &miss);
      BKMesh *rm = rounded ? bk_mesh(rounded, 0.05) : nullptr, *bm = bevelled ? bk_mesh(bevelled, 0.05) : nullptr;
      check("a bolt's head rounded and bevelled", rm && bm && rm->valid == 1 && bm->valid == 1 && rm->volume < m->volume && bm->volume < rm->volume);
      if (rm) bk_mesh_free(rm);
      if (bm) bk_mesh_free(bm);
      bk_mesh_free(m), bk_free(rounded), bk_free(bevelled);
      BKFastener misfit{}, nutMisfit{};
      misfit.kind = BK_SOCKET, misfit.size = 4;
      bk_fastener_defaults(&misfit, 1);
      misfit.depth = 20;
      nutMisfit.kind = BK_HEX_NUT, nutMisfit.size = 4;
      bk_fastener_defaults(&nutMisfit, 1);
      nutMisfit.width = 2;
      BKShape *no = bk_fastener(&misfit, 0.2);
      std::string boltSays = bk_last_error();
      BKShape *noNut = bk_fastener(&nutMisfit, 0.2);
      std::string nutSays = bk_last_error();
      BKShape *noNumber = bk_fastener(&m8, NAN);
      check("sizes that don't fit are refused, said as a bolt's or a nut's", !no && !noNut && !noNumber && boltSays.rfind("bolt: ", 0) == 0 && nutSays.rfind("nut: ", 0) == 0,
            boltSays + " · " + nutSays);
    }
    // Quick enough to change a size and see it at once.
    {
      auto t1 = std::chrono::steady_clock::now();
      BKShape *s = bk_fastener(&m8, 0.2);
      double small = ms(t1);
      BKFastener big{};
      big.kind = BK_TORX_CONE, big.size = bk_thread_count() - 1;
      bk_fastener_defaults(&big, 1);
      big.length = 100;
      bk_fastener_fit(&big);
      t1 = std::chrono::steady_clock::now();
      BKShape *b = bk_fastener(&big, 0.2);
      double large = ms(t1);
      check("an M8 bolt made (and meshed twice) quickly, an M24 × 100 one in well under a second", s && b && (!timed || (small < 150 && large < 800)),
            fmt("%.0f ms, %.0f ms", small, large));
      bk_free(s), bk_free(b);
    }
    bk_free(bolt);
  }

  // MARK: STEP files
  // Shapes written out and read back by a small reader of the test's own: every reference there, every loop closed, every
  // edge met once each way within its shell, every corner on its face's plane, the volume as the mesh's, a solid per piece
  // (a sealed hollow a void of its solid), flat faces one face each, names as given, the same text when written again.
  {
    printf("— STEP files\n");
    struct Entity {
      std::string type, args;
    };
    // The DATA section's entities (a complex one's type left empty), and the schema the header names.
    auto parse = [](const std::string &text, std::map<int, Entity> &out, std::string &schema) {
      size_t sch = text.find("FILE_SCHEMA(");
      if (sch != std::string::npos) schema = text.substr(sch, text.find(';', sch) - sch);
      size_t at = text.find("DATA;"), end = text.rfind("ENDSEC;");
      if (at == std::string::npos || end == std::string::npos) return false;
      std::string stmt;
      bool quoted = false;
      for (size_t i = at + 5; i < end; i++) {
        char c = text[i];
        if (c == '\'') quoted = !quoted;
        if (c == '\n' && !quoted) continue;
        if (c == ';' && !quoted) {
          size_t eq = stmt.find('=');
          if (stmt.empty() || stmt[0] != '#' || eq == std::string::npos) return false;
          int id = std::atoi(stmt.c_str() + 1);
          std::string body = stmt.substr(eq + 1);
          size_t open = body.find('(');
          if (open == std::string::npos || body.back() != ')') return false;
          out[id] = {body.substr(0, open), body.substr(open + 1, body.size() - open - 2)};
          stmt.clear();
          continue;
        }
        stmt += c;
      }
      return true;
    };
    // An entity's arguments, split at the top level.
    auto split = [](const std::string &s) {
      std::vector<std::string> parts;
      std::string cur;
      int depth = 0;
      bool quoted = false;
      for (char c : s) {
        if (c == '\'') quoted = !quoted;
        if (!quoted && c == '(') depth++;
        if (!quoted && c == ')') depth--;
        if (!quoted && depth == 0 && c == ',') {
          parts.push_back(cur), cur.clear();
          continue;
        }
        cur += c;
      }
      parts.push_back(cur);
      return parts;
    };
    auto refs = [&](const std::string &s) {
      std::vector<int> out;
      for (const auto &p : split(s.size() > 1 && s[0] == '(' ? s.substr(1, s.size() - 2) : s))
        if (!p.empty() && p[0] == '#') out.push_back(std::atoi(p.c_str() + 1));
      return out;
    };
    struct Read {
      bool ok = true;
      std::string why;
      int solids = 0, voids = 0, faces = 0, products = 0;
      double volume = 0, off = 0;
      std::vector<std::string> names;
    };
    auto read = [&](const std::string &text) {
      Read r;
      std::map<int, Entity> e;
      std::string schema;
      auto fail = [&](const std::string &why) {
        if (r.ok) r.ok = false, r.why = why;
      };
      if (!parse(text, e, schema) || schema.find("AUTOMOTIVE_DESIGN") == std::string::npos) {
        fail("not read");
        return r;
      }
      for (auto &[id, en] : e)
        for (size_t i = en.args.find('#'); i != std::string::npos; i = en.args.find('#', i + 1))
          if (!e.count(std::atoi(en.args.c_str() + i + 1))) fail("a reference to nothing");
      auto args = [&](int id, const char *type) {
        auto it = e.find(id);
        if (it == e.end() || it->second.type != type) {
          fail(std::string("not a ") + type);
          return std::vector<std::string>();
        }
        return split(it->second.args);
      };
      auto point = [&](int id) {
        auto a = args(id, "CARTESIAN_POINT");
        bce::V3 p{0, 0, 0};
        if (a.size() == 2) {
          auto c = split(a[1].substr(1, a[1].size() - 2));
          if (c.size() == 3) p = {std::atof(c[0].c_str()), std::atof(c[1].c_str()), std::atof(c[2].c_str())};
        }
        return p;
      };
      // One shell: its faces' loops closed, its edges met once each way; what it bounds (its faces' loops fanned from the
      // origin).
      auto shell = [&](int id) {
        auto a = args(id, "CLOSED_SHELL");
        double v = 0;
        std::map<int, std::pair<int, int>> used;
        if (a.size() != 2) return v;
        for (int f : refs(a[1])) {
          auto fa = args(f, "ADVANCED_FACE");
          if (fa.size() != 4 || fa[3] != ".T.") {
            fail("a face");
            continue;
          }
          r.faces++;
          auto pa = args(std::atoi(fa[2].c_str() + 1), "PLANE");
          auto ax = pa.size() == 2 ? args(std::atoi(pa[1].c_str() + 1), "AXIS2_PLACEMENT_3D") : std::vector<std::string>();
          if (ax.size() != 4) {
            fail("a plane");
            continue;
          }
          bce::V3 o = point(std::atoi(ax[1].c_str() + 1)), n{0, 0, 0};
          auto da = args(std::atoi(ax[2].c_str() + 1), "DIRECTION");
          if (da.size() == 2) {
            auto c = split(da[1].substr(1, da[1].size() - 2));
            if (c.size() == 3) n = {std::atof(c[0].c_str()), std::atof(c[1].c_str()), std::atof(c[2].c_str())};
          }
          auto bounds = refs(fa[1]);
          for (size_t b = 0; b < bounds.size(); b++) {
            auto it = e.find(bounds[b]);
            if (it == e.end() || it->second.type != (b == 0 ? "FACE_OUTER_BOUND" : "FACE_BOUND")) {
              fail("a bound");
              continue;
            }
            auto ba = split(it->second.args);
            auto la = args(std::atoi(ba[1].c_str() + 1), "EDGE_LOOP");
            if (la.size() != 2 || ba[2] != ".T.") {
              fail("a loop");
              continue;
            }
            std::vector<bce::V3> q;
            std::vector<int> starts, ends;
            for (int oe : refs(la[1])) {
              auto oa = args(oe, "ORIENTED_EDGE");
              if (oa.size() != 5) continue;
              int edge = std::atoi(oa[3].c_str() + 1);
              bool along = oa[4] == ".T.";
              auto ea = args(edge, "EDGE_CURVE");
              if (ea.size() != 5) continue;
              int v1 = std::atoi(ea[1].c_str() + 1), v2 = std::atoi(ea[2].c_str() + 1);
              starts.push_back(along ? v1 : v2), ends.push_back(along ? v2 : v1);
              (along ? used[edge].first : used[edge].second)++;
              auto va = args(starts.back(), "VERTEX_POINT");
              if (va.size() == 2) q.push_back(point(std::atoi(va[1].c_str() + 1)));
            }
            for (size_t k = 0; k < starts.size(); k++)
              if (ends[k] != starts[(k + 1) % starts.size()]) fail("a loop not closed");
            bce::V3 newell{0, 0, 0};
            for (size_t k = 0; k < q.size(); k++) {
              r.off = std::max(r.off, std::fabs(bce::dot(n, q[k] - o)));
              newell = newell + bce::cross(q[k], q[(k + 1) % q.size()]);
              if (k >= 1 && k + 1 < q.size()) v += bce::dot(q[0], bce::cross(q[k], q[k + 1])) / 6;
            }
            if ((bce::dot(newell, n) > 0) != (b == 0)) fail("a loop turning the wrong way");
          }
        }
        for (auto &[edge, c] : used)
          if (c.first != 1 || c.second != 1) fail("an edge not met once each way");
        return v;
      };
      for (auto &[id, en] : e) {
        if (en.type == "PRODUCT") {
          r.products++;
          r.names.push_back(split(en.args)[0]);
        }
        if (en.type == "MANIFOLD_SOLID_BREP" || en.type == "BREP_WITH_VOIDS") {
          auto a = split(en.args);
          r.solids++;
          r.volume += shell(std::atoi(a[1].c_str() + 1));
          if (en.type == "BREP_WITH_VOIDS")
            for (int v : refs(a[2])) {
              auto va = args(v, "ORIENTED_CLOSED_SHELL");
              if (va.size() != 4 || va[3] != ".F.") {
                fail("a void");
                continue;
              }
              r.voids++;
              r.volume -= shell(std::atoi(va[2].c_str() + 1));
            }
        }
      }
      return r;
    };
    auto data = [](const std::string &text) { return text.substr(text.find("DATA;")); };
    double b20[3] = {20, 30, 10}, cyl[2] = {14, 20}, ball[1] = {20}, hole[2] = {8, 40}, cube[3] = {10, 10, 10}, body6[6] = {0};
    int kb = BK_PICK_BODY, miss;
    auto at = [](BKShape *s, double x, double y, double z) {
      double m[12] = {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z};
      return bk_transform(s, m);
    };
    BKShape *box = bk_primitive(BK_BOX, b20), *c = bk_primitive(BK_CYLINDER, cyl), *s = bk_primitive(BK_SPHERE, ball);
    BKShape *drill = bk_primitive(BK_CYLINDER, hole), *drilled = bk_boolean(BK_SUBTRACT, box, drill);
    BKShape *hollowed = bk_hollow(box, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, 2, nullptr);
    BKShape *k1 = bk_primitive(BK_BOX, cube), *k2 = at(k1, 10, 10, 0), *touching = bk_boolean(BK_UNION, k1, k2);
    BKFastener f{};
    f.kind = BK_HEX, f.size = 4;
    bk_fastener_defaults(&f, 1);
    BKShape *bolt = bk_fastener(&f, 0.2);
    BKShape *rounded = bk_fillet(box, &kb, body6, 1, 2, nullptr, &miss);
    struct Want {
      const char *name;
      std::vector<BKShape *> shapes;
      int solids, voids, faces;  // (faces −1: not counted)
    };
    std::vector<Want> wants = {{"a box", {box}, 1, 0, 6},
                               {"a cylinder", {c}, 1, 0, -1},
                               {"a ball", {s}, 1, 0, -1},
                               {"a drilled box", {drilled}, 1, 0, -1},
                               {"a hollow box (its void inside it)", {hollowed}, 1, 1, 12},
                               {"two cubes touching along an edge (two solids)", {touching}, 2, 0, 12},
                               {"an M8 hex bolt", {bolt}, 1, 0, -1},
                               {"a rounded box", {rounded}, 1, 0, -1},
                               {"three bodies in one file", {box, c, bolt}, 3, 0, -1}};
    bool ok = true;
    std::string notes;
    for (const auto &w : wants) {
      std::vector<bce::Shape> shapes;
      bool built = true;
      for (BKShape *x : w.shapes) {
        built = built && x;
        if (x) shapes.push_back(bce::heldShape(x));
      }
      std::string text, again, why;
      std::vector<std::string> names{"Bolt’s 'part' 1"};
      bool wrote = built && bce::stepText(shapes, names, "test.step", text, why) && bce::stepText(shapes, names, "test.step", again, why);
      Read r = wrote ? read(text) : Read{};
      double want = 0;
      for (const auto &x : shapes) {
        bce::Solid m;
        bce::mesh(x, bce::fileDeflection, m);
        want += m.meshVolume();
      }
      bool good = wrote && r.ok && r.solids == w.solids && r.voids == w.voids && (w.faces < 0 || r.faces == w.faces) && r.products == (int)shapes.size() &&
                  std::fabs(r.volume - want) <= 1e-9 * want && r.off <= 1e-6 && data(text) == data(again) &&
                  !r.names.empty() && r.names[0] == "'Bolt\\X2\\2019\\X0\\s ''part'' 1'";
      char note[400];
      snprintf(note, sizeof note, "%s%s: %d solids, %d voids, %d faces, volume %.6f (mesh %.6f), %.0f kB; %s", w.name, good ? "" : " ✗", r.solids, r.voids, r.faces,
               r.volume, want, text.size() / 1024.0, wrote ? r.why.c_str() : why.c_str());
      if (!good) ok = false, notes += std::string(note) + "\n  ";
      else printf("  %s\n", note);
    }
    check("STEP: shapes written as closed solids and read back alike", ok, notes);
    // Through the C API, to a file.
    std::string path = std::string(getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp") + "/bcad-engine-test.step";
    const BKShape *two[2] = {box, bolt};
    const char *named[2] = {"Box", "Bolt"};
    bool wroteFile = bk_export_step(two, named, 2, path.c_str()) == 1;
    std::string fileText;
    if (FILE *fp = fopen(path.c_str(), "rb")) {
      char buf[65536];
      size_t n;
      while ((n = fread(buf, 1, sizeof buf, fp)) > 0) fileText.append(buf, n);
      fclose(fp);
    }
    Read rf = read(fileText);
    check("STEP: written to a file through the C API", wroteFile && fileText.rfind("ISO-10303-21;", 0) == 0 && rf.ok && rf.solids == 2 && rf.products == 2,
          wroteFile ? fmt("%.0f solids", rf.solids) + " " + rf.why : std::string(bk_last_error()));
    remove(path.c_str());
    for (BKShape *x : {box, c, s, drill, drilled, hollowed, k1, k2, touching, bolt, rounded}) bk_free(x);
  }

  // MARK: print meshes
  // Bodies as 3MF and STL files take them: every edge, by point number, between exactly two triangles run opposite ways;
  // the shells as many as the pieces and voids; the volume the 0.01 mm mesh's own; no triangle left flat or turned over
  // in float; where parts touch, each with its own points there.
  {
    printf("— print meshes\n");
    // Checked by point numbers alone (as a 3MF reader joins them): shells, signed volume, why not.
    auto strict = [](const BKPrintMesh *m, int &shells, double &vol, std::string &why) {
      shells = 0, vol = 0;
      std::map<std::pair<uint32_t, uint32_t>, std::vector<int>> sides;  // the triangles running along each side
      std::vector<int> up(m->triangleCount);
      for (int t = 0; t < m->triangleCount; t++) up[t] = t;
      std::function<int(int)> root = [&](int x) { return up[x] == x ? x : up[x] = root(up[x]); };
      for (int t = 0; t < m->triangleCount; t++) {
        const uint32_t *v = m->indices + 3 * t;
        if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2] || std::max({v[0], v[1], v[2]}) >= (uint32_t)m->vertexCount) return why = "a bad triangle", false;
        const float *a = m->positions + 3 * v[0], *b = m->positions + 3 * v[1], *c = m->positions + 3 * v[2];
        vol += ((double)a[0] * ((double)b[1] * c[2] - (double)b[2] * c[1]) + (double)a[1] * ((double)b[2] * c[0] - (double)b[0] * c[2]) +
                (double)a[2] * ((double)b[0] * c[1] - (double)b[1] * c[0])) / 6;
        for (int k = 0; k < 3; k++) sides[{v[k], v[(k + 1) % 3]}].push_back(t);
      }
      for (auto &[e, ts] : sides) {
        if (ts.size() != 1) return why = "a side run the same way twice", false;
        auto back = sides.find({e.second, e.first});
        if (back == sides.end()) return why = "an open side", false;
        int a = root(ts[0]), b = root(back->second[0]);
        if (a != b) up[a] = b;
      }
      for (int t = 0; t < m->triangleCount; t++) shells += root(t) == t;
      return true;
    };
    // The mesh's own volume at the files' detail.
    auto own = [](const BKShape *x) {
      BKMesh *m = bk_mesh(x, 0.01);
      double v = 0;
      for (int t = 0; t < m->triangleCount; t++) {
        const float *a = m->positions + 3 * m->indices[3 * t], *b = m->positions + 3 * m->indices[3 * t + 1], *c = m->positions + 3 * m->indices[3 * t + 2];
        v += ((double)a[0] * ((double)b[1] * c[2] - (double)b[2] * c[1]) + (double)a[1] * ((double)b[2] * c[0] - (double)b[0] * c[2]) +
              (double)a[2] * ((double)b[0] * c[1] - (double)b[1] * c[0])) / 6;
      }
      bk_mesh_free(m);
      return v;
    };
    // Sound, `want` shells, its volume the mesh's own and near `exact` (2%: the mesh cuts inside curves).
    auto sound = [&](const char *name, BKShape *x, int want, double exact, std::string &notes) {
      BKPrintMesh *pm = x ? bk_print_mesh(x) : nullptr;
      int shells = 0;
      double vol = 0, mine = x ? own(x) : 0;
      std::string why = pm && !pm->valid ? bk_last_error() : "";
      bool good = pm && pm->valid && strict(pm, shells, vol, why) && shells == want && pm->crowded == 0 && pm->slivers == 0 &&
                  std::fabs(vol - pm->volume) <= 1e-9 * std::fabs(vol) && std::fabs(vol - mine) <= 1e-6 * std::fabs(mine) && std::fabs(vol - exact) <= 0.02 * exact;
      if (!good) {
        char n[300];
        snprintf(n, sizeof n, "%s: %d shells, volume %.6f (mesh %.6f, exact %.6f), %d slivers, %d crowded, %s", name, shells, vol, mine, exact,
                 pm ? pm->slivers : -1, pm ? pm->crowded : -1, why.c_str());
        notes += std::string(n) + "\n  ";
      }
      bk_print_mesh_free(pm);
      return good;
    };
    std::string notes;
    int bad = 0;
    for (const Case &c : cases) {
      BKShape *x = bk_primitive(c.kind, c.p.data());
      bad += !sound(c.name, x, 1, c.volume, notes);
      bk_free(x);
    }
    check("print meshes: every kind of shape", bad == 0, notes);
    // Every bolt and nut, as made, stretched, mirrored and turned.
    notes.clear(), bad = 0;
    int made = 0;
    for (int kind = BK_ROD; kind <= BK_CONE_NUT; kind++) {
      BKFastener f{};
      f.kind = kind, f.size = 4;
      bk_fastener_defaults(&f, 1);
      BKShape *x = bk_fastener(&f, 0.2);
      double plain = x ? own(x) : 0;
      double stretch[12] = {1.5, 0, 0, 3, 0, 1, 0, 0, 0, 0, 1, 0}, mirror[12] = {-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0},
             turn[12] = {0.6, -0.8, 0, 10, 0.8, 0.6, 0, -20, 0, 0, 1, 5};
      const double *ms[3] = {stretch, mirror, turn};
      double scale[3] = {1.5, 1, 1};
      bad += !sound("a bolt or nut", x, 1, plain, notes);
      for (int k = 0; k < 3; k++) {
        BKShape *y = x ? bk_transform(x, ms[k]) : nullptr;
        bad += !sound(k == 0 ? "a stretched bolt or nut" : k == 1 ? "a mirrored bolt or nut" : "a turned bolt or nut", y, 1, plain * scale[k], notes);
        bk_free(y);
      }
      made += x != nullptr;
      bk_free(x);
    }
    check("print meshes: bolts and nuts, stretched, mirrored and turned", bad == 0 && made == 16, notes);
    // Parts touching along an edge and at a corner: one solid each, its own points where they touch.
    notes.clear(), bad = 0;
    double cube[3] = {10, 10, 10}, b20[3] = {20, 30, 10};
    auto at = [](BKShape *s, double x, double y, double z) {
      double m[12] = {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z};
      return bk_transform(s, m);
    };
    BKShape *k1 = bk_primitive(BK_BOX, cube), *k2 = at(k1, 10, 10, 0), *k3 = at(k1, 10, 10, 10);
    BKShape *edge = bk_boolean(BK_UNION, k1, k2), *corner = bk_boolean(BK_UNION, k1, k3);
    bad += !sound("two cubes touching along an edge", edge, 2, 2000, notes);
    bad += !sound("two cubes touching at a corner", corner, 2, 2000, notes);
    BKPrintMesh *pe = bk_print_mesh(edge), *pc = bk_print_mesh(corner);
    BKMesh *me = bk_mesh(edge, 0.01);
    // (Welded by place, as an STL reader would, the edge's two points are shared: two more points here.)
    std::map<std::tuple<float, float, float>, int> places;
    for (int i = 0; i < me->vertexCount; i++) places[{me->positions[3 * i], me->positions[3 * i + 1], me->positions[3 * i + 2]}] = 1;
    bool own2 = pe->vertexCount == (int)places.size() + 2 && pc->vertexCount == 16;
    bk_mesh_free(me);
    bk_print_mesh_free(pe), bk_print_mesh_free(pc);
    // A hollow box: its void a shell of its own, turned inward.
    BKShape *box = bk_primitive(BK_BOX, b20), *hollowed = bk_hollow(box, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, 2, nullptr);
    bad += !sound("a hollow box", hollowed, 2, 6000 - 16 * 26 * 6, notes);
    // Far from the origin, a fine thread still sound in float.
    BKFastener f{};
    f.kind = BK_HEX, f.size = 0;
    bk_fastener_defaults(&f, 1);
    BKShape *m3 = bk_fastener(&f, 0.2), *far = m3 ? at(m3, 900, -700, 300) : nullptr;
    bad += !sound("an M3 bolt far from the origin", far, 1, m3 ? own(m3) : 0, notes);
    check("print meshes: touching parts, a void, far from the origin", bad == 0 && own2, notes + (own2 ? "" : "points where parts touch not given twice"));
    // Nothing to print: said so.
    BKShape *apart = at(k1, 50, 0, 0), *nothing = bk_boolean(BK_INTERSECT, k1, apart);
    BKPrintMesh *pn = nothing ? bk_print_mesh(nothing) : nullptr;
    check("print meshes: an empty shape is said to be empty", !nothing || (pn && !pn->valid && std::string(bk_last_error()) == "file 0: empty"),
          nothing ? bk_last_error() : "");
    bk_print_mesh_free(pn);
    for (BKShape *x : {k1, k2, k3, edge, corner, box, hollowed, m3, far, apart, nothing}) bk_free(x);
  }

  // MARK: mesh bodies
  // A body that is a mesh as given (a sculpted one): its surface exactly, the same at any detail; merged, cut and split
  // like any other; printed and written to STEP; in pieces when it is; overlapping pieces resolved to what they enclose;
  // anything not a closed solid refused, and why.
  {
    printf("— mesh bodies\n");
    // A shape's mesh with its points joined where they're one (as a sculpt holds it).
    struct Welded {
      std::vector<float> pos;
      std::vector<uint32_t> idx;
    };
    auto welded = [](const BKShape *x, double d) {
      Welded w;
      BKMesh *m = bk_mesh(x, d);
      std::map<std::tuple<float, float, float>, uint32_t> at;
      for (int t = 0; t < 3 * m->triangleCount; t++) {
        const float *p = m->positions + 3 * m->indices[t];
        auto key = std::make_tuple(p[0], p[1], p[2]);
        auto it = at.find(key);
        if (it == at.end()) {
          it = at.emplace(key, (uint32_t)(w.pos.size() / 3)).first;
          w.pos.insert(w.pos.end(), {p[0], p[1], p[2]});
        }
        w.idx.push_back(it->second);
      }
      bk_mesh_free(m);
      return w;
    };
    auto body = [](const Welded &w) { return bk_mesh_shape(w.pos.data(), (int)w.pos.size() / 3, w.idx.data(), (int)w.idx.size() / 3); };
    auto vol = [](const BKShape *x) {
      BKMesh *m = x ? bk_mesh(x, 0.05) : nullptr;
      double v = m ? m->volume : -1;
      bk_mesh_free(m);
      return v;
    };
    auto sealed = [](const BKShape *x, bool touching = false) {
      BKMesh *m = x ? bk_mesh(x, 0.05) : nullptr;
      double v;
      std::string why;
      bool ok = m && closed(m, v, why, touching) && v > 0;
      bk_mesh_free(m);
      return ok;
    };
    double cube[3] = {10, 10, 10}, ball[1] = {20};
    BKShape *cubeP = bk_primitive(BK_BOX, cube), *ballP = bk_primitive(BK_SPHERE, ball);
    Welded wc = welded(cubeP, 0.05), ws = welded(ballP, 0.05);
    BKShape *cubeM = body(wc), *ballM = body(ws);
    // The sphere as a mesh: its volume and box its mesh's exactly, and its mesh the same however fine it's asked for.
    double own = 0, lo[3] = {INFINITY, INFINITY, INFINITY}, hi[3] = {-INFINITY, -INFINITY, -INFINITY};
    for (size_t t = 0; t < ws.idx.size(); t += 3) {
      const float *a = &ws.pos[3 * ws.idx[t]], *b = &ws.pos[3 * ws.idx[t + 1]], *c = &ws.pos[3 * ws.idx[t + 2]];
      own += ((double)a[0] * ((double)b[1] * c[2] - (double)b[2] * c[1]) + (double)a[1] * ((double)b[2] * c[0] - (double)b[0] * c[2]) +
              (double)a[2] * ((double)b[0] * c[1] - (double)b[1] * c[0])) / 6;
    }
    for (size_t i = 0; i < ws.pos.size(); i++) lo[i % 3] = std::min(lo[i % 3], (double)ws.pos[i]), hi[i % 3] = std::max(hi[i % 3], (double)ws.pos[i]);
    double box[6] = {0};
    int exact = ballM ? bk_bounds(ballM, I, box) : -1;
    BKMesh *fine = ballM ? bk_mesh(ballM, 0.001) : nullptr, *coarse = ballM ? bk_mesh(ballM, 1) : nullptr;
    bool same = fine && coarse && fine->triangleCount == coarse->triangleCount && fine->triangleCount == (int)ws.idx.size() / 3 && fine->faceCount == 1 &&
                fine->edgeCount == 0;
    bk_mesh_free(fine), bk_mesh_free(coarse);
    check("mesh bodies: a sphere's mesh as a body, its volume and box the mesh's exactly, the same at any detail",
          ballM && near(vol(ballM), own, 1e-9 * own) && exact == 1 && near(box[0], lo[0], 0) && near(box[5], hi[2], 0) && same && sealed(ballM) &&
              bk_piece_count(ballM) == 1,
          ballM ? fmt("volume %.9f, the mesh's %.9f", vol(ballM), own) : bk_last_error());
    // A cube as a mesh, merged with, less and cut by plain shapes: exact volumes, closed.
    double right[12] = {1, 0, 0, 5, 0, 1, 0, 0, 0, 0, 1, 0};
    BKShape *moved = bk_transform(cubeP, right);
    BKShape *both = bk_boolean(BK_UNION, cubeM, moved), *less = bk_boolean(BK_SUBTRACT, cubeM, moved), *common = bk_boolean(BK_INTERSECT, moved, cubeM);
    double p0[3] = {0, 0, 0}, nx[3] = {1, 0, 0};
    BKShape *half = bk_split(cubeM, p0, nx, 0);
    check("mesh bodies: a cube as a mesh merged with, less, meeting and cut by plain shapes",
          near(vol(both), 1500, 1e-9) && near(vol(less), 500, 1e-9) && near(vol(common), 500, 1e-9) && near(vol(half), 500, 1e-9) && sealed(both) &&
              sealed(less) && sealed(common) && sealed(half),
          fmt("%.9f %.9f %.9f", vol(both), vol(less), vol(common)) + fmt(" %.9f", vol(half)));
    // A sphere as a mesh merged with a box poking out of it (its volume between the sphere's and the sum), and less a cube
    // wholly inside it (a void of exactly the cube).
    BKShape *lump = bk_boolean(BK_UNION, ballM, moved), *bite = bk_boolean(BK_SUBTRACT, ballM, cubeP);
    check("mesh bodies: a sphere as a mesh merged with a box, and with a cube taken from inside it",
          sealed(lump) && sealed(bite) && vol(lump) > own && vol(lump) < own + 1000 && near(vol(bite), own - 1000, 1e-9 * own) && bk_piece_count(bite) == 1,
          fmt("%.4f %.4f", vol(lump), vol(bite)));
    // Printed and written to STEP as it is.
    BKPrintMesh *pm = ballM ? bk_print_mesh(ballM) : nullptr;
    const BKShape *list[1] = {ballM};
    std::string stepPath = std::string(getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp") + "/bcad-mesh-body.step";
    int written = ballM ? bk_export_step(list, nullptr, 1, stepPath.c_str()) : 0;
    check("mesh bodies: printed and written to STEP", pm && pm->valid && near(pm->volume, own, 1e-6 * own) && written == 1,
          pm && !pm->valid ? bk_last_error() : "");
    bk_print_mesh_free(pm);
    // Two cubes apart in one mesh: two pieces. Overlapping: one solid, of their union's volume, once resolved.
    auto pair = [&](double dx) {
      Welded w = wc;
      size_t n = w.pos.size() / 3;
      for (size_t i = 0; i < n; i++) w.pos.insert(w.pos.end(), {w.pos[3 * i] + (float)dx, w.pos[3 * i + 1], w.pos[3 * i + 2]});
      for (size_t t = 0, nt = w.idx.size(); t < nt; t++) w.idx.push_back(w.idx[t] + (uint32_t)n);
      return w;
    };
    BKShape *apart = body(pair(20));
    std::string why;
    std::vector<bce::V3> pts;
    Welded ov = pair(5);
    for (size_t i = 0; i < ov.pos.size(); i += 3) pts.push_back({ov.pos[i], ov.pos[i + 1], ov.pos[i + 2]});
    auto overlapping = bce::meshModel(pts, ov.idx, why);
    bce::Solid whole = overlapping ? bce::resolved(*overlapping->mesh) : bce::Solid();
    check("mesh bodies: pieces apart count as two; overlapping ones resolve to their union",
          apart && bk_piece_count(apart) == 2 && near(vol(apart), 2000, 1e-9) && overlapping && near(overlapping->volume, 2000, 1e-9) &&
              near(whole.meshVolume(), 1500, 1e-9) && bce::pieces(whole) == 1,
          fmt("%.0f pieces, resolved %.9f", apart ? bk_piece_count(apart) : -1, whole.meshVolume()));
    // Anything not a closed solid refused, and said why.
    auto refused = [&](Welded w, const char *want) {
      BKShape *x = body(w);
      bool ok = !x && std::string(bk_last_error()) == want;
      if (!ok) printf("    %s: got %s\n", want, x ? "a body" : bk_last_error());
      bk_free(x);
      return ok;
    };
    Welded open = wc, flipped = wc, nan = wc, outside = wc, twice = wc, doubled = wc;
    open.idx.resize(open.idx.size() - 3);
    for (size_t t = 0; t < flipped.idx.size(); t += 3) std::swap(flipped.idx[t + 1], flipped.idx[t + 2]);
    nan.pos[4] = NAN;
    outside.idx[7] = (uint32_t)(outside.pos.size() / 3);
    twice.idx[1] = twice.idx[0];
    doubled.idx.insert(doubled.idx.end(), wc.idx.begin(), wc.idx.begin() + 3);
    check("mesh bodies: open, inside out, not numbers, a corner that isn't there, a corner twice, a side twice: refused",
          refused(open, "mesh: open") && refused(flipped, "mesh: inside out") && refused(nan, "mesh: points must be numbers") &&
              refused(outside, "mesh: a triangle's corner isn't one of the points") && refused(twice, "mesh: a triangle has a corner twice") &&
              refused(doubled, "mesh: two triangles run the same way along a side"));
    // A point no triangle uses (here the first): left out, the box the triangles' own.
    Welded stray = wc;
    stray.pos.insert(stray.pos.begin(), {100.f, 100.f, 100.f});
    for (uint32_t &i : stray.idx) i++;
    BKShape *strayM = body(stray);
    double sb[6] = {0};
    if (strayM) bk_bounds(strayM, I, sb);
    check("mesh bodies: a point no triangle uses is left out", strayM && near(sb[3], 5, 0) && near(vol(strayM), 1000, 1e-9) && sealed(strayM),
          fmt("box to %.3f", sb[3]));
    bk_free(strayM);
    // Turned and stretched, its box still exact.
    double turn[12] = {0, -2, 0, 7, 1, 0, 0, 0, 0, 0, 1.5, -3};
    BKShape *placed = ballM ? bk_transform(ballM, turn) : nullptr;
    double pb[6];
    int pe = placed ? bk_bounds(placed, I, pb) : -1;
    check("mesh bodies: turned and stretched, its box exact", pe == 1 && near(pb[0], 7 - 2 * hi[1], 1e-9) && near(pb[3], 7 - 2 * lo[1], 1e-9) &&
                                                               near(pb[2], -3 + 1.5 * lo[2], 1e-9),
          fmt("%.0f: %.6f %.6f", pe, pb[0], pb[3]));
    // How a big mesh body merges (measured, to know: sculpts run to 100k triangles and more).
    {
      BKShape *dense = bk_primitive(BK_SPHERE, ball);
      Welded wd = welded(dense, 0.0006);
      auto t0 = std::chrono::steady_clock::now();
      BKShape *big = body(wd);
      double made = ms(t0);
      t0 = std::chrono::steady_clock::now();
      BKShape *joined = big ? bk_boolean(BK_UNION, big, moved) : nullptr;
      double v = vol(joined);
      double merged = ms(t0);
      printf("    a %zu-triangle mesh body: made in %.0f ms, merged with a box in %.0f ms (volume %.3f)\n", wd.idx.size() / 3, made, merged, v);
      for (BKShape *x : {dense, big, joined}) bk_free(x);
    }
    for (BKShape *x : {cubeP, ballP, cubeM, ballM, moved, both, less, common, half, lump, bite, apart, placed}) bk_free(x);

    // MARK: remeshing
    // A body made again for sculpting at a detail: one closed mesh of even triangles (a sphere's surface one piece with no
    // hole, a torus's one with one), its volume the shape's to within the detail; overlapping pieces one solid, a void
    // kept, a mirrored placement the right way out; too fine said so; quick enough to use.
    printf("— remeshing\n");
    auto remeshed = [&](const BKShape *x, const double *m, double detail, int &euler, double &v, std::string &why) -> BKShape * {
      BKSculptMesh *r = x ? bk_remesh(x, m, detail) : nullptr;
      if (!r) {
        why = bk_last_error();
        return nullptr;
      }
      BKShape *y = bk_mesh_shape(r->positions, r->vertexCount, r->indices, r->triangleCount);
      euler = r->vertexCount - 3 * r->triangleCount / 2 + r->triangleCount;
      bool areas = true;
      for (int t = 0; t < r->triangleCount; t++) {
        const float *a = r->positions + 3 * r->indices[3 * t], *b = r->positions + 3 * r->indices[3 * t + 1], *c = r->positions + 3 * r->indices[3 * t + 2];
        double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2], wx = c[0] - a[0], wy = c[1] - a[1], wz = c[2] - a[2];
        double nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
        areas = areas && nx * nx + ny * ny + nz * nz > 0;
      }
      if (!areas) why = "a triangle with no area";
      if (!y) why = bk_last_error();
      v = y && areas ? vol(y) : -1;
      bk_sculpt_mesh_free(r);
      return areas ? y : (bk_free(y), nullptr);
    };
    double d30[1] = {30}, tor[3] = {0, 40, 10};
    BKShape *sphere = bk_primitive(BK_SPHERE, d30), *torus = bk_primitive(BK_TORUS, tor);
    int es = 0, et = 0;
    double vs = 0, vt = 0;
    std::string ws1, wt;
    BKShape *rs = remeshed(sphere, I, 0.5, es, vs, ws1), *rt = remeshed(torus, I, 0.5, et, vt, wt);
    double exactS = PI * 30 * 30 * 30 / 6, exactT = 2 * PI * PI * 15 * 25;
    check("remeshing: a sphere and a torus at 0.5 mm, closed, one piece each, no hole and one hole, their volumes to 0.5%",
          rs && rt && es == 2 && et == 0 && bk_piece_count(rs) == 1 && bk_piece_count(rt) == 1 && near(vs, exactS, 0.005 * exactS) && near(vt, exactT, 0.005 * exactT),
          (rs ? fmt("sphere %.2f (exact %.2f)", vs, exactS) : ws1) + (rt ? fmt(", torus %.2f (exact %.2f)", vt, exactT) : ", " + wt) +
              fmt(", Euler %.0f and %.0f", es, et));
    // Two cubes overlapping in one mesh (as a sculpt pulled through itself): one solid of their union; a cube with a
    // smaller one turned inside out within it (a void): kept.
    int eo = 0, ev = 0;
    double vo = 0, vv = 0;
    std::string wo, wv;
    BKShape *both2 = body(pair(5));
    BKShape *ro = both2 ? remeshed(both2, I, 0.25, eo, vo, wo) : nullptr;
    Welded hollowCube = wc;
    {
      size_t n = wc.pos.size() / 3;
      for (size_t i = 0; i < n; i++) hollowCube.pos.insert(hollowCube.pos.end(), {wc.pos[3 * i] / 2, wc.pos[3 * i + 1] / 2, wc.pos[3 * i + 2] / 2});
      for (size_t t = 0; t < wc.idx.size(); t += 3) hollowCube.idx.insert(hollowCube.idx.end(), {wc.idx[t] + (uint32_t)n, wc.idx[t + 2] + (uint32_t)n, wc.idx[t + 1] + (uint32_t)n});
    }
    BKShape *withVoid = body(hollowCube), *rv = withVoid ? remeshed(withVoid, I, 0.25, ev, vv, wv) : nullptr;
    check("remeshing: overlapping pieces one solid, a void kept",
          ro && bk_piece_count(ro) == 1 && eo == 2 && near(vo, 1500, 15) && rv && ev == 4 && near(vv, 1000 - 125, 10) && withVoid && near(vol(withVoid), 875, 1e-9),
          (ro ? fmt("union %.3f", vo) : wo) + (rv ? fmt(", with its void %.3f, Euler %.0f", vv, ev) : ", " + wv));
    // Mirrored, still facing out; too fine, refused and said why.
    double mirror[12] = {-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    int em = 0;
    double vm = 0;
    std::string wm;
    BKShape *rm = remeshed(torus, mirror, 0.5, em, vm, wm);
    BKSculptMesh *tooFine = bk_remesh(sphere, I, 0.005);
    std::string fineWhy = tooFine ? "" : bk_last_error();
    check("remeshing: mirrored, the right way out; too fine, refused",
          rm && near(vm, vt, 0.002 * vt) && !tooFine && fineWhy.rfind("remesh: too fine", 0) == 0, (rm ? fmt("%.2f", vm) : wm) + " · " + fineWhy);
    bk_sculpt_mesh_free(tooFine);
    // Quick enough: about 200,000 triangles in well under two seconds.
    auto t1 = std::chrono::steady_clock::now();
    BKSculptMesh *big = bk_remesh(sphere, I, 0.25);
    double took = ms(t1);
    check("remeshing: a 30 mm sphere at 0.25 mm quickly", big && big->triangleCount > 150000 && (!timed || took < 1500),
          fmt("%.0f triangles in %.0f ms", big ? big->triangleCount : 0, took));
    bk_sculpt_mesh_free(big);
    for (BKShape *x : {sphere, torus, rs, rt, both2, ro, withVoid, rv, rm}) bk_free(x);
  }

  // MARK: sculpting
  // Brushes on a body made ready for sculpting: a ray finds its surface; Draw raises by its height (carves inverted), Grab
  // carries the point under it exactly with the drag and leaves all beyond its radius be, Smooth evens out roughness, the
  // mirror does the same across x = 0; every stroke undone and done again to the bit; broken meshes refused; quick.
  {
    printf("— sculpting\n");
    using bce::V3;
    struct Ready {
      std::vector<float> pos;
      std::vector<uint32_t> idx;
    };
    auto ready = [](const BKShape *x, double detail) {
      Ready g;
      BKSculptMesh *r = x ? bk_remesh(x, I, detail) : nullptr;
      if (r) g.pos.assign(r->positions, r->positions + 3 * r->vertexCount), g.idx.assign(r->indices, r->indices + 3 * r->triangleCount);
      bk_sculpt_mesh_free(r);
      return g;
    };
    auto make = [](const Ready &g) { return bk_sculpt_new(g.pos.data(), (int)g.pos.size() / 3, g.idx.data(), (int)g.idx.size() / 3); };
    auto pt = [](const BKSculpt *s, uint32_t i) {
      const float *q = bk_sculpt_positions(s) + 3 * i;
      return V3{q[0], q[1], q[2]};
    };
    auto nearest = [&](const BKSculpt *s, V3 c) {
      uint32_t best = 0;
      double d = INFINITY;
      for (int i = 0; i < bk_sculpt_vertex_count(s); i++) {
        double e = bce::norm2(pt(s, (uint32_t)i) - c);
        if (e < d) d = e, best = (uint32_t)i;
      }
      return best;
    };
    auto stroke = [](BKSculpt *s, int brush, V3 from, std::vector<V3> to, double radius, double strength, bool mirror = false, bool invert = false) {
      double a[3] = {from.x, from.y, from.z};
      bk_sculpt_begin(s, brush, a, radius, strength, mirror, invert);
      for (V3 q : to) {
        double b[3] = {q.x, q.y, q.z};
        bk_sculpt_dab(s, b, 1);
      }
      bk_sculpt_end(s);
      return bk_sculpt_sync(s);
    };
    auto snapshot = [](const BKSculpt *s) {
      int n = 3 * bk_sculpt_vertex_count(s);
      std::vector<float> v(bk_sculpt_positions(s), bk_sculpt_positions(s) + n);
      v.insert(v.end(), bk_sculpt_normals(s), bk_sculpt_normals(s) + n);
      return v;
    };
    auto surface = [](const BKSculpt *s, double x, double y) {
      double o[3] = {x, y, 100}, down[3] = {0, 0, -1}, at[3] = {0, 0, NAN}, n[3];
      bk_sculpt_ray(s, o, down, at, n);
      return at[2];
    };

    // A ray finds the top of a 20 mm box, facing up; one pointing away misses. Draw at the top's middle raises it by a
    // tenth of the radius times the strength; inverted, carves it back down; nothing beyond the radius moves.
    double b20[3] = {20, 20, 20}, d30[1] = {30};
    BKShape *box = bk_primitive(BK_BOX, b20), *ball = bk_primitive(BK_SPHERE, d30);
    Ready rb = ready(box, 0.5);
    BKSculpt *sb = make(rb);
    double o[3] = {0.3, 0.2, 100}, down[3] = {0, 0, -1}, up[3] = {0, 0, 1}, at[3] = {0}, nrm[3] = {0}, at2[3], nrm2[3];
    int hit = sb ? bk_sculpt_ray(sb, o, down, at, nrm) : 0, missed = sb ? bk_sculpt_ray(sb, o, up, at2, nrm2) : 1;
    check("sculpting: a ray finds the surface and its normal; one pointing away misses",
          hit && near(at[0], 0.3, 1e-9) && near(at[2], 10, 0.011) && near(nrm[2], 1, 1e-6) && !missed, fmt("at z %.6f, normal z %.9f", at[2], nrm[2]));
    bool drew = false, carved = false, kept = true;
    double rose = 0, sank = 0;
    if (sb && hit) {
      uint32_t v = nearest(sb, {at[0], at[1], at[2]});
      V3 c = pt(sb, v);
      std::vector<float> before = snapshot(sb);
      int changed = stroke(sb, BK_BRUSH_DRAW, c, {c}, 3, 0.5);
      V3 after = pt(sb, v);
      rose = after.z - c.z;
      drew = changed > 0 && near(rose, 0.15, 1e-5) && near(after.x, c.x, 1e-6) && near(after.y, c.y, 1e-6);
      for (int i = 0; i < bk_sculpt_vertex_count(sb); i++) {
        V3 was{before[3 * i], before[3 * i + 1], before[3 * i + 2]};
        if (bce::norm(was - c) >= 3 && !(pt(sb, (uint32_t)i) == was)) kept = false;
      }
      stroke(sb, BK_BRUSH_DRAW, c, {c}, 3, 0.5, false, true);
      sank = pt(sb, v).z - after.z;
      carved = near(sank, -0.15, 0.01);
    }
    check("sculpting: Draw raises by its height, inverted carves, nothing beyond its radius moves", drew && carved && kept,
          fmt("rose %.6f, sank %.6f", rose, sank));

    // Grab on a 30 mm ball: the point under it carried exactly by the drag, one halfway out by the falloff, the rest kept.
    Ready rs = ready(ball, 0.5);
    BKSculpt *ss = make(rs);
    bool grabbed = false, fell = false, still = true, undid = false, redid = false, forgot = false;
    double moved = 0;
    if (ss) {
      uint32_t v = nearest(ss, {0, 0, 15});
      V3 from = pt(ss, v);
      // A point about half the radius away.
      uint32_t h = nearest(ss, {3, 0, std::sqrt(225.0 - 9)});
      V3 hFrom = pt(ss, h);
      std::vector<float> before = snapshot(ss);
      V3 drag{2, 0, 5};
      stroke(ss, BK_BRUSH_GRAB, from, {from + V3{0, 0, 3}, from + drag}, 6, 0.5);
      std::vector<float> after = snapshot(ss);
      moved = bce::norm(pt(ss, v) - (from + drag));
      grabbed = moved < 1e-5;
      double t = bce::norm2(hFrom - from) / 36, w = (1 - t) * (1 - t);
      fell = bce::norm(pt(ss, h) - (hFrom + drag * w)) < 1e-5;
      for (int i = 0; i < bk_sculpt_vertex_count(ss); i++) {
        V3 was{before[3 * i], before[3 * i + 1], before[3 * i + 2]};
        if (bce::norm(was - from) >= 6 && !(pt(ss, (uint32_t)i) == was)) still = false;
      }
      // Undone: every point and normal as it was, to the bit; done again: as it was after.
      int u = bk_sculpt_undo(ss);
      bk_sculpt_sync(ss);
      undid = u == 1 && snapshot(ss) == before && bk_sculpt_undo(ss) == 0;
      int r = bk_sculpt_redo(ss);
      bk_sculpt_sync(ss);
      redid = r == 1 && snapshot(ss) == after && bk_sculpt_redo(ss) == 0;
      // A new stroke after an undo: what was undone can't be done again.
      bk_sculpt_undo(ss);
      stroke(ss, BK_BRUSH_INFLATE, from, {from}, 4, 0.5);
      forgot = bk_sculpt_redo(ss) == 0;
    }
    check("sculpting: Grab carries the point exactly by the drag, nearer ones by the falloff, nothing beyond its radius",
          grabbed && fell && still, fmt("off by %.2g", moved));
    check("sculpting: a stroke undone and done again to the bit; a new one forgets what was undone", undid && redid && forgot);

    // Smooth: a rough ball (points nudged in and out by 0.09 mm) evened out under the brush, and only there.
    Ready rough = rs;
    for (size_t i = 0; i < rough.pos.size() / 3; i++) {
      double k = 1 + 0.006 * ((int)((uint32_t)(i * 2654435761u) >> 16) % 3 - 1);
      for (int a = 0; a < 3; a++) rough.pos[3 * i + a] = (float)(rough.pos[3 * i + a] * k);
    }
    BKSculpt *sr = make(rough);
    double was = 0, now = 0;
    bool elsewhere = true;
    if (sr) {
      size_t np = rough.pos.size() / 3;
      std::vector<std::vector<uint32_t>> nb(np);
      for (size_t t = 0; t < rough.idx.size(); t += 3)
        for (int q = 0; q < 3; q++) nb[rough.idx[t + q]].push_back(rough.idx[t + (q + 1) % 3]);
      auto roughness = [&]() {
        double sum = 0;
        int count = 0;
        for (size_t i = 0; i < np; i++) {
          V3 q = pt(sr, (uint32_t)i);
          if (bce::norm(q - V3{0, 0, 15}) > 3) continue;
          V3 m{0, 0, 0};
          for (uint32_t j : nb[i]) m += pt(sr, j);
          sum += bce::norm(m / (double)nb[i].size() - q), count++;
        }
        return count ? sum / count : 0;
      };
      std::vector<float> before = snapshot(sr);
      was = roughness();
      V3 top = pt(sr, nearest(sr, {0, 0, 15}));
      for (int k = 0; k < 3; k++) stroke(sr, BK_BRUSH_SMOOTH, top, {top}, 5, 1);
      now = roughness();
      for (size_t i = 0; i < np; i++)
        if (before[3 * i + 2] < 5 && !(pt(sr, (uint32_t)i) == V3{before[3 * i], before[3 * i + 1], before[3 * i + 2]})) elsewhere = false;
    }
    check("sculpting: Smooth evens out roughness under it, and only there", sr && now < 0.4 * was && elsewhere, fmt("%.4f → %.4f", was, now));

    // The mirror: a dab at x = 6 raises x = −6 as much; without it, x = −6 stays.
    BKSculpt *sm = make(rs);
    double left0 = 0, right0 = 0, left1 = 0, right1 = 0, left2 = 0;
    if (sm) {
      left0 = surface(sm, -6, 0.2), right0 = surface(sm, 6, 0.2);
      V3 c{6, 0.2, right0};
      stroke(sm, BK_BRUSH_DRAW, c, {c}, 3, 0.5, true);
      left1 = surface(sm, -6, 0.2), right1 = surface(sm, 6, 0.2);
      stroke(sm, BK_BRUSH_DRAW, c, {c}, 3, 0.5);
      left2 = surface(sm, -6, 0.2);
    }
    check("sculpting: the mirror does the same across x = 0",
          sm && right1 - right0 > 0.1 && near(left1 - left0, right1 - right0, 0.01) && left2 == left1,
          fmt("raised %.4f and %.4f", right1 - right0, left1 - left0) + fmt(", then %.2g", left2 - left1));

    // Broken meshes refused.
    Ready broken = rb;
    if (!broken.idx.empty()) broken.idx[4] = (uint32_t)(broken.pos.size() / 3);
    BKSculpt *bad = broken.idx.empty() ? nullptr : make(broken);
    std::string badWhy = bad ? "" : bk_last_error();
    check("sculpting: a mesh with a corner that isn't there refused", !bad && badWhy == "sculpt: a triangle's corner is missing", badWhy);

    // Quick: a stroke of 50 dabs (each found by a ray, as the app does) on a ball of more than 50,000 points.
    Ready fine = ready(ball, 0.35);
    BKSculpt *sf = make(fine);
    double each = 0;
    if (sf) {
      double a0[3] = {-12, 0, surface(sf, -12, 0)};
      auto t0 = std::chrono::steady_clock::now();
      bk_sculpt_begin(sf, BK_BRUSH_DRAW, a0, 5, 0.5, 1, 0);
      for (int k = 0; k < 50; k++) {
        double ox[3] = {-12 + 0.5 * k, 0, 100}, a[3], n[3];
        if (bk_sculpt_ray(sf, ox, down, a, n)) bk_sculpt_dab(sf, a, 1);
        bk_sculpt_sync(sf);
      }
      bk_sculpt_end(sf);
      each = ms(t0) / 50;
    }
    check("sculpting: a dab with the mirror on 50,000 points in under 2 ms", sf && bk_sculpt_vertex_count(sf) > 50000 && (!timed || each < 2),
          fmt("%.3f ms each on %.0f points", each, sf ? bk_sculpt_vertex_count(sf) : 0));

    for (BKSculpt *x : {sb, ss, sr, sm, sf}) bk_sculpt_free(x);
    bk_free(box), bk_free(ball);
  }

  // MARK: coverage
  // What the rest leaves out: distances (and the points they're between) on more shapes, sections across a corner, a face,
  // a whole body and a concave edge, edge picks of other shapes, boxes of treated shapes, picks that match nothing, the
  // oval torus merged and rounded, prisms and pyramids of every number of sides, every thread merged, stretched and
  // mirrored shapes rounded and hollowed, the smallest and largest sizes, many shapes at once, three roundings in a row.
  {
    printf("— coverage\n");
    // A sound solid as files take it: closed, its pieces as many as `pieces` (0: any).
    auto sound = [](BKShape *s, int pieces = 0) {
      if (!s) return false;
      BKPrintMesh *p = bk_print_mesh(s);
      bool ok = p->valid && p->triangleCount > 0 && (pieces == 0 || bk_piece_count(s) == pieces);
      bk_print_mesh_free(p);
      return ok;
    };
    auto vol = [](BKShape *s) {
      BKMesh *m = bk_mesh(s, 0.05);
      double v = m ? m->volume : 0;
      bk_mesh_free(m);
      return v;
    };
    auto faceWhere = [](BKShape *s, auto pred) {
      BKMesh *m = bk_mesh(s, 0.05);
      int found = -1;
      for (int f = 0; f < m->faceCount && found < 0; f++)
        if (pred(m->faceInfo + 6 * f)) found = f;
      bk_mesh_free(m);
      return found;
    };
    auto at = [](BKShape *s, double x, double y, double z) {
      double m[12] = {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z};
      return bk_transform(s, m);
    };
    int kb = BK_PICK_BODY, ke = BK_PICK_EDGE, kc = BK_PICK_CORNER, kf = BK_PICK_FACE, miss = 0;
    double body6[6] = {0}, mr = 0, out[6];
    std::string notes;
    int bad = 0;
    auto expect = [&](const char *what, bool ok, const std::string &note = "") {
      if (!ok) bad++, notes += std::string(what) + (note.empty() ? "" : ": " + note) + "\n  ";
    };

    // Distances, each with its closest points checked to lie that far apart.
    {
      double cone[3] = {20, 0, 20}, torus[3] = {0, 30, 8}, p20[3] = {20, 0, 0}, above[3] = {0, 0, 10}, b20[3] = {20, 20, 20};
      BKShape *c = bk_primitive(BK_CONE, cone), *t = bk_primitive(BK_TORUS, torus), *box = bk_primitive(BK_BOX, b20);
      int side = faceWhere(c, [](double *i) { return std::fabs(i[2]) < 0.9; });
      double d = bk_distance(nullptr, nullptr, BK_END_POINT, 0, p20, c, I, BK_END_FACE, side, nullptr, out);
      // The cone's side runs from (10, -10) to (0, 10) in its xz plane.
      expect("point to a cone's side", near(d, 300 / std::sqrt(500.0), 1e-9) && near(std::hypot(out[3] - out[0], out[4] - out[1], out[5] - out[2]), d, 1e-9),
             fmt("%.12f", d));
      d = bk_distance(nullptr, nullptr, BK_END_POINT, 0, above, t, I, BK_END_FACE, 0, nullptr, out);
      expect("point to a torus", near(d, std::sqrt(121.0 + 100) - 4, 1e-9) && near(std::hypot(out[3] - out[0], out[4] - out[1], out[5] - out[2]), d, 1e-9),
             fmt("%.12f", d));
      BKFastener f{};
      f.kind = BK_HEX, f.size = 4;
      bk_fastener_defaults(&f, 1);
      BKShape *bolt = bk_fastener(&f, 0.2);
      double bb[6];
      bk_bounds(bolt, I, bb);
      double over[3] = {0, 0, bb[5] + 5};
      int headTop = faceWhere(bolt, [&](double *i) { return i[2] > 0.99 && near(i[5], bb[5], 1e-6); });
      d = bk_distance(nullptr, nullptr, BK_END_POINT, 0, over, bolt, I, BK_END_FACE, headTop, nullptr, out);
      expect("point to a bolt head's top", near(d, 5, 1e-9), fmt("%.12f", d));
      // Rounded edge: the point (15, 15, 0) to the edge rounded with radius 2 along z at (8, 8).
      BKShape *r = bk_fillet(box, &kb, body6, 1, 2, &mr, &miss);
      double corner[3] = {15, 15, 0}, best = 1e9;
      BKMesh *rm = bk_mesh(r, 0.05);
      for (int fi = 0; rm && fi < rm->faceCount; fi++) {
        double g = bk_distance(nullptr, nullptr, BK_END_POINT, 0, corner, r, I, BK_END_FACE, fi, nullptr, out);
        if (g >= 0) best = std::min(best, g);
      }
      bk_mesh_free(rm);
      expect("point to a rounded edge", near(best, std::sqrt(98.0) - 2, 2e-3), fmt("%.6f", best));
      // Shapes overlapping: a small box's top crosses the big one's side, nothing between them.
      double b10[3] = {10, 10, 10};
      BKShape *small = bk_primitive(BK_BOX, b10), *moved = at(small, 10, 0, 0);
      int px = faceWhere(box, [](double *i) { return i[0] > 0.9; }), pz = faceWhere(small, [](double *i) { return i[2] > 0.9; });
      d = bk_distance(box, I, BK_END_FACE, px, nullptr, moved, I, BK_END_FACE, pz, nullptr, out);
      expect("faces crossing", d >= 0 && d <= 1e-9, fmt("%.12f", d));
      bk_free(small);
      for (BKShape *x : {c, t, box, bolt, r, moved}) bk_free(x);
    }

    // Sections across a corner, a face, a whole body and a concave edge (an L of two boxes).
    {
      double b20[3] = {20, 20, 20}, corner[6] = {0, 0, 1, 10, 10, 10}, top[6] = {0, 0, 1, 0, 0, 10};
      BKShape *box = bk_primitive(BK_BOX, b20), *other = at(box, 20, 0, 0), *tall = at(box, 0, 0, 20), *l = bk_boolean(BK_UNION, other, tall);
      BKShape *ell = bk_boolean(BK_UNION, box, l);
      struct S {
        const char *name;
        BKShape *s;
        int kind;
        double *pick;
        double angle;
      };
      // The L's concave edge: along y at x = 10, z = 10.
      double concave[6] = {10, 0, 10, 0, 1, 0};
      for (S x : {S{"a corner", box, kc, corner, 90}, S{"a face", box, kf, top, 90}, S{"a body", box, kb, body6, 90}, S{"a concave edge", ell, ke, concave, 270}}) {
        BKSection *sec = bk_section(x.s, x.kind, x.pick, 10);
        bool ok = sec && near(sec->angle, x.angle, 1e-9) && sec->loopCount >= 1;
        if (ok) {
          // Its point on an edge of the shape, its direction along an axis.
          double dir = std::max({std::fabs(sec->direction[0]), std::fabs(sec->direction[1]), std::fabs(sec->direction[2])});
          ok = near(dir, 1, 1e-9);
        }
        expect((std::string("section across ") + x.name).c_str(), ok, sec ? fmt("%.6f°", sec->angle) : bk_last_error());
        if (sec) bk_section_free(sec);
      }
      for (BKShape *x : {box, other, tall, l, ell}) bk_free(x);
    }

    // Edge picks of other shapes: a cylinder's body is its two rims, a wedge's its 9 edges.
    {
      double cyl[2] = {20, 20}, wedge[3] = {30, 20, 10};
      BKShape *c = bk_primitive(BK_CYLINDER, cyl), *w = bk_primitive(BK_WEDGE, wedge);
      int rims = bk_pick_edges(c, &kb, body6, 1, nullptr, 0), edges = bk_pick_edges(w, &kb, body6, 1, nullptr, 0);
      expect("edge picks of a cylinder and a wedge", rims == 2 && edges == 9, fmt("%.0f rims, %.0f edges", rims, edges));
      // A pick that matches nothing is said to, and the rest still rounded.
      double picks[12] = {0, -10, 10, 1, 0, 0, 500, 500, 500, 1, 0, 0}, b20[3] = {20, 20, 20};
      int kinds[2] = {BK_PICK_EDGE, BK_PICK_EDGE};
      BKShape *box = bk_primitive(BK_BOX, b20), *one = bk_fillet(box, kinds, picks, 2, 2, &mr, &miss);
      expect("a pick matching nothing is said to be missing", one && miss == 1 && sound(one, 1), fmt("missing %.0f", miss));
      for (BKShape *x : {c, w, box, one}) bk_free(x);
    }

    // Boxes of treated and hollowed shapes (rounding and hollowing keep a box's box), and of a bolt turned aslant.
    {
      double b20[3] = {20, 20, 20}, bb[6];
      BKShape *box = bk_primitive(BK_BOX, b20), *r = bk_fillet(box, &kb, body6, 1, 2, &mr, &miss);
      BKShape *h = bk_hollow(box, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, 2, nullptr);
      for (BKShape *x : {r, h}) {
        int exact = bk_bounds(x, I, bb);
        bool ok = exact >= 0;
        for (int k = 0; k < 6; k++) ok = ok && near(bb[k], k < 3 ? -10 : 10, 0.1);
        expect(x == r ? "a rounded box's box" : "a hollow box's box", ok, fmt("%.4f … %.4f", bb[0], bb[3]));
      }
      BKFastener f{};
      f.kind = BK_SOCKET, f.size = 4;
      bk_fastener_defaults(&f, 1);
      BKShape *bolt = bk_fastener(&f, 0.2);
      double turn[12] = {1, 0, 0, 0, 0, std::cos(0.5), -std::sin(0.5), 0, 0, std::sin(0.5), std::cos(0.5), 0};
      BKShape *aslant = bk_transform(bolt, turn);
      bk_bounds(bolt, turn, bb);
      BKMesh *m = bk_mesh(aslant, 0.01);
      double lo = 1e9, hi = -1e9;
      for (int i = 0; m && i < m->vertexCount; i++) lo = std::min(lo, (double)m->positions[3 * i + 2]), hi = std::max(hi, (double)m->positions[3 * i + 2]);
      bk_mesh_free(m);
      expect("an aslant bolt's box", near(bb[2], lo, 0.1) && near(bb[5], hi, 0.1), fmt("%.4f … %.4f", bb[2], bb[5]) + fmt(" (mesh %.4f … %.4f)", lo, hi));
      for (BKShape *x : {box, r, h, bolt, aslant}) bk_free(x);
    }

    // The oval torus merged with a box and rounded where it meets it; every prism and pyramid, 3 to 24 sides.
    {
      double ot[5] = {0, 30, 20, 90, 6}, slab[3] = {40, 10, 4};
      BKShape *o = bk_primitive(BK_OVAL_TORUS, ot), *s = bk_primitive(BK_BOX, slab), *both = bk_boolean(BK_UNION, o, s);
      expect("an oval torus merged with a slab", sound(both, 1));
      BKShape *r = both ? bk_fillet(both, &kb, body6, 1, 0.5, &mr, &miss) : nullptr;
      expect("an oval torus and a slab rounded", !r || sound(r, 1), r ? "" : fmt("refused (largest %.3f)", mr));
      for (BKShape *x : {o, s, both, r}) bk_free(x);
      for (int n = 3; n <= 24; n++)
        for (int kind : {BK_PRISM, BK_PYRAMID}) {
          double p[3] = {(double)n, 20, 20};
          BKShape *x = bk_primitive(kind, p);
          double area = n / 2.0 * 100 * std::sin(2 * PI / n), want = kind == BK_PRISM ? area * 20 : area * 20 / 3;
          expect(fmt(kind == BK_PRISM ? "%.0f-sided prism" : "%.0f-sided pyramid", n).c_str(), sound(x, 1) && near(vol(x), want, 1e-6 * want), fmt("%.6f", vol(x)));
          bk_free(x);
        }
    }

    // Every thread merged with a block and cut from one.
    {
      for (int size = 0; size < bk_thread_count(); size++) {
        BKFastener f{};
        f.kind = BK_HEX, f.size = size;
        bk_fastener_defaults(&f, 1);
        BKShape *bolt = bk_fastener(&f, 0.2);
        double bb[6];
        bk_bounds(bolt, I, bb);
        double block[3] = {bb[3] - bb[0] + 10, bb[4] - bb[1] + 10, 6};
        BKShape *b = bk_primitive(BK_BOX, block), *merged = bk_boolean(BK_UNION, bolt, b), *cut = bk_boolean(BK_SUBTRACT, b, bolt);
        expect((std::string(bk_thread_name(size)) + " merged and cut").c_str(), sound(merged, 1) && sound(cut));
        for (BKShape *x : {bolt, b, merged, cut}) bk_free(x);
      }
    }

    // Stretched and mirrored shapes rounded and hollowed: a mirror changes nothing; a stretch is an oval to round.
    {
      double b20[3] = {20, 20, 20}, cyl[2] = {20, 20};
      double mirror[12] = {-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}, stretch[12] = {1.5, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
      BKShape *box = bk_primitive(BK_BOX, b20), *c = bk_primitive(BK_CYLINDER, cyl);
      BKShape *mb = bk_transform(box, mirror), *sc = bk_transform(c, stretch);
      BKShape *h = bk_hollow(mb, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, 2, nullptr), *h0 = bk_hollow(box, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, 2, nullptr);
      expect("a mirrored box hollowed as the box", sound(h, 1) && near(vol(h), vol(h0), 1e-9 * vol(h0)), fmt("%.6f vs %.6f", vol(h), vol(h0)));
      BKShape *rm = bk_fillet(mb, &kb, body6, 1, 2, &mr, &miss), *r0 = bk_fillet(box, &kb, body6, 1, 2, &mr, &miss);
      expect("a mirrored box rounded as the box", sound(rm, 1) && near(vol(rm), vol(r0), 1e-9 * vol(r0)));
      BKShape *rs = bk_fillet(sc, &kb, body6, 1, 2, &mr, &miss), *hs = bk_hollow(sc, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, 2, nullptr);
      expect("a stretched cylinder rounded", sound(rs, 1), rs ? "" : bk_last_error());
      expect("a stretched cylinder hollowed", sound(hs, 1) && vol(hs) < vol(sc), hs ? "" : bk_last_error());
      for (BKShape *x : {box, c, mb, sc, h, h0, rm, r0, rs, hs}) bk_free(x);
    }

    // The smallest and largest sizes: a 0.01 mm cube, a 1 m box and ball.
    {
      double tiny[3] = {0.01, 0.01, 0.01}, metre[3] = {1000, 1000, 1000}, ball[1] = {1000};
      BKShape *t = bk_primitive(BK_BOX, tiny), *m = bk_primitive(BK_BOX, metre), *s = bk_primitive(BK_SPHERE, ball);
      expect("a 0.01 mm cube", sound(t, 1) && near(vol(t), 1e-6, 1e-12), fmt("%.3g", vol(t)));
      expect("a 1 m box", sound(m, 1) && near(vol(m), 1e9, 1e-3), fmt("%.6g", vol(m)));
      expect("a 1 m ball", sound(s, 1) && near(vol(s), 4 * PI * 500 * 500 * 500 / 3, 1e-6 * 5e8), fmt("%.6g", vol(s)));
      for (BKShape *x : {t, m, s}) bk_free(x);
    }

    // Twelve cubes in a row, each touching the next, merged one by one: one piece, twelve cubes' volume.
    {
      double cube[3] = {10, 10, 10};
      BKShape *k = bk_primitive(BK_BOX, cube), *row = bk_copy(k);
      for (int i = 1; i < 12 && row; i++) {
        BKShape *next = at(k, 10.0 * i, 0, 0), *joined = bk_boolean(BK_UNION, row, next);
        bk_free(row), bk_free(next);
        row = joined;
      }
      expect("twelve cubes in a row merged", sound(row, 1) && near(vol(row), 12000, 1e-6), row ? fmt("%.6f", vol(row)) : "failed");
      bk_free(k), bk_free(row);
    }

    // Three roundings in a row, each on an edge of its own.
    {
      double b20[3] = {20, 20, 20}, e1[6] = {0, -10, 10, 1, 0, 0}, e2[6] = {0, 10, 10, 1, 0, 0}, e3[6] = {10, 0, -10, 0, 1, 0};
      BKShape *box = bk_primitive(BK_BOX, b20), *a = bk_fillet(box, &ke, e1, 1, 2, &mr, &miss);
      BKShape *b = a ? bk_fillet(a, &ke, e2, 1, 2, &mr, &miss) : nullptr, *c = b ? bk_fillet(b, &ke, e3, 1, 2, &mr, &miss) : nullptr;
      double one = (4 - PI) * 20;
      expect("three roundings in a row", sound(c, 1) && near(vol(c), 8000 - 3 * one, 1e-6), c ? fmt("%.6f", vol(c)) : "refused");
      for (BKShape *x : {box, a, b, c}) bk_free(x);
    }
    check("coverage: distances, sections, picks, boxes, oval torus, 3–24 sides, every thread, mirrors, sizes, rows, roundings", bad == 0, notes);
  }

  // MARK: speed
  {
    double sph[1] = {200}, cyl[2] = {20, 20};
    BKShape *s = bk_primitive(BK_SPHERE, sph), *c = bk_primitive(BK_CYLINDER, cyl);
    auto t0 = std::chrono::steady_clock::now();
    int tris = 0;
    for (int i = 0; i < 100; i++) {
      BKMesh *m = bk_mesh(c, 0.05);
      tris += m->triangleCount;
      bk_mesh_free(m);
    }
    double cylTook = ms(t0) / 100;
    t0 = std::chrono::steady_clock::now();
    BKMesh *m = bk_mesh(s, 0.01);
    double sphTook = ms(t0);
    printf("  a cylinder meshed in %.3f ms (%d triangles); a 200 mm sphere at 0.01 mm in %.1f ms (%d triangles)\n", cylTook, tris / 100, sphTook, m->triangleCount);
    check("meshing is quick", !timed || (cylTook < 5 && sphTook < 2000));
    bk_mesh_free(m);
    bk_free(s), bk_free(c);
  }

  // Stability: what can't be made again, nothing to work on, and whether a mesh is a closed solid.
  {
    printf("— stability\n");
    // A rounding made once (and shown) that fails at a finer detail: there it is as shown, not the shape untreated.
    bce::Shape box, small;
    std::string why;
    double b20[3] = {20, 20, 20}, b10[3] = {10, 10, 10};
    bce::primitive(BK_BOX, b20, box, why), bce::primitive(BK_BOX, b10, small, why);
    auto node = std::make_shared<bce::Node>();
    node->kind = bce::Node::Treat, node->a = box;
    bce::Treatment tooBig;
    tooBig.kind = bce::Treatment::Round, tooBig.radius = 40, tooBig.kinds = {BK_PICK_EDGE}, tooBig.picks = {0, -10, 10, 1, 0, 0};
    node->treat = std::make_shared<bce::Treatment>(tooBig);
    bce::Solid shownMesh;
    bce::mesh(small, 0.05, shownMesh);
    node->shown = std::make_shared<bce::Solid>(shownMesh);
    auto fine = bce::evaluate(*node, 0.01);
    check("a treatment that fails at another detail stands as it was shown", near(fine->meshVolume(), 1000, 1e-6), fmt("%.4f mm³", fine->meshVolume()));
    // Nothing to hollow (two boxes apart, their common part): refused, not a crash.
    double far[12] = {1, 0, 0, 100, 0, 1, 0, 0, 0, 0, 1, 0};
    BKShape *a = bk_primitive(BK_BOX, b20), *b0 = bk_primitive(BK_BOX, b20), *b = bk_transform(b0, far);
    BKShape *none = bk_boolean(BK_INTERSECT, a, b);
    BKShape *hollowNone = none ? bk_hollow(none, nullptr, 0, nullptr, 0, nullptr, nullptr, 0, 2, nullptr) : nullptr;
    check("nothing to hollow is refused", none && !hollowNone);
    bk_free(hollowNone), bk_free(none), bk_free(a), bk_free(b0), bk_free(b);
    // A closed mesh says so; the result of every merge is one.
    int kb = BK_PICK_BODY;
    double body[6] = {0, 0, 0, 0, 0, 0}, mr;
    int miss;
    BKShape *cube = bk_primitive(BK_BOX, b20), *rounded = bk_fillet(cube, &kb, body, 1, 2, &mr, &miss);
    BKMesh *m = rounded ? bk_mesh(rounded, 0.05) : nullptr;
    check("a rounded box's mesh is a closed solid", m && m->valid == 1);
    bk_mesh_free(m), bk_free(rounded), bk_free(cube);
    // Where a part sits doesn't change what's made of it: a small box far from the middle is one piece, rounded the same.
    double b2[3] = {2, 2, 2}, away[12] = {1, 0, 0, 5000, 0, 1, 0, -3000, 0, 0, 1, 2000}, bodyAway[6] = {5000, -3000, 2000, 0, 0, 0};
    BKShape *tiny = bk_primitive(BK_BOX, b2), *tinyAway = bk_transform(tiny, away);
    check("a small part far from the middle is one piece", tinyAway && bk_piece_count(tinyAway) == 1);
    BKShape *r0 = bk_fillet(tiny, &kb, body, 1, 0.4, &mr, &miss), *r1 = tinyAway ? bk_fillet(tinyAway, &kb, bodyAway, 1, 0.4, &mr, &miss) : nullptr;
    auto volumeOf = [](BKShape *s) {
      BKMesh *k = bk_mesh(s, 0.05);
      double v = k ? k->volume : -1;
      bk_mesh_free(k);
      return v;
    };
    double v0 = r0 ? volumeOf(r0) : 0, v1 = r1 ? volumeOf(r1) : -1;
    check("rounded far from the middle as at it", r0 && r1 && near(v0, v1, 1e-6 * v0), fmt("%.6f vs %.6f mm³", v0, v1));
    bk_free(r1), bk_free(r0), bk_free(tinyAway), bk_free(tiny);
  }

  printf(failures ? "FAILURES: %d\n" : "ALL OK\n", failures);
  return failures ? 1 : 0;
}
