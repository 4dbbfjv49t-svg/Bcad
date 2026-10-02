// Bcad's own geometry engine against exact maths: volumes and boxes from formulas, closed outward meshes, the faces,
// edges, corners and circles the app works with, placements, merging and splitting (again and again), distances, exact
// orientation, and how fast it all is.
// c++ -std=c++17 -O2 -I. test/engine.cpp Engine/*.cpp -o engine-test && ./engine-test
#include "BcadKernel.h"
#include "Engine/Math.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <tuple>
#include <vector>

static int failures = 0;
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
  int faces, edges, corners, circles;  // as OpenCascade's own topology for the same shape (what the app expects)
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
    g = look(keep(bk_boolean(BK_SUBTRACT, box, box2)));
    check("subtract: a corner taken out", g.shut && near(g.volume, 7000, 1e-9) && g.faces == 9 && g.pieces == 1 && boxIs(g, {-10, -10, -10}, {10, 10, 10}), says(g));
    g = look(keep(bk_boolean(BK_INTERSECT, box, box2)));
    check("intersect: the shared corner", g.shut && near(g.volume, 1000, 1e-9) && g.faces == 6 && boxIs(g, {0, 0, 0}, {10, 10, 10}), says(g));

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
      check("merging and splitting are quick", first < 50 && again < first / 5 && split < 50);
    }
    for (BKShape *s : made) bk_free(s);
    bk_free(box), bk_free(box2), bk_free(cyl);
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
    check("meshing is quick", cylTook < 5 && sphTook < 2000);
    bk_mesh_free(m);
    bk_free(s), bk_free(c);
  }

  printf(failures ? "FAILURES: %d\n" : "ALL OK\n", failures);
  return failures ? 1 : 0;
}
