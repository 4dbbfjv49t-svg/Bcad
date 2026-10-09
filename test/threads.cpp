// Bcad's engine from several threads at once, as the app calls it (its worker and the main thread): figures made
// together (their cache shared), one shape meshed, boxed and counted from every thread, each thread's last error its own,
// scans repaired together; everything as when made alone. And a very deep tree of merges worked out on a thread with the
// main thread's stack.
// Run under ThreadSanitizer: c++ -std=c++17 -O1 -g -fsanitize=thread -I. test/threads.cpp Engine/*.cpp -o threads && ./threads
#include "BcadKernel.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <string>
#include <thread>
#include <vector>

static int failures = 0;
static void check(const char *name, bool ok, const std::string &note = "") {
  printf("%s %s %s\n", ok ? "✓" : "✗", name, note.c_str());
  fflush(stdout);
  if (!ok) failures++;
}
static const double I[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};

// Every bit of a shape's display mesh.
static uint64_t hashOf(const BKShape *s) {
  BKMesh *m = bk_mesh(s, 0.05);
  uint64_t h = 1469598103934665603ull;
  auto mix = [&](const void *d, size_t n) {
    for (size_t i = 0; i < n; i++) h = (h ^ ((const unsigned char *)d)[i]) * 1099511628211ull;
  };
  if (m) mix(m->positions, 12 * (size_t)m->vertexCount), mix(m->indices, 12 * (size_t)m->triangleCount);
  bk_mesh_free(m);
  return h;
}

static std::vector<double> figureNumbers(int k) {
  std::vector<double> p(BK_FIG_COUNT);
  bk_figure_defaults(k % 2, p.data());
  bk_figure_pose(k % BK_POSE_COUNT, p.data());
  p[BK_FIG_HEIGHT] = 80 + 10 * (k % 3);
  return p;
}

int main() {
  const int threads = 8;
  // Figures: made alone first, then by every thread at once, in different orders (the cache of recent ones shared).
  {
    std::vector<uint64_t> alone(6);
    for (int k = 0; k < 6; k++) {
      std::vector<double> p = figureNumbers(k);
      BKShape *s = bk_figure(p.data(), BK_FIG_COUNT, 1);
      alone[k] = s ? hashOf(s) : 0;
      bk_free(s);
    }
    std::atomic<int> wrong{0};
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; t++)
      pool.emplace_back([&, t] {
        for (int round = 0; round < 3; round++)
          for (int j = 0; j < 6; j++) {
            int k = (j + t) % 6;
            std::vector<double> p = figureNumbers(k);
            BKShape *s = bk_figure(p.data(), BK_FIG_COUNT, 1);
            double ext[6];
            if (!s || hashOf(s) != alone[k] || !bk_figure_extent(p.data(), BK_FIG_COUNT, ext)) wrong++;
            bk_free(s);
          }
      });
    for (auto &th : pool) th.join();
    check("threads: figures made together come out as made alone", wrong == 0 && alone[0] != 0, std::to_string(wrong.load()) + " wrong");
  }

  // One shape (a merge with a rounding, so caches fill as it's asked) meshed, boxed and counted from every thread.
  {
    double b[3] = {20, 20, 20}, c[2] = {12, 30};
    BKShape *box = bk_primitive(BK_BOX, b), *cyl = bk_primitive(BK_CYLINDER, c);
    BKShape *merged = bk_boolean(BK_UNION, box, cyl);
    int kinds[1] = {BK_PICK_BODY};
    double picks[6] = {0}, maxR;
    int missing;
    BKShape *round = bk_fillet(box, kinds, picks, 1, 2, &maxR, &missing);
    BKShape *shared[2] = {bk_boolean(BK_SUBTRACT, merged, round ? round : cyl), bk_copy(merged)};
    // Alone (on copies, so the shared ones' caches are still empty).
    uint64_t alone[2];
    double bbAlone[2][6];
    int piecesAlone[2];
    for (int i = 0; i < 2; i++) {
      BKShape *copy = bk_copy(shared[i]);
      alone[i] = hashOf(copy), bk_bounds(copy, I, bbAlone[i]), piecesAlone[i] = bk_piece_count(copy);
      bk_free(copy);
    }
    std::atomic<int> wrong{0};
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; t++)
      pool.emplace_back([&, t] {
        for (int round = 0; round < 4; round++) {
          int i = (t + round) % 2;
          double bb[6];
          switch ((t + round) % 3) {
            case 0:
              if (hashOf(shared[i]) != alone[i]) wrong++;
              break;
            case 1:
              bk_bounds(shared[i], I, bb);
              if (memcmp(bb, bbAlone[i], sizeof bb)) wrong++;
              break;
            default:
              if (bk_piece_count(shared[i]) != piecesAlone[i]) wrong++;
          }
          BKPrintMesh *pm = bk_print_mesh(shared[i]);
          if (!pm || !pm->valid) wrong++;
          bk_print_mesh_free(pm);
        }
      });
    for (auto &th : pool) th.join();
    check("threads: one shape meshed, boxed, counted and printed from every thread at once, as when alone", wrong == 0, std::to_string(wrong.load()) + " wrong");
    for (BKShape *s : {box, cyl, merged, round, shared[0], shared[1]}) bk_free(s);
  }

  // Each thread's last error its own.
  {
    std::atomic<int> wrong{0};
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; t++)
      pool.emplace_back([&, t] {
        for (int k = 0; k < 200; k++) {
          double bad[3] = {t % 2 ? -1.0 : NAN, 1, 1};
          BKShape *s = bk_primitive(BK_BOX, bad);
          std::string why = bk_last_error();
          if (s || why != (t % 2 ? "shape: sizes can't be below zero" : "shape: sizes must be numbers")) wrong++;
          bk_free(s);
        }
      });
    for (auto &th : pool) th.join();
    check("threads: each thread's last error is its own", wrong == 0, std::to_string(wrong.load()) + " wrong");
  }

  // Scans repaired on four threads at once (a ball's mesh with holes, triangles turned round, as a soup; and two of it
  // overlapping): each to the bit as repaired alone.
  {
    const double d[1] = {30};
    BKShape *ball = bk_primitive(BK_SPHERE, d);
    BKSculptMesh *m = bk_remesh(ball, I, 1.5);
    std::vector<std::vector<float>> pos(2);
    std::vector<std::vector<uint32_t>> idx(2);
    for (int k = 0; k < 2 && m; k++) {
      for (int i = 0; i < 3 * m->triangleCount; i++) {
        if (i % 3 == 0 && (i / 3) % 37 == 5) {
          i += 2;
          continue;
        }
        uint32_t v = m->indices[i / 3 * 3 + ((i / 3) % 11 == 0 ? (3 - i % 3) % 3 : i % 3)];
        idx[k].push_back((uint32_t)(pos[k].size() / 3));
        for (int c = 0; c < 3; c++) pos[k].push_back(m->positions[3 * v + c] + (k && c == 0 && i >= 3 * m->triangleCount / 2 ? 9.0f : 0.0f));
      }
    }
    auto repairHash = [&](int k) {
      BKSculptMesh *r = bk_scan_repair(pos[k].data(), (int)pos[k].size() / 3, idx[k].data(), (int)idx[k].size() / 3, nullptr, nullptr);
      uint64_t h = 1469598103934665603ull;
      auto mix = [&](const void *data, size_t n) {
        for (size_t i = 0; i < n; i++) h = (h ^ ((const unsigned char *)data)[i]) * 1099511628211ull;
      };
      if (r) mix(r->positions, 12 * (size_t)r->vertexCount), mix(r->indices, 12 * (size_t)r->triangleCount);
      bk_sculpt_mesh_free(r);
      return r ? h : 0;
    };
    uint64_t alone[2] = {repairHash(0), repairHash(1)};
    std::atomic<int> wrong{0};
    std::vector<std::thread> pool;
    for (int t = 0; t < 4; t++)
      pool.emplace_back([&, t] {
        if (repairHash(t % 2) != alone[t % 2]) wrong++;
      });
    for (auto &th : pool) th.join();
    check("threads: scans repaired on four threads at once, each as repaired alone", wrong == 0 && alone[0] && alone[1], std::to_string(wrong.load()) + " wrong");
    bk_sculpt_mesh_free(m), bk_free(ball);
  }

  // A sketch's regions found, its rules solved and a plate with holes stood up and a ring turned from it on eight
  // threads at once: each to the bit as alone.
  {
    // A 40 × 20 rectangle (its sides level and upright, one corner held, its width and height set) with two circles in it.
    std::vector<double> pts0 = {0, 0, 39, 1, 41, 21, -1, 19, 10, 10, 30, 10, -5, 0, -5, 1};
    std::vector<BKCurve> curves = {{BK_CURVE_LINE, {0, 1, -1}, 0, 0}, {BK_CURVE_LINE, {1, 2, -1}, 0, 0}, {BK_CURVE_LINE, {2, 3, -1}, 0, 0},
                                   {BK_CURVE_LINE, {3, 0, -1}, 0, 0}, {BK_CURVE_CIRCLE, {4, -1, -1}, 3, 0},  {BK_CURVE_CIRCLE, {5, -1, -1}, 3, 0},
                                   {BK_CURVE_LINE, {6, 7, -1}, 0, BK_CURVE_CONSTRUCTION}};
    std::vector<BKRule> rules = {{BK_RULE_HORIZONTAL, {-1, -1, -1}, {0, -1}, 0, 1}, {BK_RULE_VERTICAL, {-1, -1, -1}, {1, -1}, 0, 1},
                                 {BK_RULE_HORIZONTAL, {-1, -1, -1}, {2, -1}, 0, 1}, {BK_RULE_VERTICAL, {-1, -1, -1}, {3, -1}, 0, 1},
                                 {BK_RULE_FIX, {0, -1, -1}, {-1, -1}, 0, 1},         {BK_DIM_LENGTH, {-1, -1, -1}, {0, -1}, 40, 1},
                                 {BK_DIM_LENGTH, {-1, -1, -1}, {1, -1}, 20, 1}};
    auto made = [&]() -> uint64_t {
      std::vector<double> pts = pts0;
      std::vector<BKCurve> cs = curves;
      BKSketch s{(int)pts.size() / 2, (int)cs.size(), (int)rules.size(), pts.data(), nullptr, cs.data(), rules.data()};
      BKSolveReport r{};
      if (bk_sketch_solve(&s, 0, nullptr, nullptr, &r) != BK_SOLVE_OK || r.freedom != 10) return 0;
      BKRegions *g = bk_sketch_regions(&s, 0.05);
      if (!g) return 0;
      uint64_t h = 1469598103934665603ull;
      auto mix = [&](const void *d, size_t n) {
        for (size_t i = 0; i < n; i++) h = (h ^ ((const unsigned char *)d)[i]) * 1099511628211ull;
      };
      mix(pts.data(), pts.size() * sizeof(double));
      mix(g->area, g->regionCount * sizeof(double)), mix(g->seed, 2 * g->regionCount * sizeof(double));
      bk_sketch_regions_free(g);
      int start[2] = {0, 0};
      double seed[2] = {20, 2};
      BKForm forms[2] = {{BK_FORM_EXTRUDE, 0, 5, -1}, {BK_FORM_REVOLVE, 0, 120, 6}};
      for (const BKForm &f : forms) {
        BKShape *solid = bk_sketch_solid(&s, start, nullptr, seed, 1, &f);
        if (!solid) return 0;
        uint64_t m = hashOf(solid);
        mix(&m, sizeof m);
        bk_free(solid);
      }
      return h;
    };
    uint64_t alone = made();
    std::atomic<int> wrong{0};
    std::vector<std::thread> pool;
    for (int t = 0; t < 8; t++)
      pool.emplace_back([&] {
        if (made() != alone) wrong++;
      });
    for (auto &t : pool) t.join();
    check("threads: a sketch solved, its regions found and solids made of it on eight threads at once, as alone", wrong == 0 && alone != 0,
          std::to_string(wrong.load()) + " wrong");
  }

  // A tree of merges 400 deep, worked out on a thread with the main thread's stack (8 MB): meshed, boxed, counted.
  {
    const int depth = 400;
    double b[3] = {2, 2, 2};
    BKShape *cube = bk_primitive(BK_BOX, b);
    BKShape *tree = bk_copy(cube);
    for (int k = 1; k < depth; k++) {
      double m[12] = {1, 0, 0, 1.5 * k, 0, 1, 0, 0, 0, 0, 1, 0};
      BKShape *moved = bk_transform(cube, m);
      BKShape *next = bk_boolean(BK_UNION, tree, moved);
      bk_free(moved), bk_free(tree);
      tree = next;
    }
    struct Job {
      BKShape *tree;
      double volume = 0, bb[6] = {0};
      int pieces = 0;
    } job{tree};
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 8 << 20);
    pthread_t th;
    pthread_create(
        &th, &attr,
        [](void *a) -> void * {
          Job *j = (Job *)a;
          BKMesh *m = bk_mesh(j->tree, 0.05);
          j->volume = m ? m->volume : 0;
          bk_mesh_free(m);
          bk_bounds(j->tree, I, j->bb);
          j->pieces = bk_piece_count(j->tree);
          return nullptr;
        },
        &job);
    pthread_join(th, nullptr);
    pthread_attr_destroy(&attr);
    check("threads: a tree of merges 400 deep worked out on an 8 MB stack", std::fabs(job.volume - 4 * (1.5 * (depth - 1) + 2)) < 1e-6 * depth && job.pieces == 1 &&
                                                                               std::fabs(job.bb[3] - (1.5 * (depth - 1) + 1)) < 1e-9,
          "volume " + std::to_string(job.volume) + ", " + std::to_string(job.pieces) + " pieces");
    bk_free(tree), bk_free(cube);
  }
  printf(failures ? "%d FAILED\n" : "ALL OK\n", failures);
  return failures ? 1 : 0;
}
