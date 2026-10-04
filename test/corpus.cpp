// Bcad's own engine on a corpus of saved cases (test/corpus/cases.txt), each checked against what it's expected to give.
// A case is a program building a shape (primitives, moves and turns, merges, cuts) and operations on it (roundings,
// inward roundings, bevels, hollows, a cut across an edge):
//   id|program|operations|expected
//   program:    P kind sizes… ;   N kind thread length clearance ;   T m00 m01 m02 tx m10 … tz ;   B op ;
//               S px py pz nx ny nz side ;   M detail ;   (N: a bolt or nut, its other sizes standard; length 0 the usual one;
//               M: the shape so far as a mesh body, its mesh at that detail as a sculpt holds it; R detail ; it remeshed for
//               sculpting at that detail)
//   operations: F r n picks…   V r n picks…   C legA legB corner n picks…   H t n opens… m walls… (each 6 numbers and a
//               thickness)   X kind pick   (a pick: kind and 6 numbers)
//   expected:   made VOLUME TOLERANCE | refused | sec AREA TOLERANCE | any
// Every shape made is also made as printers take it (bk_print_mesh): it must be a sound solid, its volume the shape's.
// The digest carries, for each case as saved, a hash of that mesh's every bit and of its STEP file's, so two machines'
// files are seen to be the same, not only their volumes. With --perturb, a case whose outcome flips fails the run.
// With --perturb, each case is also made moved far off, turned a quarter turn, made a millionth larger, and with its tools
// taken in two other orders; what then comes out differently is counted. --digest FILE writes one line per case and
// variant (to compare one machine's results with another's). --record prints the cases with what they now give as their
// expectation.
#include "BcadKernel.h"
#include "Engine/Model.hpp"
#include "Engine/Step.hpp"
#include "Engine/Treat.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

// How a variant differs from the case as saved: a placement put on the shape (and on every pick and size with it), and the
// order its tools are taken in.
struct Variant {
  const char *name;
  double m[12];
  double scale;
  int seed;
};

const Variant variants[] = {
    {"base", {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}, 1, 0},
    {"moved", {1, 0, 0, 1000, 0, 1, 0, -700, 0, 0, 1, 350}, 1, 0},
    {"turned", {0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0}, 1, 0},
    {"scaled", {1 + 1e-6, 0, 0, 0, 0, 1 + 1e-6, 0, 0, 0, 0, 1 + 1e-6, 0}, 1 + 1e-6, 0},
    {"order1", {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}, 1, 1},
    {"order2", {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}, 1, 2},
};

struct Result {
  std::string status;  // OK, FAIL, SEC, BASEFAIL
  double volume = 0, area = 0;
  int pieces = 0, faces = 0;
  bool closed = true;
  std::string print;  // what's wrong with it as printers take it ("" when nothing)
  int crowded = 0, slivers = 0;
  uint64_t files = 0;  // a hash of its print mesh and STEP file (the case as saved)
  long booleans = 0;
  double ms = 0;
  std::string why;
};

void apply(const Variant &v, const double *p, double *out) {
  for (int r = 0; r < 3; r++) out[r] = v.m[4 * r] * p[0] + v.m[4 * r + 1] * p[1] + v.m[4 * r + 2] * p[2] + v.m[4 * r + 3];
}
void turn(const Variant &v, const double *d, double *out) {
  for (int r = 0; r < 3; r++) out[r] = v.m[4 * r] * d[0] + v.m[4 * r + 1] * d[1] + v.m[4 * r + 2] * d[2];
}
// A pick moved with the shape: an edge pick's point and direction, a face's or corner's normal and point.
void movePick(const Variant &v, int kind, const double *in, double *out) {
  if (kind == BK_PICK_EDGE) {
    apply(v, in, out), turn(v, in + 3, out + 3);
  } else if (kind == BK_PICK_FACE || kind == BK_PICK_CORNER) {
    turn(v, in, out), apply(v, in + 3, out + 3);
    double n = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    if (n > 0)
      for (int k = 0; k < 3; k++) out[k] /= n;
  } else {
    std::memcpy(out, in, 6 * sizeof(double));
  }
}

BKShape *build(const std::string &prog, std::string &why) {
  std::istringstream in(prog);
  std::vector<std::string> tok;
  for (std::string t; in >> t;) tok.push_back(t);
  size_t at = 0;
  // The numbers that follow, up to the next word that isn't one (";" or the next step).
  auto numbers = [&]() {
    std::vector<double> v;
    for (; at < tok.size(); at++) {
      char *end = nullptr;
      double x = std::strtod(tok[at].c_str(), &end);
      if (end == tok[at].c_str() || *end) break;
      v.push_back(x);
    }
    return v;
  };
  std::vector<BKShape *> st;
  auto fail = [&](const std::string &w) {
    why = w;
    for (BKShape *s : st) bk_free(s);
    return nullptr;
  };
  while (at < tok.size()) {
    std::string t = tok[at++];
    std::vector<double> v = numbers();
    auto need = [&](size_t n) { return v.size() >= n; };
    if (t == "P") {
      // A kind and its sizes, as many as it takes (more are ignored).
      if (!need(2)) return fail("primitive without sizes");
      std::vector<double> p(v.begin() + 1, v.end());
      p.resize(std::max<size_t>(p.size(), 8), 0);
      BKShape *s = bk_primitive((int)v[0], p.data());
      if (!s) return fail(std::string("prim: ") + bk_last_error());
      st.push_back(s);
    } else if (t == "N") {
      // A bolt or nut: kind, thread, length (0: the usual one), clearance; the other sizes standard for it.
      if (!need(4)) return fail("bolt without sizes");
      BKFastener f{};
      f.kind = (int)v[0], f.size = (int)v[1];
      bk_fastener_defaults(&f, 1);
      if (v[2] > 0) f.length = v[2], bk_fastener_fit(&f);
      BKShape *s = bk_fastener(&f, v[3]);
      if (!s) return fail(std::string("bolt: ") + bk_last_error());
      st.push_back(s);
    } else if (t == "T") {
      if (!need(12)) return fail("transform without its numbers");
      if (st.empty()) return fail("transform of nothing");
      BKShape *s = bk_transform(st.back(), v.data());
      bk_free(st.back());
      st.back() = s;
      if (!s) return fail("transform");
    } else if (t == "B") {
      if (!need(1)) return fail("merge without its kind");
      if (st.size() < 2) return fail("merge of one");
      BKShape *b = st.back();
      st.pop_back();
      BKShape *a = st.back();
      BKShape *s = bk_boolean((int)v[0], a, b);
      bk_free(a), bk_free(b);
      st.back() = s;
      if (!s) return fail(std::string("bool: ") + bk_last_error());
    } else if (t == "S") {
      if (!need(7)) return fail("split without its plane");
      if (st.empty()) return fail("split of nothing");
      BKShape *s = bk_split(st.back(), v.data(), v.data() + 3, (int)v[6]);
      bk_free(st.back());
      st.back() = s;
      if (!s) return fail(std::string("split: ") + bk_last_error());
    } else if (t == "M") {
      // The shape as a mesh body (as a sculpt holds it): its mesh at detail d, points joined where they're one.
      if (!need(1)) return fail("mesh without its detail");
      if (st.empty()) return fail("mesh of nothing");
      BKMesh *m = bk_mesh(st.back(), v[0]);
      std::map<std::tuple<float, float, float>, uint32_t> id;
      std::vector<float> pos;
      std::vector<uint32_t> idx;
      for (int i = 0; m && i < 3 * m->triangleCount; i++) {
        const float *q = m->positions + 3 * m->indices[i];
        auto at = id.emplace(std::make_tuple(q[0] + 0.0f, q[1] + 0.0f, q[2] + 0.0f), (uint32_t)id.size());
        if (at.second) pos.insert(pos.end(), {q[0], q[1], q[2]});
        idx.push_back(at.first->second);
      }
      bk_mesh_free(m);
      BKShape *s = bk_mesh_shape(pos.data(), (int)pos.size() / 3, idx.data(), (int)idx.size() / 3);
      bk_free(st.back());
      st.back() = s;
      if (!s) return fail(bk_last_error());
    } else if (t == "R") {
      // The shape so far made ready for sculpting at detail d: remeshed, as a mesh body.
      if (!need(1)) return fail("remesh without its detail");
      if (st.empty()) return fail("remesh of nothing");
      const double same[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
      BKSculptMesh *r = bk_remesh(st.back(), same, v[0]);
      BKShape *s = r ? bk_mesh_shape(r->positions, r->vertexCount, r->indices, r->triangleCount) : nullptr;
      std::string why = s ? "" : bk_last_error();
      bk_sculpt_mesh_free(r);
      bk_free(st.back());
      st.back() = s;
      if (!s) return fail(why);
    }
  }
  if (st.size() != 1) return fail("program left " + std::to_string(st.size()) + " shapes");
  return st[0];
}

// Whether a mesh is closed: every side met by one running the other way (points by position).
bool closedMesh(const BKMesh *m) {
  std::map<std::tuple<float, float, float>, uint32_t> id;
  std::vector<uint32_t> at(m->vertexCount);
  for (int i = 0; i < m->vertexCount; i++) {
    const float *q = m->positions + 3 * i;
    at[i] = id.emplace(std::make_tuple(q[0] + 0.0f, q[1] + 0.0f, q[2] + 0.0f), (uint32_t)id.size()).first->second;
  }
  std::vector<uint64_t> fwd, back;
  for (int t = 0; t < m->triangleCount; t++)
    for (int k = 0; k < 3; k++) {
      uint32_t a = at[m->indices[3 * t + k]], b = at[m->indices[3 * t + (k + 1) % 3]];
      if (a == b) return false;
      fwd.push_back((uint64_t)a << 32 | b), back.push_back((uint64_t)b << 32 | a);
    }
  std::sort(fwd.begin(), fwd.end()), std::sort(back.begin(), back.end());
  return fwd == back;
}

std::vector<int> kindsOf(const std::vector<double> &picks, std::vector<double> &data) {
  std::vector<int> k;
  for (size_t i = 0; i + 6 < picks.size() + 1; i += 7) {
    k.push_back((int)picks[i]);
    data.insert(data.end(), picks.begin() + i + 1, picks.begin() + i + 7);
  }
  return k;
}

std::string digits(double x);

Result run(const std::string &prog, const std::string &ops, const Variant &v) {
  Result r;
  auto t0 = std::chrono::steady_clock::now();
  long calls0 = bce::combineReport.calls;
  bce::toolOrderSeed = v.seed;
  auto done = [&]() {
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    r.booleans = bce::combineReport.calls - calls0;
    bce::toolOrderSeed = 0;
    return r;
  };
  BKShape *base = build(prog, r.why);
  if (!base) {
    r.status = "BASEFAIL";
    return done();
  }
  if (std::memcmp(v.m, variants[0].m, sizeof v.m) != 0) {
    BKShape *moved = bk_transform(base, v.m);
    bk_free(base);
    base = moved;
    if (!base) {
      r.status = "BASEFAIL";
      r.why = "variant transform";
      return done();
    }
  }
  BKShape *cur = bk_copy(base);
  std::istringstream in(ops);
  std::string op;
  bool treated = false;
  while (in >> op) {
    BKShape *next = nullptr;
    int missing = 0;
    double most = 0;
    auto picks = [&](int n, std::vector<int> &kinds, std::vector<double> &data) {
      for (int i = 0; i < n; i++) {
        int k;
        double p[6], q[6];
        in >> k;
        for (double &x : p) in >> x;
        movePick(v, k, p, q);
        kinds.push_back(k);
        data.insert(data.end(), q, q + 6);
      }
    };
    if (op == "F" || op == "V") {
      double rad;
      int n;
      in >> rad >> n;
      std::vector<int> k;
      std::vector<double> d;
      picks(n, k, d);
      rad *= v.scale;
      next = op == "F" ? bk_fillet(cur, k.data(), d.data(), n, rad, &most, &missing) : bk_cove(cur, k.data(), d.data(), n, rad, &most, &missing);
      treated = true;
    } else if (op == "C") {
      double a, b, cr;
      int n;
      in >> a >> b >> cr >> n;
      std::vector<int> k;
      std::vector<double> d;
      picks(n, k, d);
      next = bk_chamfer(cur, k.data(), d.data(), n, a * v.scale, b * v.scale, cr * v.scale, &missing);
      treated = true;
    } else if (op == "H") {
      double th;
      int no, nw;
      in >> th >> no;
      std::vector<double> open(6 * no), walls, wt;
      for (int i = 0; i < no; i++) {
        double p[6];
        for (double &x : p) in >> x;
        movePick(v, BK_PICK_FACE, p, &open[6 * i]);
      }
      in >> nw;
      walls.resize(6 * nw), wt.resize(nw);
      for (int i = 0; i < nw; i++) {
        double p[6];
        for (double &x : p) in >> x;
        movePick(v, BK_PICK_FACE, p, &walls[6 * i]);
        in >> wt[i];
        wt[i] *= v.scale;
      }
      // As the app does: the shape without its treatments offered as the sharp one to fall back on.
      const BKShape *sharp[1] = {base};
      next = bk_hollow(cur, treated ? sharp : nullptr, treated ? 1 : 0, open.data(), no, walls.data(), wt.data(), nw, th * v.scale, &missing);
    } else if (op == "X") {
      int k;
      double p[6], q[6];
      in >> k;
      for (double &x : p) in >> x;
      movePick(v, k, p, q);
      BKSection *s = bk_section(cur, k, q, 20);
      if (!s) {
        r.status = "FAIL";
        r.why = bk_last_error();
      } else {
        double area = 0;
        for (int l = 0; l < s->loopCount; l++)
          for (int i = s->loopStart[l]; i < s->loopStart[l + 1]; i++) {
            int j = i + 1 < s->loopStart[l + 1] ? i + 1 : s->loopStart[l];
            area += s->points[2 * i] * s->points[2 * j + 1] - s->points[2 * j] * s->points[2 * i + 1];
          }
        r.status = "SEC", r.area = area / 2 / (v.scale * v.scale);
        bk_section_free(s);
      }
      bk_free(cur), bk_free(base);
      return done();
    } else {
      continue;
    }
    if (!next) {
      r.status = "FAIL";
      r.why = bk_last_error();
      bk_free(cur), bk_free(base);
      return done();
    }
    bk_free(cur);
    cur = next;
  }
  BKMesh *m = bk_mesh(cur, 0.05);
  if (!m) {
    r.status = "FAIL";
    r.why = "mesh";
  } else {
    r.status = "OK";
    r.volume = m->volume / (v.scale * v.scale * v.scale);
    r.faces = m->faceCount;
    r.closed = closedMesh(m);
    r.pieces = bk_piece_count(cur);
    bk_mesh_free(m);
    if (v.seed == 0 && v.m[3] == 0 && v.m[0] == 1) {
      // (Not counted in the case's time, nor its merges.)
      auto p0 = std::chrono::steady_clock::now();
      long c0 = bce::combineReport.calls;
      BKPrintMesh *pm = bk_print_mesh(cur);
      // Its volume the mesh's own at that detail (not the shape's exact one: the mesh cuts inside curves).
      BKMesh *fine = bk_mesh(cur, 0.01);
      double own = 0;
      for (int t = 0; t < fine->triangleCount; t++) {
        const float *a = fine->positions + 3 * fine->indices[3 * t], *b = fine->positions + 3 * fine->indices[3 * t + 1],
                    *c = fine->positions + 3 * fine->indices[3 * t + 2];
        own += ((double)a[0] * ((double)b[1] * c[2] - (double)b[2] * c[1]) + (double)a[1] * ((double)b[2] * c[0] - (double)b[0] * c[2]) +
                (double)a[2] * ((double)b[0] * c[1] - (double)b[1] * c[0])) / 6;
      }
      bk_mesh_free(fine);
      if (!pm->valid) r.print = bk_last_error();
      else if (!(std::fabs(pm->volume - own) <= 1e-6 * own)) r.print = "volume " + digits(pm->volume) + ", the mesh's " + digits(own);
      r.crowded = pm->crowded, r.slivers = pm->slivers;
      // FNV-1a over the mesh's bits and the STEP file's text (its header, which has the time, left out).
      uint64_t h = 1469598103934665603ull;
      auto mix = [&](const void *data, size_t n) {
        for (size_t i = 0; i < n; i++) h = (h ^ ((const unsigned char *)data)[i]) * 1099511628211ull;
      };
      mix(pm->positions, sizeof(float) * 3 * pm->vertexCount);
      mix(pm->indices, sizeof(uint32_t) * 3 * pm->triangleCount);
      std::string step, why;
      if (bce::stepText({bce::heldShape(cur)}, {"case"}, "case.step", step, why)) {
        size_t data = step.find("DATA;");
        mix(step.data() + data, step.size() - data);
      }
      r.files = h;
      t0 += std::chrono::steady_clock::now() - p0;
      calls0 += bce::combineReport.calls - c0;
      bk_print_mesh_free(pm);
    }
  }
  bk_free(cur), bk_free(base);
  return done();
}

std::string digits(double x) {
  char b[40];
  snprintf(b, sizeof b, "%.7g", x);
  return b;
}

}  // namespace

int main(int argc, char **argv) {
  std::string path = "test/corpus/cases.txt", digestPath;
  bool perturb = false, record = false;
  std::string only;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--perturb") perturb = true;
    else if (a == "--record") record = true;
    else if (a == "--digest" && i + 1 < argc) digestPath = argv[++i];
    else if (a == "--only" && i + 1 < argc) only = argv[++i];
    else path = a;
  }
  std::ifstream file(path);
  if (!file) {
    fprintf(stderr, "no corpus at %s\n", path.c_str());
    return 2;
  }
  FILE *digest = digestPath.empty() ? nullptr : fopen(digestPath.c_str(), "w");
  std::string line;
  int cases = 0, wrong = 0, open = 0, unprintable = 0, crowded = 0, slivers = 0, sliverCases = 0;
  // Per variant: outcomes that differ from the case's own, volumes off by more than 1e-6 and 1e-3, faces or pieces
  // that differ.
  int nv = perturb ? (int)(sizeof variants / sizeof variants[0]) : 1;
  std::vector<int> flips(nv), drift6(nv), drift3(nv), shape(nv);
  long booleans = 0, mostBooleans = 0;
  double slowest = 0, total = 0;
  std::string slowestId, mostId;
  bce::combineReport = {};
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> f;
    for (size_t at = 0;;) {
      size_t bar = line.find('|', at);
      f.push_back(line.substr(at, bar == std::string::npos ? std::string::npos : bar - at));
      if (bar == std::string::npos) break;
      at = bar + 1;
    }
    if (f.size() < 3) continue;
    const std::string &id = f[0];
    if (!only.empty() && id != only) continue;
    std::string expect = f.size() > 3 ? f[3] : "any";
    cases++;
    std::vector<Result> rs;
    for (int k = 0; k < nv; k++) rs.push_back(run(f[1], f[2], variants[k]));
    const Result &r = rs[0];
    total += r.ms, booleans += r.booleans;
    if (r.ms > slowest) slowest = r.ms, slowestId = id;
    if (r.booleans > mostBooleans) mostBooleans = r.booleans, mostId = id;
    if (r.status == "OK" && !r.closed) {
      open++;
      printf("OPEN %s: the result's mesh isn't closed\n", id.c_str());
    }
    if (r.status == "OK" && !r.print.empty()) {
      unprintable++;
      printf("PRINT %s: %s\n", id.c_str(), r.print.c_str());
    }
    crowded += r.crowded, slivers += r.slivers, sliverCases += r.slivers > 0;
    if (record) {
      std::string e = r.status == "OK" ? "made " + digits(r.volume) + " " + digits(std::max(2e-4 * r.volume, 1e-3))
                      : r.status == "SEC" ? "sec " + digits(r.area) + " " + digits(std::max(1e-4 * std::fabs(r.area), 1e-3))
                                          : "refused";
      printf("%s|%s|%s|%s\n", id.c_str(), f[1].c_str(), f[2].c_str(), e.c_str());
    } else {
      std::istringstream es(expect);
      std::string kind;
      double value = 0, tol = 0;
      es >> kind >> value >> tol;
      bool ok = kind == "any" || (kind == "refused" && r.status == "FAIL") || (kind == "made" && r.status == "OK" && std::fabs(r.volume - value) <= tol) ||
                (kind == "sec" && r.status == "SEC" && std::fabs(r.area - value) <= tol);
      if (!ok) {
        wrong++;
        printf("WRONG %s: expected %s, got %s %s%s\n", id.c_str(), expect.c_str(), r.status.c_str(),
               r.status == "OK" ? digits(r.volume).c_str() : r.status == "SEC" ? digits(r.area).c_str() : "", r.why.empty() ? "" : (" (" + r.why + ")").c_str());
      }
    }
    for (int k = 0; k < nv; k++) {
      const Result &x = rs[k];
      if (digest)
        fprintf(digest, "%s %s %s %s %d %d%s\n", id.c_str(), variants[k].name, x.status.c_str(),
                x.status == "OK" ? digits(x.volume).c_str() : x.status == "SEC" ? digits(x.area).c_str() : "-", x.pieces, x.faces,
                x.files ? (" files " + std::to_string(x.files)).c_str() : "");
      if (k == 0) continue;
      if (x.status != r.status) {
        flips[k]++;
        if (perturb) printf("FLIP %s %s: %s, as saved %s%s\n", id.c_str(), variants[k].name, x.status.c_str(), r.status.c_str(), x.why.empty() ? "" : (" (" + x.why + ")").c_str());
        continue;
      }
      double a = r.status == "SEC" ? r.area : r.volume, b = x.status == "SEC" ? x.area : x.volume;
      double rel = std::fabs(a - b) / std::max(1e-9, std::fabs(a));
      if (rel > 1e-6) drift6[k]++;
      if (rel > 1e-3) {
        drift3[k]++;
        printf("DRIFT %s %s: %s, as saved %s\n", id.c_str(), variants[k].name, digits(b).c_str(), digits(a).c_str());
      }
      if (x.faces != r.faces || x.pieces != r.pieces) shape[k]++;
    }
  }
  if (digest) fclose(digest);
  if (record) return 0;
  printf("%d cases, %d not as expected, %d open, %d not printable (%d crowded edges, %d slivers in %d); %.1f s, slowest %s %.0f ms; %ld mesh "
         "booleans, most %s %ld\n", cases, wrong, open, unprintable, crowded, slivers, sliverCases, total / 1000, slowestId.c_str(), slowest, booleans,
         mostId.c_str(), mostBooleans);
  const auto &cr = bce::combineReport;
  printf("booleans: %ld; keeps failed %ld, cuts dropped %ld, cuts crossing %ld, misses %ld, overflows %ld, came out open %ld, "
         "rays all grazing %ld, made again %ld\n", cr.calls, cr.keepsFailed, cr.segsDropped, cr.crossingsMade, cr.misses, cr.overflows, cr.open, cr.unsure,
         cr.again);
  for (int k = 1; k < nv; k++)
    printf("%-7s outcome flips %d, volume drift >1e-6 %d, >1e-3 %d, faces or pieces differ %d\n", variants[k].name, flips[k], drift6[k], drift3[k], shape[k]);
  int flipped = 0;
  for (int k = 1; k < nv; k++) flipped += flips[k];
  return wrong || open || unprintable || flipped ? 1 : 0;
}
