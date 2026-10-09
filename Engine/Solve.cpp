// Bcad's geometry engine: a sketch's rules (constraints and dimensions) made to hold. Each rule is one or two equations in
// the points' coordinates and circles' radii; the sketch is moved as little as it can be until they all hold (damped
// least-norm Newton steps, in a fixed order: the same answer on every machine), then what's left free is worked out.
#include "Engine/Sketch.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>

namespace bce {

namespace {

constexpr double pi = M_PI;
constexpr int NV = 12;  // the most numbers one rule takes

// A number and how it changes with each of a rule's numbers (forward differentiation).
struct Dual {
  double v = 0;
  double g[NV] = {};
};
Dual constant(double v) {
  Dual d;
  d.v = v;
  return d;
}
Dual operator+(Dual a, const Dual &b) {
  a.v += b.v;
  for (int k = 0; k < NV; k++) a.g[k] += b.g[k];
  return a;
}
Dual operator-(Dual a, const Dual &b) {
  a.v -= b.v;
  for (int k = 0; k < NV; k++) a.g[k] -= b.g[k];
  return a;
}
Dual operator*(Dual a, double s) {
  a.v *= s;
  for (int k = 0; k < NV; k++) a.g[k] *= s;
  return a;
}
Dual operator*(const Dual &a, const Dual &b) {
  Dual r;
  r.v = a.v * b.v;
  for (int k = 0; k < NV; k++) r.g[k] = a.g[k] * b.v + a.v * b.g[k];
  return r;
}
Dual operator/(const Dual &a, const Dual &b) {
  Dual r;
  r.v = a.v / b.v;
  for (int k = 0; k < NV; k++) r.g[k] = (a.g[k] * b.v - a.v * b.g[k]) / (b.v * b.v);
  return r;
}
Dual root(const Dual &a) {
  Dual r;
  r.v = std::sqrt(std::max(a.v, 0.0));
  for (int k = 0; k < NV; k++) r.g[k] = r.v > 1e-300 ? a.g[k] / (2 * r.v) : 0;
  return r;
}
Dual angleOf(const Dual &y, const Dual &x) {
  Dual r;
  r.v = trig::atan2(y.v, x.v);
  double den = x.v * x.v + y.v * y.v;
  for (int k = 0; k < NV; k++) r.g[k] = den > 0 ? (x.v * y.g[k] - y.v * x.g[k]) / den : 0;
  return r;
}
// An angle brought within (−π, π] (a whole turn off makes no difference to how it changes).
Dual wrapped(Dual a) {
  a.v = std::remainder(a.v, 2 * pi);
  return a;
}

struct P2 {
  Dual x, y;
};
P2 operator+(const P2 &a, const P2 &b) { return {a.x + b.x, a.y + b.y}; }
P2 operator-(const P2 &a, const P2 &b) { return {a.x - b.x, a.y - b.y}; }
P2 operator*(const P2 &a, double s) { return {a.x * s, a.y * s}; }
Dual cross(const P2 &a, const P2 &b) { return a.x * b.y - a.y * b.x; }
Dual dot(const P2 &a, const P2 &b) { return a.x * b.x + a.y * b.y; }
Dual length(const P2 &a) { return root(a.x * a.x + a.y * a.y); }
// The signed angle from a to b.
Dual angleBetween(const P2 &a, const P2 &b) { return angleOf(cross(a, b), dot(a, b)); }

// Where an equation comes from: a rule, or an arc's own (its end on its circle).
struct Source {
  int rule = -1, arc = -1;
};

struct Row {
  double f = 0;
  int n = 0;
  int var[NV];
  double d[NV];
};

struct System {
  const Sketch &s;
  std::vector<int> vx, vy, vr;  // each point's and circle's numbers (-1: held where it is)
  std::vector<double> x;
  double L = 1;

  explicit System(const Sketch &sk) : s(sk) {}

  // A rule's own numbers, as they're met.
  struct Local {
    int var[NV];
    int n = 0;
    bool full = false;
    int slot(int g) {
      if (g < 0) return -1;
      for (int k = 0; k < n; k++)
        if (var[k] == g) return k;
      if (n == NV) return full = true, -1;
      var[n] = g;
      return n++;
    }
  };
  Dual number(Local &l, int g, double held) const {
    Dual d = constant(g >= 0 ? x[g] : held);
    int k = l.slot(g);
    if (k >= 0) d.g[k] = 1;
    return d;
  }
  P2 point(Local &l, int i) const { return {number(l, vx[i], s.points[i].x), number(l, vy[i], s.points[i].y)}; }
  int kind(int c) const { return c >= 0 && c < (int)s.curves.size() ? s.curves[c].kind : -1; }
  Dual radius(Local &l, int c) const {
    const SketchCurve &cv = s.curves[c];
    if (cv.kind == BK_CURVE_CIRCLE) return number(l, vr[c], cv.radius);
    return length(point(l, cv.p[1]) - point(l, cv.p[0]));
  }

  // A source's equations (one or two), or false where the rule doesn't fit what it names.
  bool rows(const Source &src, std::vector<Row> &out) const {
    Local l;
    Dual f[2];
    int count = 1;
    if (src.arc >= 0) {
      const SketchCurve &c = s.curves[src.arc];
      P2 C = point(l, c.p[0]);
      f[0] = length(point(l, c.p[1]) - C) - length(point(l, c.p[2]) - C);
    } else {
      const SketchRule &r = s.rules[src.rule];
      int c0 = r.c[0], c1 = r.c[1], p0 = r.p[0], p1 = r.p[1];
      bool line0 = kind(c0) == BK_CURVE_LINE, line1 = kind(c1) == BK_CURVE_LINE;
      bool round0 = kind(c0) == BK_CURVE_ARC || kind(c0) == BK_CURVE_CIRCLE, round1 = kind(c1) == BK_CURVE_ARC || kind(c1) == BK_CURVE_CIRCLE;
      double side = r.side < 0 ? -1 : 1;
      auto A = [&](int c) { return point(l, s.curves[c].p[0]); };
      auto B = [&](int c) { return point(l, s.curves[c].p[1]); };
      auto way = [&](int c) { return B(c) - A(c); };
      // The distance of p from line c, positive on its left.
      auto off = [&](const P2 &p, int c) { return cross(way(c), p - A(c)) / length(way(c)); };
      switch (r.kind) {
      case BK_RULE_COINCIDENT: {
        if (p0 < 0 || p1 < 0) return false;
        P2 d = point(l, p1) - point(l, p0);
        f[0] = d.x, f[1] = d.y, count = 2;
        break;
      }
      case BK_RULE_ON:
        if (p0 < 0 || (!line0 && !round0)) return false;
        f[0] = line0 ? off(point(l, p0), c0) : length(point(l, p0) - A(c0)) - radius(l, c0);
        break;
      case BK_RULE_HORIZONTAL:
      case BK_RULE_VERTICAL: {
        P2 d;
        if (line0) d = way(c0);
        else if (p0 >= 0 && p1 >= 0) d = point(l, p1) - point(l, p0);
        else return false;
        f[0] = r.kind == BK_RULE_HORIZONTAL ? d.y : d.x;
        break;
      }
      case BK_RULE_PARALLEL:
        if (!line0 || !line1) return false;
        f[0] = angleBetween(way(c0) * side, way(c1)) * L;
        break;
      case BK_RULE_PERPENDICULAR: {
        if (!line0 || !line1) return false;
        Dual a = angleBetween(way(c0), way(c1));
        a.v -= side * pi / 2;
        f[0] = wrapped(a) * L;
        break;
      }
      case BK_RULE_TANGENT:
        if (line0 && round1) f[0] = off(A(c1), c0) * side - radius(l, c1);
        else if (round0 && line1) f[0] = off(A(c0), c1) * side - radius(l, c0);
        else if (round0 && round1) {
          Dual d = length(A(c1) - A(c0));
          // side 1: apart (radii added); -1: c1 inside c0; -2: c0 inside c1.
          f[0] = r.side == -1 ? d - (radius(l, c0) - radius(l, c1)) : r.side == -2 ? d - (radius(l, c1) - radius(l, c0)) : d - (radius(l, c0) + radius(l, c1));
        } else {
          return false;
        }
        break;
      case BK_RULE_EQUAL:
        if ((!line0 && !round0) || (!line1 && !round1)) return false;
        f[0] = (line0 ? length(way(c0)) : radius(l, c0)) - (line1 ? length(way(c1)) : radius(l, c1));
        break;
      case BK_RULE_CONCENTRIC: {
        if (!round0 || !round1) return false;
        P2 d = A(c1) - A(c0);
        f[0] = d.x, f[1] = d.y, count = 2;
        break;
      }
      case BK_RULE_MIDPOINT: {
        if (p0 < 0 || !line0) return false;
        P2 d = point(l, p0) - (A(c0) + B(c0)) * 0.5;
        f[0] = d.x, f[1] = d.y, count = 2;
        break;
      }
      case BK_RULE_COLLINEAR:
        if (!line0 || !line1) return false;
        f[0] = off(A(c1), c0), f[1] = off(B(c1), c0), count = 2;
        break;
      case BK_RULE_SYMMETRIC: {
        if (p0 < 0 || p1 < 0 || !line0) return false;
        P2 p = point(l, p0), q = point(l, p1);
        f[0] = off((p + q) * 0.5, c0), f[1] = dot(q - p, way(c0)) / length(way(c0)), count = 2;
        break;
      }
      case BK_RULE_FIX:
        return p0 >= 0;  // (held as a constant: no equation)
      case BK_DIM_DISTANCE:
        if (p0 < 0 || p1 < 0) return false;
        f[0] = length(point(l, p1) - point(l, p0)) - constant(r.value);
        break;
      case BK_DIM_HORIZONTAL:
      case BK_DIM_VERTICAL: {
        if (p0 < 0 || p1 < 0) return false;
        P2 d = point(l, p1) - point(l, p0);
        f[0] = (r.kind == BK_DIM_HORIZONTAL ? d.x : d.y) * side - constant(r.value);
        break;
      }
      case BK_DIM_POINT_LINE:
        if (p0 < 0 || !line0) return false;
        f[0] = off(point(l, p0), c0) * side - constant(r.value);
        break;
      case BK_DIM_LINES:
        if (!line0 || !line1) return false;
        f[0] = off((A(c1) + B(c1)) * 0.5, c0) * side - constant(r.value);
        break;
      case BK_DIM_LENGTH:
        if (!line0) return false;
        f[0] = length(way(c0)) - constant(r.value);
        break;
      case BK_DIM_RADIUS:
      case BK_DIM_DIAMETER:
        if (!round0) return false;
        f[0] = radius(l, c0) * (r.kind == BK_DIM_DIAMETER ? 2.0 : 1.0) - constant(r.value);
        break;
      case BK_DIM_ANGLE: {
        if (!line0 || !line1) return false;
        Dual a = angleBetween(way(c0) * (r.side & 1 ? -1.0 : 1.0), way(c1) * (r.side & 2 ? -1.0 : 1.0));
        a.v -= r.value * pi / 180;
        f[0] = wrapped(a) * L;
        break;
      }
      default:
        return false;
      }
    }
    if (l.full) return false;
    for (int k = 0; k < count; k++) {
      Row row;
      row.f = f[k].v, row.n = l.n;
      for (int j = 0; j < l.n; j++) row.var[j] = l.var[j], row.d[j] = f[k].g[j];
      out.push_back(row);
    }
    return true;
  }
};

// a = L Lᵀ in place (lower half), false where it isn't positive definite. first[i]: row i's first number off zero (L
// has none before it either: only what's within that envelope is worked out).
bool cholesky(std::vector<double> &a, int m, const std::vector<int> &first) {
  for (int j = 0; j < m; j++) {
    double d = a[(size_t)j * m + j];
    for (int k = first[j]; k < j; k++) d -= a[(size_t)j * m + k] * a[(size_t)j * m + k];
    if (!(d > 0)) return false;
    d = std::sqrt(d);
    a[(size_t)j * m + j] = d;
    for (int i = j + 1; i < m; i++) {
      if (first[i] > j) continue;
      double s = a[(size_t)i * m + j];
      for (int k = std::max(first[i], first[j]); k < j; k++) s -= a[(size_t)i * m + k] * a[(size_t)j * m + k];
      a[(size_t)i * m + j] = s / d;
    }
  }
  return true;
}

void choleskySolve(const std::vector<double> &a, int m, const std::vector<int> &first, std::vector<double> &b) {
  for (int i = 0; i < m; i++) {
    double s = b[i];
    for (int k = first[i]; k < i; k++) s -= a[(size_t)i * m + k] * b[k];
    b[i] = s / a[(size_t)i * m + i];
  }
  for (int i = m - 1; i >= 0; i--) {
    double s = b[i];
    for (int k = i + 1; k < m; k++)
      if (first[k] <= i) s -= a[(size_t)k * m + i] * b[k];
    b[i] = s / a[(size_t)i * m + i];
  }
}

}  // namespace

double sketchJacobianError(const Sketch &s) {
  int np = (int)s.points.size(), nc = (int)s.curves.size();
  System sys(s);
  sys.vx.assign(np, -1), sys.vy.assign(np, -1), sys.vr.assign(nc, -1);
  for (int i = 0; i < np; i++) {
    sys.vx[i] = (int)sys.x.size(), sys.x.push_back(s.points[i].x);
    sys.vy[i] = (int)sys.x.size(), sys.x.push_back(s.points[i].y);
  }
  for (int c = 0; c < nc; c++)
    if (s.curves[c].kind == BK_CURVE_CIRCLE) sys.vr[c] = (int)sys.x.size(), sys.x.push_back(s.curves[c].radius);
  double worst = 0;
  for (int r = 0; r < (int)s.rules.size(); r++) {
    std::vector<Row> rows;
    if (!sys.rows({r, -1}, rows)) return INFINITY;
    for (size_t q = 0; q < rows.size(); q++)
      for (int j = 0; j < rows[q].n; j++) {
        int v = rows[q].var[j];
        double keep = sys.x[v], h = 1e-6 * std::max(1.0, std::fabs(keep));
        std::vector<Row> up, down;
        sys.x[v] = keep + h, sys.rows({r, -1}, up);
        sys.x[v] = keep - h, sys.rows({r, -1}, down);
        sys.x[v] = keep;
        double fd = (up[q].f - down[q].f) / (2 * h);
        worst = std::max(worst, std::fabs(fd - rows[q].d[j]) / std::max(1.0, std::fabs(fd)));
      }
  }
  return worst;
}

bool solveSketch(Sketch &s, const std::vector<int> &drag, const std::vector<V3> &targets, SolveReport &rep, std::string &why) {
  rep = SolveReport();
  if (!sketchValid(s, why)) return false;
  int np = (int)s.points.size(), nc = (int)s.curves.size();
  System sys(s);
  // What's held where it is: points marked fixed, on a curve from a face, or fixed by a rule; and points nothing uses.
  std::vector<char> held(np, 0), used(np, 0);
  for (int i = 0; i < np; i++) held[i] = !s.fixed.empty() && s.fixed[i];
  for (const auto &c : s.curves) {
    int n = c.kind == BK_CURVE_LINE ? 2 : c.kind == BK_CURVE_ARC ? 3 : 1;
    for (int k = 0; k < n; k++) {
      used[c.p[k]] = 1;
      if (c.flags & BK_CURVE_REFERENCE) held[c.p[k]] = 1;
    }
  }
  for (const auto &r : s.rules) {
    for (int k = 0; k < 3; k++)
      if (r.p[k] >= 0) used[r.p[k]] = 1;
    if (r.kind == BK_RULE_FIX && r.p[0] >= 0) held[r.p[0]] = 1;
  }
  sys.vx.assign(np, -1), sys.vy.assign(np, -1), sys.vr.assign(nc, -1);
  double extent = 0;
  for (int i = 0; i < np; i++) {
    extent = std::max({extent, std::fabs(s.points[i].x), std::fabs(s.points[i].y)});
    if (held[i] || !used[i]) continue;
    sys.vx[i] = (int)sys.x.size(), sys.x.push_back(s.points[i].x);
    sys.vy[i] = (int)sys.x.size(), sys.x.push_back(s.points[i].y);
  }
  for (int c = 0; c < nc; c++)
    if (s.curves[c].kind == BK_CURVE_CIRCLE && !(s.curves[c].flags & BK_CURVE_REFERENCE)) sys.vr[c] = (int)sys.x.size(), sys.x.push_back(s.curves[c].radius);
  sys.L = std::max(1.0, extent);
  int nv = (int)sys.x.size();
  // Dragged points start where they're dragged to, and move least.
  std::vector<double> weight(nv, 1);
  for (size_t k = 0; k < drag.size() && k < targets.size(); k++) {
    int i = drag[k];
    if (i < 0 || i >= np || sys.vx[i] < 0) continue;
    if (!std::isfinite(targets[k].x) || !std::isfinite(targets[k].y)) return why = "sketch: sizes must be numbers", false;
    sys.x[sys.vx[i]] = targets[k].x, sys.x[sys.vy[i]] = targets[k].y;
    weight[sys.vx[i]] = weight[sys.vy[i]] = 1e3;
  }
  std::vector<Source> sources;
  for (int c = 0; c < nc; c++)
    if (s.curves[c].kind == BK_CURVE_ARC && !(s.curves[c].flags & BK_CURVE_REFERENCE)) sources.push_back({-1, c});
  for (int r = 0; r < (int)s.rules.size(); r++) sources.push_back({r, -1});
  // Which numbers each source moves; sources sharing a number are one part, solved together.
  std::vector<std::vector<Row>> at(sources.size());
  for (size_t k = 0; k < sources.size(); k++)
    if (!sys.rows(sources[k], at[k])) return why = "sketch: rule " + std::to_string(sources[k].rule) + " doesn't fit what it names", false;
  std::vector<int> part(nv);
  std::iota(part.begin(), part.end(), 0);
  std::function<int(int)> find = [&](int a) { return part[a] == a ? a : part[a] = find(part[a]); };
  for (const auto &rows : at)
    for (const Row &row : rows)
      for (int j = 1; j < row.n; j++) {
        int a = find(row.var[0]), b = find(row.var[j]);
        if (a != b) part[std::max(a, b)] = std::min(a, b);
      }
  std::vector<int> partOfSource(sources.size(), -1);
  std::vector<std::vector<int>> parts(nv);  // sources per part (by its smallest number)
  std::vector<char> inRow(nv, 0);
  std::vector<int> constants;  // sources with no number to move
  for (size_t k = 0; k < sources.size(); k++) {
    int first = -1;
    for (const Row &row : at[k])
      for (int j = 0; j < row.n; j++) inRow[row.var[j]] = 1, first = row.var[0];
    if (first < 0) {
      if (!at[k].empty()) constants.push_back((int)k);
      continue;
    }
    parts[find(first)].push_back((int)k);
  }
  const double tol = 1e-10 * sys.L;
  // (While points are dragged, what's held and what's free isn't worked out: asked again once they're let go.)
  bool analyse = drag.empty();
  bool solved = true;
  std::vector<double> start = sys.x;
  int dependent = -1;
  auto noteDependent = [&](int k) {
    if (sources[k].rule >= 0 && (dependent < 0 || sources[k].rule < dependent)) dependent = sources[k].rule;
  };
  std::vector<char> determined(nv, 0);
  int freedom = 0;
  for (int v = 0; v < nv; v++)
    if (!inRow[v]) freedom++;
  for (int root = 0; root < nv; root++) {
    const auto &srcs = parts[root];
    if (srcs.empty()) continue;
    // Its numbers, in order, as columns.
    std::vector<int> vars;
    for (int v = 0; v < nv; v++)
      if (inRow[v] && find(v) == root) vars.push_back(v);
    int n = (int)vars.size();
    std::vector<int> col(nv, -1);
    for (int j = 0; j < n; j++) col[vars[j]] = j;
    if (n > 3000) return why = "sketch: too many conditions tied together", false;
    // Its equations' values, and their derivatives as rows of (column, value).
    struct Rows {
      std::vector<double> f;
      std::vector<int> start{0}, cols;
      std::vector<double> vals;
    };
    auto evaluate = [&](Rows &r) {
      r.f.clear(), r.start.assign(1, 0), r.cols.clear(), r.vals.clear();
      for (int k : srcs) {
        std::vector<Row> rows;
        sys.rows(sources[k], rows);
        for (const Row &row : rows) {
          r.f.push_back(row.f);
          for (int j = 0; j < row.n; j++) r.cols.push_back(col[row.var[j]]), r.vals.push_back(row.d[j]);
          r.start.push_back((int)r.cols.size());
        }
      }
    };
    Rows R;
    evaluate(R);
    int m = (int)R.f.size();
    if (m > 1500) return why = "sketch: too many conditions tied together", false;
    auto worst = [](const std::vector<double> &v) {
      double w = 0;
      for (double a : v) w = std::max(w, std::fabs(a));
      return w;
    };
    auto squares = [](const std::vector<double> &v) {
      double w = 0;
      for (double a : v) w += a * a;
      return w;
    };
    double lambda = 0;
    std::vector<double> A;
    std::vector<int> first(m);
    for (int it = 0; it < 60 && worst(R.f) > tol; it++) {
      // A = J W⁻¹ Jᵀ, from the rows each column is in.
      std::vector<std::vector<std::pair<int, double>>> byCol(n);
      for (int i = 0; i < m; i++)
        for (int e = R.start[i]; e < R.start[i + 1]; e++) byCol[R.cols[e]].push_back({i, R.vals[e]});
      A.assign((size_t)m * m, 0);
      for (int j = 0; j < n; j++) {
        double w = weight[vars[j]];
        const auto &list = byCol[j];
        for (size_t a = 0; a < list.size(); a++)
          for (size_t b = 0; b <= a; b++) A[(size_t)list[a].first * m + list[b].first] += list[a].second * list[b].second / w;
      }
      for (int i = 0; i < m; i++) {
        first[i] = i;
        for (int k = 0; k < i; k++)
          if (A[(size_t)i * m + k] != 0) {
            first[i] = k;
            break;
          }
      }
      double trace = 0;
      for (int i = 0; i < m; i++) trace += A[(size_t)i * m + i];
      double floor = 1e-14 * std::max(trace / m, 1e-300);
      lambda = std::max(lambda / 10, floor);
      bool moved = false;
      for (int tries = 0; tries < 24 && !moved; tries++, lambda *= 10) {
        std::vector<double> a = A;
        for (int i = 0; i < m; i++) a[(size_t)i * m + i] += lambda;
        if (!cholesky(a, m, first)) continue;
        std::vector<double> y(m);
        for (int i = 0; i < m; i++) y[i] = -R.f[i];
        choleskySolve(a, m, first, y);
        std::vector<double> before(n), step(n, 0);
        for (int j = 0; j < n; j++) before[j] = sys.x[vars[j]];
        for (int i = 0; i < m; i++)
          for (int e = R.start[i]; e < R.start[i + 1]; e++) step[R.cols[e]] += R.vals[e] * y[i];
        for (int j = 0; j < n; j++) sys.x[vars[j]] += step[j] / weight[vars[j]];
        Rows next;
        evaluate(next);
        bool finite = true;
        for (double v : next.f) finite = finite && std::isfinite(v);
        if (finite && squares(next.f) < squares(R.f)) {
          R = std::move(next), moved = true;
        } else {
          for (int j = 0; j < n; j++) sys.x[vars[j]] = before[j];
        }
      }
      if (!moved) break;
    }
    if (!(worst(R.f) <= tol)) solved = false;
    if (!analyse) continue;
    // What it holds: its rows made unit and taken one by one, each less what the earlier ones already say (twice over);
    // one left with almost nothing says nothing new. (Rows touch few numbers, and so do most of what's kept of them: each
    // kept as its numbers alone.)
    struct Sparse {
      std::vector<int> at;
      std::vector<double> v;
    };
    std::vector<Sparse> basis;
    std::vector<int> rowSource;
    for (int k : srcs) {
      std::vector<Row> rows;
      sys.rows(sources[k], rows);
      for (size_t q = 0; q < rows.size(); q++) rowSource.push_back(k);
    }
    std::vector<double> v(n, 0);
    std::vector<char> on(n, 0);
    for (int i = 0; i < m; i++) {
      std::vector<int> support;
      for (int e = R.start[i]; e < R.start[i + 1]; e++) {
        int j = R.cols[e];
        if (!on[j]) on[j] = 1, support.push_back(j);
        v[j] += R.vals[e];
      }
      double len = 0;
      for (int j : support) len += v[j] * v[j];
      len = std::sqrt(len);
      if (len > 1e-300)
        for (int j : support) v[j] /= len;
      for (int pass = 0; pass < 2; pass++)
        for (const auto &b : basis) {
          double d = 0;
          for (size_t e = 0; e < b.at.size(); e++) d += v[b.at[e]] * b.v[e];
          if (d == 0) continue;
          for (size_t e = 0; e < b.at.size(); e++) {
            int j = b.at[e];
            if (!on[j]) on[j] = 1, support.push_back(j);
            v[j] -= d * b.v[e];
          }
        }
      std::sort(support.begin(), support.end());
      double left = 0;
      for (int j : support) left += v[j] * v[j];
      left = std::sqrt(left);
      if (!(left > 1e-8) || !(len > 1e-300)) {
        noteDependent(rowSource[i]);
      } else {
        Sparse b;
        for (int j : support)
          if (v[j] != 0) b.at.push_back(j), b.v.push_back(v[j] / left);
        basis.push_back(std::move(b));
      }
      for (int j : support) v[j] = 0, on[j] = 0;
    }
    freedom += n - (int)basis.size();
    std::vector<double> in(n, 0);
    for (const auto &b : basis)
      for (size_t e = 0; e < b.at.size(); e++) in[b.at[e]] += b.v[e] * b.v[e];
    for (int j = 0; j < n; j++) determined[vars[j]] = 1 - in[j] <= 1e-8;
  }
  // Rules on held points alone: they hold already, or never can.
  for (int k : constants) {
    std::vector<Row> rows;
    sys.rows(sources[k], rows);
    for (const Row &row : rows)
      if (std::fabs(row.f) > tol) solved = false;
    noteDependent(k);
  }
  double residual = 0;
  for (size_t k = 0; k < sources.size(); k++) {
    std::vector<Row> rows;
    sys.rows(sources[k], rows);
    for (const Row &row : rows) residual = std::max(residual, std::fabs(row.f));
  }
  if (!analyse) freedom = -1;
  rep.solved = solved, rep.freedom = freedom, rep.dependent = dependent, rep.conflicting = !solved, rep.residual = residual;
  rep.pointFixed.assign(np, 0), rep.curveFixed.assign(nc, 0);
  for (int i = 0; i < np; i++) rep.pointFixed[i] = used[i] && (sys.vx[i] < 0 || (determined[sys.vx[i]] && determined[sys.vy[i]]));
  for (int c = 0; c < nc; c++) {
    const SketchCurve &cv = s.curves[c];
    int count = cv.kind == BK_CURVE_LINE ? 2 : cv.kind == BK_CURVE_ARC ? 3 : 1;
    bool fixed = true;
    for (int k = 0; k < count; k++) fixed = fixed && rep.pointFixed[cv.p[k]];
    if (cv.kind == BK_CURVE_CIRCLE && sys.vr[c] >= 0) fixed = fixed && determined[sys.vr[c]];
    rep.curveFixed[c] = fixed;
  }
  if (!solved) {
    // (Left as it was: a sketch that can't hold its rules isn't moved.)
    (void)start;
    return true;
  }
  for (int i = 0; i < np; i++)
    if (sys.vx[i] >= 0) s.points[i].x = sys.x[sys.vx[i]], s.points[i].y = sys.x[sys.vy[i]];
  for (int c = 0; c < nc; c++)
    if (sys.vr[c] >= 0) s.curves[c].radius = sys.x[sys.vr[c]];
  return true;
}

}  // namespace bce
