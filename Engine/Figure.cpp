// Human figures: a skeleton posed by its numbers, smooth convex parts round its bones (ellipsoids, some cut flat, and round
// cones from a ball at one end to a ball at the other), each blended into the next only near the joint they share; the
// surface where the nearest part (or blend) is at no distance, found on a grid a 160th of the height apart, three times
// finer each way at the hands (each finger jointed), the face and the toes. Hair is solid, in a few styles. Everything
// is worked out with + − × ÷ and √ (and Trig's sines) in a fixed order, so every machine makes the same figure to the bit.
#include "Engine/Figure.hpp"

#include "BcadKernel.h"
#include "Engine/Sculpt.hpp"

#include <array>
#include <list>
#include <memory>
#include <mutex>
#include <vector>

namespace bce {

namespace {

const double degree = 3.14159265358979323846 / 180;

const double lowest[FigureNumbers] = {0, 10, 0.7, 0.5, 0.8, 0.5, 0.75, 0.8, 0.85, 0.85, 0.8, -40, -70, -30, -20, -40, -25,
                                      0, -50, 0, 0, -50, 0, -30, -10, 0, -30, -10, 0, -70, 0, 0, -70, 0, 0, 0, 0.5};
const double highest[FigureNumbers] = {1, 5000, 1.4, 1.6, 1.25, 1.6, 1.4, 1.3, 1.15, 1.15, 1.25, 50, 70, 30, 60, 40, 25,
                                       170, 170, 150, 170, 170, 150, 110, 50, 140, 110, 50, 140, 80, 100, 100, 80, 100, 100,
                                       BK_HAIR_COUNT - 1, 1.5};
const char *names[FigureNumbers] = {"sex", "height", "build", "muscle", "shoulders", "chest", "waist", "hips", "arms", "legs",
                                    "head", "nod", "turn", "tilt", "bend", "twist", "lean", "left arm's raise",
                                    "left arm's forward", "left elbow", "right arm's raise", "right arm's forward", "right elbow",
                                    "left hip", "left leg's out", "left knee", "right hip", "right leg's out", "right knee",
                                    "left wrist", "left hand's curl", "left hand's spread", "right wrist", "right hand's curl",
                                    "right hand's spread", "hair", "hair's volume"};

// Each pose's numbers from the nod to the hands: the head's nod, turn, tilt; the torso's bend, twist, lean; the left
// arm's raise, forward, elbow; the right arm's; the left leg's hip, out, knee; the right leg's; the left hand's wrist,
// curl, spread; the right hand's. (The hair isn't a pose's.)
const int posed = BK_FIG_RIGHT_SPREAD + 1 - BK_FIG_NOD;
const double poses[BK_POSE_COUNT][posed] = {
    {0, 0, 0, 0, 0, 0, 8, 0, 10, 8, 0, 10, 0, 0, 0, 0, 0, 0, 0, 25, 15, 0, 25, 15},            // standing
    {0, 0, 0, 0, 0, 0, 90, 0, 0, 90, 0, 0, 0, 4, 0, 0, 4, 0, 0, 25, 15, 0, 25, 15},            // T
    {0, 0, 0, 0, 4, 0, 8, -20, 15, 8, 25, 25, 20, 0, 5, -15, 0, 5, 0, 25, 15, 0, 25, 15},      // walking
    {0, 0, 0, 0, 0, 0, 8, 20, 62, 8, 20, 62, 90, 6, 90, 90, 6, 90, 0, 25, 15, 0, 25, 15},      // sitting
    {0, 0, 5, 0, 0, 0, 8, 0, 10, 35, 95, 100, 0, 0, 0, 0, 0, 0, 0, 25, 15, -10, 0, 60},       // waving
};

// A turn, row by row.
struct M3 {
  double m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  V3 operator*(V3 v) const { return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z, m[6] * v.x + m[7] * v.y + m[8] * v.z}; }
  M3 operator*(const M3 &b) const {
    M3 r;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) r.m[3 * i + j] = m[3 * i] * b.m[j] + m[3 * i + 1] * b.m[3 + j] + m[3 * i + 2] * b.m[6 + j];
    return r;
  }
  V3 column(int j) const { return {m[j], m[3 + j], m[6 + j]}; }
};

M3 turnX(double a) {
  double c = trig::cos(a * degree), s = trig::sin(a * degree);
  M3 r;
  r.m[4] = c, r.m[5] = -s, r.m[7] = s, r.m[8] = c;
  return r;
}
M3 turnY(double a) {
  double c = trig::cos(a * degree), s = trig::sin(a * degree);
  M3 r;
  r.m[0] = c, r.m[2] = s, r.m[6] = -s, r.m[8] = c;
  return r;
}
M3 turnZ(double a) {
  double c = trig::cos(a * degree), s = trig::sin(a * degree);
  M3 r;
  r.m[0] = c, r.m[1] = -s, r.m[3] = s, r.m[4] = c;
  return r;
}

// The same turn seen across x = 0.
M3 mirrored(const M3 &a) {
  M3 r = a;
  r.m[1] = -r.m[1], r.m[2] = -r.m[2], r.m[3] = -r.m[3], r.m[6] = -r.m[6];
  return r;
}

// Where a part of the skeleton puts its points: turned by R about `pivot` (on x = 0), its own x first mirrored for the
// right side (so a right limb is a left limb's mirror image to the bit).
struct Frame {
  M3 R;
  V3 pivot;
  bool mirror = false;
  V3 at(V3 p) const {
    V3 q = p - pivot;
    if (mirror) q.x = -q.x;
    return pivot + R * q;
  }
  M3 turn(const M3 &a) const { return R * (mirror ? mirrored(a) : a); }
  Frame side(bool right) const {
    Frame f = *this;
    f.mirror = right;
    return f;
  }
};

struct Part {
  enum Kind { Ellipsoid, Cone, Foot, Slab } kind = Ellipsoid;
  // Added and blended with the rest (0); carved out of what's made so far, smoothly by `soft` (1); added on top, after
  // the carving (2).
  int layer = 0;
  double soft = 0;
  // The head's (and the face's): what the carving cuts (a hand passing through the face, or the hair, it leaves be).
  bool head = false;
  V3 a, b;          // an ellipsoid's (or a slab's) middle (a); a cone's (or a foot's) balls' middles
  double ra = 0, rb = 0;  // their radii; a slab's rounding (ra)
  V3 r;             // an ellipsoid's semi-axes, a slab's half sizes before its rounding (along R's columns)
  M3 R;             // an ellipsoid's (a slab's) axes; a foot's up (its third column: its sole the plane through a and b across it)
  V3 lo, hi;        // its box
  double grow = 1;  // how much further than its distance it may say it is (an ellipsoid's longest axis over its shortest)
  double rmin = 0;
  V3 ba;
  double l2 = 0, rr = 0, a2 = 0, il2 = 0;
  // An ellipsoid cut flat: only what lies where dot(cut, p) ≤ cutAt kept.
  bool clipped = false;
  V3 cut{0, 0, 0};
  double cutAt = 0;
};

// Two parts blended together near a joint: by k at the joint, less further off, none from R away.
struct Joint {
  int a, b;
  V3 at;
  double k, R2;
};

// The parts and joints that may matter near each of a grid of bins over the body (for any point in a bin, every part
// within `margin` of it, its box grown by margin times its grow, and every joint whose reach takes it in).
struct Bins {
  V3 lo{0, 0, 0};
  double step = 1;
  int n[3] = {1, 1, 1};
  std::vector<uint32_t> partStart, parts, jointStart, joints;
  size_t at(V3 p) const {
    int c[3];
    for (int a = 0; a < 3; a++) c[a] = (int)std::min((double)n[a] - 1, std::max(0.0, std::floor((p[a] - lo[a]) / step)));
    return (size_t)c[0] + (size_t)n[0] * ((size_t)c[1] + (size_t)n[1] * (size_t)c[2]);
  }
};

struct Body {
  std::vector<Part> parts;
  std::vector<Joint> joints;
  V3 lo, hi, anchor;
  double kmax = 0;
  // Where the grid is made finer (the hands, the face, the toes): boxes; and the radius of the thinnest part there.
  std::vector<std::pair<V3, V3>> fine;
  double finest = INFINITY;
  Bins bins;
};

// Each point's values worked out once.
struct Eval {
  std::vector<double> val;
  std::vector<uint32_t> stamp;
  uint32_t tick = 0;
  explicit Eval(size_t parts) : val(parts), stamp(parts, 0) {}
};

double sign(double x) { return x > 0 ? 1 : x < 0 ? -1 : 0; }

// A round cone's distance (exact: Inigo Quilez's).
double coneDistance(const Part &P, V3 p) {
  V3 pa = p - P.a;
  double y = dot(pa, P.ba), z = y - P.l2;
  V3 w = pa * P.l2 - P.ba * y;
  double x2 = dot(w, w), y2 = y * y * P.l2, z2 = z * z * P.l2;
  double k = sign(P.rr) * P.rr * P.rr * x2;
  if (sign(z) * P.a2 * z2 > k) return std::sqrt(x2 + z2) * P.il2 - P.rb;
  if (sign(y) * P.a2 * y2 < k) return std::sqrt(x2 + y2) * P.il2 - P.ra;
  return (std::sqrt(x2 * P.a2 * P.il2) + y * P.rr) * P.il2 - P.ra;
}

// How far p is from a part (never more than it is; never off by more than a step for a step along the way).
double distance(const Part &P, V3 p) {
  switch (P.kind) {
    case Part::Ellipsoid: {
      V3 d = p - P.a;
      double x = dot(P.R.column(0), d) / P.r.x, y = dot(P.R.column(1), d) / P.r.y, z = dot(P.R.column(2), d) / P.r.z;
      double e = (std::sqrt(x * x + y * y + z * z) - 1) * P.rmin;
      return P.clipped ? std::max(e, dot(P.cut, p) - P.cutAt) : e;
    }
    case Part::Cone: return coneDistance(P, p);
    case Part::Foot: return std::max(coneDistance(P, p), -dot(P.R.column(2), p - P.a));
    case Part::Slab: {
      V3 d = p - P.a, q{std::fabs(dot(P.R.column(0), d)) - P.r.x, std::fabs(dot(P.R.column(1), d)) - P.r.y, std::fabs(dot(P.R.column(2), d)) - P.r.z};
      V3 o{std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0)};
      return norm(o) + std::min(std::max({q.x, q.y, q.z}), 0.0) - P.ra;
    }
  }
  return 0;
}


// The two smoothly together (as much as k rounds them where they meet).
double smoothMin(double a, double b, double k) {
  if (!(k > 0)) return std::min(a, b);
  double h = std::min(1.0, std::max(0.0, 0.5 + 0.5 * (b - a) / k));
  return b + (a - b) * h - k * h * (1 - h);
}
// What's left of a once b's taken away from it, smoothly (by k).
double smoothMax(double a, double b, double k) { return -smoothMin(-a, -b, k); }

// The body's distance at p, as far as `margin` (further: just that; margin no more than the bins were made for): only
// parts within margin of p looked at. The parts added and blended first, then the carvings (of the head, and what's
// blended into it, alone), then what's added on top.
double field(const Body &B, V3 p, double margin, Eval &E) {
  if (++E.tick == 0) std::fill(E.stamp.begin(), E.stamp.end(), 0), E.tick = 1;
  size_t bin = B.bins.at(p);
  double best = margin, top = margin;  // the rest's, the head's
  bool layered = false;
  auto near = [&](const Part &P, double m) {
    return !(p.x < P.lo.x - m || p.x > P.hi.x + m || p.y < P.lo.y - m || p.y > P.hi.y + m || p.z < P.lo.z - m || p.z > P.hi.z + m);
  };
  for (uint32_t s = B.bins.partStart[bin]; s < B.bins.partStart[bin + 1]; s++) {
    uint32_t i = B.bins.parts[s];
    const Part &P = B.parts[i];
    if (P.layer) {
      layered = true;
      continue;
    }
    if (!near(P, margin * P.grow)) continue;
    E.val[i] = distance(P, p), E.stamp[i] = E.tick;
    double &to = P.head ? top : best;
    to = std::min(to, E.val[i]);
  }
  for (uint32_t s = B.bins.jointStart[bin]; s < B.bins.jointStart[bin + 1]; s++) {
    const Joint &J = B.joints[B.bins.joints[s]];
    double d2 = norm2(p - J.at);
    if (!(d2 < J.R2)) continue;
    double w = 1 - d2 / J.R2;
    for (int i : {J.a, J.b})
      if (E.stamp[i] != E.tick) E.val[i] = distance(B.parts[i], p), E.stamp[i] = E.tick;
    double &to = B.parts[J.a].head || B.parts[J.b].head ? top : best;
    to = std::min(to, smoothMin(E.val[J.a], E.val[J.b], J.k * w * w));
  }
  if (!layered) return std::min(best, top);
  for (int layer = 1; layer <= 2; layer++) {
    if (layer == 2) best = std::min(best, top);
    for (uint32_t s = B.bins.partStart[bin]; s < B.bins.partStart[bin + 1]; s++) {
      const Part &P = B.parts[B.bins.parts[s]];
      if (P.layer != layer || !near(P, (margin + P.soft) * P.grow)) continue;
      double d = distance(P, p);
      if (layer == 1) top = smoothMax(top, -d, P.soft);
      else best = std::min(best, d);
    }
  }
  return best;
}

// The bins for margins up to `margin`, over the body's box grown by `around`.
void binned(Body &B, double margin, double around) {
  Bins &G = B.bins;
  V3 span = B.hi - B.lo;
  G.step = std::max({span.x, span.y, span.z}) / 16 + 1e-9;
  G.lo = B.lo - V3{around, around, around};
  for (int a = 0; a < 3; a++) G.n[a] = std::max(1, (int)std::ceil((span[a] + 2 * around) / G.step));
  size_t nb = (size_t)G.n[0] * G.n[1] * G.n[2];
  auto fill = [&](size_t count, auto box, std::vector<uint32_t> &start, std::vector<uint32_t> &list) {
    std::vector<std::vector<uint32_t>> per(nb);
    for (size_t i = 0; i < count; i++) {
      V3 lo, hi;
      box(i, lo, hi);
      int c0[3], c1[3];
      for (int a = 0; a < 3; a++) {
        c0[a] = (int)std::min((double)G.n[a] - 1, std::max(0.0, std::floor((lo[a] - G.lo[a]) / G.step)));
        c1[a] = (int)std::min((double)G.n[a] - 1, std::max(0.0, std::floor((hi[a] - G.lo[a]) / G.step)));
      }
      for (int z = c0[2]; z <= c1[2]; z++)
        for (int y = c0[1]; y <= c1[1]; y++)
          for (int x = c0[0]; x <= c1[0]; x++) per[(size_t)x + (size_t)G.n[0] * ((size_t)y + (size_t)G.n[1] * (size_t)z)].push_back((uint32_t)i);
    }
    start.assign(nb + 1, 0), list.clear();
    for (size_t b = 0; b < nb; b++) start[b + 1] = start[b] + (uint32_t)per[b].size(), list.insert(list.end(), per[b].begin(), per[b].end());
  };
  fill(B.parts.size(), [&](size_t i, V3 &lo, V3 &hi) {
    const Part &P = B.parts[i];
    double m = (margin + P.soft) * P.grow;
    lo = P.lo - V3{m, m, m}, hi = P.hi + V3{m, m, m};
  }, G.partStart, G.parts);
  fill(B.joints.size(), [&](size_t i, V3 &lo, V3 &hi) {
    const Joint &J = B.joints[i];
    double r = std::sqrt(J.R2);
    lo = J.at - V3{r, r, r}, hi = J.at + V3{r, r, r};
  }, G.jointStart, G.joints);
}

// Parts made at a height (all sizes given as parts of it), none thinner than 0.4 mm across… a radius of 0.4 mm.
struct Maker {
  Body body;
  double H;
  const double least = 0.4;
  bool head = false;  // the parts made the head's
  explicit Maker(double height) : H(height) {}

  int add(Part &P) {
    P.head = head;
    body.parts.push_back(P);
    return (int)body.parts.size() - 1;
  }
  // (A detail carved out or laid over the rest as small as asked: made thicker, it would cut through what it's in, or
  // stand out of it.)
  Part ellipsoidOf(V3 c, V3 r, const M3 &R, bool detail = false) {
    Part P;
    P.kind = Part::Ellipsoid, P.a = c * H, P.R = R;
    double at = detail ? 0 : least;
    P.r = {std::max(r.x * H, at), std::max(r.y * H, at), std::max(r.z * H, at)};
    P.rmin = std::min({P.r.x, P.r.y, P.r.z});
    P.grow = std::max({P.r.x, P.r.y, P.r.z}) / P.rmin;
    for (int k = 0; k < 3; k++) {
      double ex = R.m[3 * k] * P.r.x, ey = R.m[3 * k + 1] * P.r.y, ez = R.m[3 * k + 2] * P.r.z;
      double e = std::sqrt(ex * ex + ey * ey + ez * ez);
      P.lo[k] = P.a[k] - e, P.hi[k] = P.a[k] + e;
    }
    return P;
  }
  int ellipsoid(V3 c, V3 r, const M3 &R) {
    Part P = ellipsoidOf(c, r, R);
    return add(P);
  }
  // An ellipsoid carved smoothly out of what's made (by `soft`), or added on top of it after the carving.
  void carve(V3 c, V3 r, const M3 &R, double soft) {
    Part P = ellipsoidOf(c, r, R, true);
    P.layer = 1, P.soft = soft * H;
    add(P);
  }
  void over(V3 c, V3 r, const M3 &R) {
    Part P = ellipsoidOf(c, r, R, true);
    P.layer = 2;
    add(P);
  }
  // A box (half sizes along R's columns, any of them 0) rounded all round by `round`: its box exactly.
  int slab(V3 c, V3 half, double round, const M3 &R) {
    Part P;
    P.kind = Part::Slab, P.a = c * H, P.R = R, P.r = half * H, P.ra = std::max(round * H, least);
    for (int k = 0; k < 3; k++) {
      double e = std::fabs(R.m[3 * k]) * P.r.x + std::fabs(R.m[3 * k + 1]) * P.r.y + std::fabs(R.m[3 * k + 2]) * P.r.z + P.ra;
      P.lo[k] = P.a[k] - e, P.hi[k] = P.a[k] + e;
    }
    return add(P);
  }
  // An ellipsoid with what lies beyond the plane through `at` facing `out` (unit) cut away; its box exactly what's left's.
  int clipped(V3 c, V3 r, const M3 &R, V3 at, V3 out) {
    Part P = ellipsoidOf(c, r, R);
    P.clipped = true, P.cut = out, P.cutAt = dot(out, at * H);
    // In the unit ball's terms (x = a + M u, M = R·diag(r)): kept where dot(A, u) ≤ -b.
    V3 A{0, 0, 0};
    for (int k = 0; k < 3; k++) A[k] = dot(R.column(k), out) * P.r[k];
    double b = dot(out, P.a) - P.cutAt, la = norm(A);
    for (int axis = 0; axis < 3; axis++)
      for (int side = -1; side <= 1; side += 2) {
        // The farthest the kept part reaches along e (+ or − the axis): the ball's own farthest point if kept, else the
        // farthest on the circle where the plane cuts it.
        V3 g{0, 0, 0};
        for (int k = 0; k < 3; k++) g[k] = side * R.m[3 * axis + k] * P.r[k];
        double lg = norm(g), reach;
        if (!(la > 0) || dot(A, g) <= -b * lg) {
          reach = lg;
        } else {
          V3 ah = A / la, gp = g - ah * dot(g, ah);
          double t = std::max(0.0, 1 - (b / la) * (b / la));
          reach = -b * dot(g, ah) / la + std::sqrt(t) * norm(gp);
        }
        double x = P.a[axis] + side * reach;
        if (side < 0) P.lo[axis] = x;
        else P.hi[axis] = x;
      }
    return add(P);
  }
  Part coneOf(V3 a, double ra, V3 b, double rb) {
    Part P;
    P.kind = Part::Cone, P.a = a * H, P.b = b * H, P.ra = std::max(ra * H, least), P.rb = std::max(rb * H, least);
    P.ba = P.b - P.a, P.l2 = dot(P.ba, P.ba), P.rr = P.ra - P.rb, P.a2 = P.l2 - P.rr * P.rr, P.il2 = 1 / P.l2;
    for (int k = 0; k < 3; k++) P.lo[k] = std::min(P.a[k] - P.ra, P.b[k] - P.rb), P.hi[k] = std::max(P.a[k] + P.ra, P.b[k] + P.rb);
    return P;
  }
  int cone(V3 a, double ra, V3 b, double rb) {
    Part P = coneOf(a, ra, b, rb);
    return add(P);
  }
  // Two half balls (heel and toe, their flat sides on the sole) and what's between them.
  int foot(V3 heel, double rh, V3 toe, double rt, const M3 &R) {
    Part P = coneOf(heel, rh, toe, rt);
    P.kind = Part::Foot, P.R = R;
    for (int k = 0; k < 3; k++) {
      double up = R.m[3 * k + 2], across = std::sqrt(std::max(0.0, 1 - up * up));
      double hiA = P.a[k] + (up >= 0 ? P.ra : P.ra * across), hiB = P.b[k] + (up >= 0 ? P.rb : P.rb * across);
      double loA = P.a[k] - (up <= 0 ? P.ra : P.ra * across), loB = P.b[k] - (up <= 0 ? P.rb : P.rb * across);
      P.lo[k] = std::min(loA, loB), P.hi[k] = std::max(hiA, hiB);
    }
    return add(P);
  }
  void join(int a, int b, V3 at, double k, double R) {
    body.joints.push_back({a, b, at * H, k * H, R * H * R * H});
    body.kmax = std::max(body.kmax, k * H);
  }
  // The parts made since `from` as a place to make the grid finer.
  void fine(size_t from) {
    V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
    for (size_t i = from; i < body.parts.size(); i++) {
      const Part &P = body.parts[i];
      lo = vmin(lo, P.lo), hi = vmax(hi, P.hi);
      if (P.layer) continue;
      double thin = P.kind == Part::Ellipsoid ? P.rmin : P.kind == Part::Slab ? P.ra + std::min({P.r.x, P.r.y, P.r.z}) : std::min(P.ra, P.rb);
      body.finest = std::min(body.finest, thin);
    }
    if (from < body.parts.size()) body.fine.push_back({lo, hi});
  }
};

// The figure's parts, centred on their box.
Body build(const FigureSpec &spec) {
  const double *v = spec.v;
  double s = v[BK_FIG_SEX], H = v[BK_FIG_HEIGHT], bulk = v[BK_FIG_BUILD], muscle = v[BK_FIG_MUSCLE], shoulders = v[BK_FIG_SHOULDERS];
  double chest = v[BK_FIG_CHEST], waist = v[BK_FIG_WAIST], hips = v[BK_FIG_HIPS], arms = v[BK_FIG_ARMS], legs = v[BK_FIG_LEGS];
  double head = v[BK_FIG_HEAD];
  auto mix = [&](double man, double woman) { return man + (woman - man) * s; };
  Maker mk(H);
  double girth = 1 + 0.3 * (muscle - 1);  // the limbs' (some of it muscle)
  double small = mix(1, 0.92);            // hands and feet
  // Heights (parts of the height): the ankle and the hip joints; above them, stretched to reach the crown at 1.
  double ankle = 0.039 * small, thigh = 0.235 * legs, shin = 0.241 * legs;
  double hipZ = ankle + thigh + shin, headHeight = 0.13 * head, chin = 1 - headHeight;
  double u = (chin - hipZ) / (0.870 - 0.515);
  auto Z = [&](double z) { return hipZ + (z - 0.515) * u; };
  const M3 none;
  const Frame root{none, V3{0, 0, 0}};

  // The pelvis and buttocks stay where they are as the torso turns above them.
  int pelvis = mk.ellipsoid({0, 0.006, Z(0.525)}, {mix(0.083, 0.092) * hips * bulk, mix(0.060, 0.064) * bulk, 0.07 * u}, none);
  int buttock[2];
  for (int side = 0; side < 2; side++) {
    V3 c = root.side(side).at({0.040 * hips, mix(0.034, 0.038), hipZ - 0.024 * u});
    buttock[side] = mk.ellipsoid(c, {mix(0.043, 0.048) * hips * bulk, mix(0.034, 0.042) * bulk, mix(0.052, 0.058) * u}, none);
    mk.join(pelvis, buttock[side], c, 0.025, 0.07);
  }

  // The torso, turned at the small of the back.
  V3 lumbar{0, 0, Z(0.60)};
  const Frame torso{turnZ(v[BK_FIG_TWIST]) * turnY(v[BK_FIG_LEAN]) * turnX(v[BK_FIG_BEND]), lumbar};
  int belly = mk.ellipsoid(torso.at({0, 0, Z(0.605)}), {mix(0.070, 0.062) * waist * bulk, mix(0.054, 0.050) * waist * bulk * bulk, 0.09 * u}, torso.R);
  mk.join(pelvis, belly, lumbar, 0.03, 0.10);
  V3 ribsAt{0, 0.008, Z(0.72)}, ribsSize{mix(0.092, 0.078) * bulk * (0.6 + 0.4 * shoulders), mix(0.066, 0.058) * bulk, mix(0.13, 0.12) * u};
  int ribs = mk.ellipsoid(torso.at(ribsAt), ribsSize, torso.R);
  mk.join(belly, ribs, torso.at({0, 0, Z(0.655)}), 0.03, 0.11);
  // How far forward the ribs' front is at (x, z), and how far back their back (in the torso's own frame).
  auto front = [&](double x, double z) {
    double a = x / ribsSize.x, c = (z - ribsAt.z) / ribsSize.z;
    return ribsAt.y - ribsSize.y * std::sqrt(std::max(0.0, 1 - a * a - c * c));
  };
  auto back = [&](double x, double z) { return 2 * ribsAt.y - front(x, z); };
  // A man's pecs and a woman's bust, as much of each as the figure's sex has, set into the ribs' front where they are.
  for (int side = 0; side < 2; side++) {
    Frame f = torso.side(side);
    if (s < 1) {
      double k = (1 - s) * chest, x = 0.46 * ribsSize.x, z = Z(0.752);
      V3 size{0.046 * k, 0.012 * k * (0.6 + 0.4 * muscle), 0.030 * k};
      V3 c = f.at({x, front(x, z) + 0.5 * size.y, z});
      int pec = mk.ellipsoid(c, size, torso.R);
      mk.join(ribs, pec, c, 0.012, 0.05);
    }
    if (s > 0) {
      double x = 0.55 * ribsSize.x, z = Z(0.722);
      V3 size = V3{0.036, 0.032, 0.034} * (s * chest);
      double y = front(x, z) - 0.45 * size.y;
      int breast = mk.ellipsoid(f.at({x, y, z}), size, torso.R);
      mk.join(ribs, breast, f.at({x, y + 0.4 * size.y, z + 0.6 * size.z}), 0.02, 0.05);
    }
  }
  // The shoulders (the line from the neck out to each shoulder joint) and the neck.
  V3 shoulder{mix(0.112, 0.100) * shoulders, 0.010, Z(0.812)};
  int neck = mk.cone(torso.at({0, 0.010, Z(0.83)}), mix(0.034, 0.028) * bulk, torso.at({0, 0.004, Z(0.885)}), mix(0.030, 0.025) * bulk);
  mk.join(ribs, neck, torso.at({0, 0.01, Z(0.84)}), 0.02, 0.06);
  int yoke[2];
  for (int side = 0; side < 2; side++) {
    Frame f = torso.side(side);
    yoke[side] = mk.cone(f.at({0, 0.012, Z(0.832)}), mix(0.040, 0.034) * bulk, f.at(shoulder), mix(0.030, 0.026) * bulk);
    mk.join(ribs, yoke[side], f.at({0.06, 0.01, Z(0.825)}), 0.02, 0.07);
    mk.join(neck, yoke[side], f.at({0.01, 0.012, Z(0.836)}), 0.02, 0.05);
    // The collarbone, from the top of the breastbone out to the shoulder; the shoulder blade on the back.
    double zc = Z(0.815);
    int collar = mk.cone(f.at({0.014, front(0.014, zc) + 0.006, zc}), 0.0062 * bulk, f.at({shoulder.x - 0.012, -0.010, Z(0.822)}), 0.0055 * bulk);
    mk.join(ribs, collar, f.at({0.03, front(0.03, zc), zc}), 0.012, 0.05);
    mk.join(yoke[side], collar, f.at({shoulder.x - 0.02, -0.006, Z(0.822)}), 0.012, 0.04);
    double xb = 0.042 * shoulders, zb = Z(0.765);
    int blade = mk.ellipsoid(f.at({xb, back(xb, zb) - 0.007, zb}), V3{0.030, 0.009, 0.040} * bulk, torso.R);
    mk.join(ribs, blade, f.at({xb, back(xb, zb) - 0.004, zb}), 0.014, 0.06);
  }

  // The head, turned on the neck.
  V3 nape{0, 0.008, Z(0.845)};
  M3 nod = turnZ(v[BK_FIG_TURN]) * turnY(v[BK_FIG_TILT]) * turnX(v[BK_FIG_NOD]);
  auto onHead = [&](V3 p) { return torso.at(nape + nod * (p - nape)); };
  M3 headTurn = torso.R * nod;
  double crown = 0.057 * head;
  mk.head = true;
  int skull = mk.ellipsoid(onHead({0, 0.006 * head, 1 - crown}), {0.045 * head * mix(1, 0.96), 0.053 * head, crown}, headTurn);
  int jaw = mk.ellipsoid(onHead({0, -0.004 * head, chin + 0.032 * head}), {0.034 * head * mix(1, 0.9), 0.039 * head, 0.032 * head}, headTurn);
  mk.join(skull, jaw, onHead({0, -0.004 * head, chin + 0.045 * head}), 0.035, 0.08);
  mk.join(neck, jaw, onHead({0, 0.004, chin + 0.012 * head}), 0.02, 0.05);
  mk.join(neck, skull, onHead({0, 0.01, chin + 0.03 * head}), 0.02, 0.06);

  // The face (all of it made finer): brows over the eyes, cheekbones, the nose (its bridge, tip and wings), the lips, the
  // chin and the ears, each blended into the skull or the jaw all over; then the eye sockets, the line between the lips
  // and each ear's bowl carved out; then the eyes, set in their sockets.
  size_t faceFrom = mk.body.parts.size();
  {
    double hd = head, eyeZ = 1 - 0.065 * hd, noseZ = eyeZ - 0.031 * hd, mouthZ = eyeZ - 0.046 * hd, lip = mix(1, 1.1);
    auto F = [&](double x, double y, double z) { return onHead(V3{x * hd, y * hd, z}); };
    double earX = 0.045 * mix(1, 0.96) + 0.0015;
    for (int side = 0; side < 2; side++) {
      double sx = side ? -1 : 1;
      int brow = mk.ellipsoid(F(sx * 0.015, -0.0410, eyeZ + 0.0100 * hd), V3{0.0150, 0.0055 * mix(1, 0.7), 0.0060} * hd, headTurn * turnZ(sx * 15));
      mk.join(skull, brow, F(sx * 0.015, -0.038, eyeZ + 0.0105 * hd), 0.020 * hd, 0.05 * hd);
      int cheek = mk.ellipsoid(F(sx * 0.0245, -0.0300, eyeZ - 0.019 * hd), V3{0.0160, 0.0062, 0.0110} * hd, headTurn);
      mk.join(skull, cheek, F(sx * 0.0245, -0.028, eyeZ - 0.016 * hd), 0.024 * hd, 0.06 * hd);
      mk.join(jaw, cheek, F(sx * 0.0245, -0.028, eyeZ - 0.024 * hd), 0.024 * hd, 0.06 * hd);
      int ear = mk.ellipsoid(F(sx * earX, 0.006, eyeZ - 0.010 * hd), V3{0.0038, 0.0105, 0.0185} * hd, headTurn * turnX(-12));
      mk.join(skull, ear, F(sx * (earX - 0.003), 0.006, eyeZ - 0.010 * hd), 0.006 * hd, 0.025 * hd);
    }
    int bridge = mk.cone(F(0, -0.0425, eyeZ + 0.004 * hd), 0.0040 * hd, F(0, -0.0545, noseZ + 0.0015 * hd), 0.0052 * hd);
    mk.join(skull, bridge, F(0, -0.042, eyeZ), 0.014 * hd, 0.04 * hd);
    int tip = mk.ellipsoid(F(0, -0.0530, noseZ + 0.0010 * hd), V3{0.0062, 0.0058, 0.0055} * hd, headTurn);
    mk.join(bridge, tip, F(0, -0.053, noseZ + 0.003 * hd), 0.004 * hd, 0.015 * hd);
    mk.join(jaw, tip, F(0, -0.047, noseZ - 0.002 * hd), 0.006 * hd, 0.02 * hd);
    for (int side = 0; side < 2; side++) {
      double sx = side ? -1 : 1;
      int wing = mk.ellipsoid(F(sx * 0.0070, -0.0478, noseZ + 0.0005 * hd), V3{0.0048, 0.0052, 0.0042} * hd, headTurn);
      mk.join(tip, wing, F(sx * 0.005, -0.050, noseZ + 0.001 * hd), 0.004 * hd, 0.015 * hd);
      mk.join(jaw, wing, F(sx * 0.008, -0.044, noseZ), 0.006 * hd, 0.02 * hd);
    }
    int upperLip = mk.ellipsoid(F(0, -0.0405, mouthZ + 0.0030 * hd), V3{0.0135, 0.0046 * lip, 0.0040 * lip} * hd, headTurn);
    int lowerLip = mk.ellipsoid(F(0, -0.0395, mouthZ - 0.0042 * hd), V3{0.0122, 0.0050 * lip, 0.0042 * lip} * hd, headTurn);
    mk.join(jaw, upperLip, F(0, -0.038, mouthZ + 0.003 * hd), 0.014 * hd, 0.04 * hd);
    mk.join(jaw, lowerLip, F(0, -0.037, mouthZ - 0.004 * hd), 0.014 * hd, 0.04 * hd);
    int chinPart = mk.ellipsoid(F(0, -0.0345, chin + 0.0150 * hd), V3{0.0125 * mix(1, 0.88), 0.0080, 0.0100} * hd, headTurn);
    mk.join(jaw, chinPart, F(0, -0.032, chin + 0.0150 * hd), 0.022 * hd, 0.06 * hd);
    for (int side = 0; side < 2; side++) {
      double sx = side ? -1 : 1;
      mk.carve(F(sx * 0.0165, -0.0465, eyeZ + 0.0005 * hd), V3{0.0110, 0.0072, 0.0075} * hd, headTurn, 0.006 * hd);
      mk.carve(F(sx * (earX + 0.0034), 0.0065, eyeZ - 0.011 * hd), V3{0.0022, 0.0068, 0.0120} * hd, headTurn * turnX(-12), 0.0015 * hd);
      mk.over(F(sx * 0.0165, -0.0372, eyeZ), V3{0.0058, 0.0058, 0.0058} * hd, headTurn);
    }
    mk.carve(F(0, -0.0470, mouthZ - 0.0004 * hd), V3{0.0118, 0.0060, 0.0011} * hd, headTurn, 0.0015 * hd);
  }
  mk.head = false;
  mk.fine(faceFrom);

  // The hair, solid: a cap over the skull down to the hairline (a plane leaning back from the forehead to above the ears),
  // and as its style has it: a bob to the jaw, long down the back, a ponytail, a bun, or an afro all round.
  int style = (int)std::lround(v[BK_FIG_HAIR]);
  if (style > BK_HAIR_NONE) {
    double vol = v[BK_FIG_HAIR_VOLUME], hd = head, sx = 0.045 * hd * mix(1, 0.96), sy = 0.053 * hd;
    V3 sc{0, 0.006 * hd, 1 - crown};
    V3 lean = headTurn * V3{0, -0.75, -0.66} * (1 / std::sqrt(0.75 * 0.75 + 0.66 * 0.66));
    V3 line = onHead(sc + V3{0, -0.85 * sy, 0.022 * hd});
    double t = (style == BK_HAIR_SHORT ? 0.006 : 0.010) * hd * vol;
    int cap = -1;
    if (style != BK_HAIR_AFRO) cap = mk.clipped(onHead(sc + V3{0, 0.002 * hd, 0.004 * hd}), V3{sx + t, sy + t, crown + t}, headTurn, line, lean);
    V3 faceCut = headTurn * V3{0, -1, 0};
    switch (style) {
      case BK_HAIR_BOB:
      case BK_HAIR_LONG: {
        int mass = mk.clipped(onHead(sc + V3{0, 0.006 * hd, -0.030 * hd}), V3{sx + 0.014 * hd * vol, sy + 0.012 * hd * vol, crown + 0.014 * hd}, headTurn,
                              onHead(sc + V3{0, -0.6 * sy, 0}), faceCut);
        mk.join(cap, mass, onHead(sc + V3{0, 0.02 * hd, -0.02 * hd}), 0.010 * hd, 0.09 * hd);
        if (style == BK_HAIR_LONG) {
          // Down the back from the nape, over the shoulder blades.
          double zb = Z(0.78);
          V3 top = onHead(sc + V3{0, 0.8 * sy, -0.050 * hd}), low = torso.at({0, back(0, zb) + 0.010 * vol, zb});
          int fall = mk.cone(top, 0.026 * hd * vol, low, 0.030 * vol);
          int sheet = mk.ellipsoid(torso.at({0, back(0, zb) + 0.008 * vol, zb}), V3{0.050 * vol, 0.013 * vol, 0.070}, torso.R);
          mk.join(mass, fall, top, 0.012 * hd, 0.06 * hd);
          mk.join(fall, sheet, low, 0.012, 0.06);
        }
        break;
      }
      case BK_HAIR_PONYTAIL: {
        V3 tie = sc + V3{0, sy + 0.002 * hd, -0.026 * hd};
        int knot = mk.ellipsoid(onHead(tie), V3{0.011, 0.010, 0.011} * (hd * vol), headTurn);
        int tail = mk.cone(onHead(tie), 0.013 * hd * vol, onHead(sc + V3{0, sy + 0.012 * hd, -0.120 * hd}), 0.008 * hd * vol);
        mk.join(cap, knot, onHead(tie), 0.006 * hd, 0.03 * hd);
        mk.join(knot, tail, onHead(tie), 0.006 * hd, 0.03 * hd);
        break;
      }
      case BK_HAIR_BUN: {
        V3 at = sc + V3{0, sy + 0.010 * hd, 0.40 * crown};
        int bun = mk.ellipsoid(onHead(at), V3{0.021, 0.019, 0.020} * (hd * vol), headTurn);
        mk.join(cap, bun, onHead(sc + V3{0, sy, 0.40 * crown}), 0.008 * hd, 0.04 * hd);
        break;
      }
      case BK_HAIR_AFRO:
        mk.clipped(onHead(sc + V3{0, 0.004 * hd, 0.010 * hd}), V3{sx + 0.030 * hd * vol, sy + 0.028 * hd * vol, crown + 0.030 * hd * vol}, headTurn,
                   onHead(sc + V3{0, -0.95 * sy, 0.026 * hd}), lean);
        break;
      default: break;
    }
  }

  // The arms, each from its shoulder: raised out to the side, then forward; the forearm bent forward at the elbow; the
  // hand, its palm toward the body when hanging, the thumb in front.
  for (int side = 0; side < 2; side++) {
    int at = side ? BK_FIG_RIGHT_RAISE : BK_FIG_LEFT_RAISE;
    Frame f = torso.side(side);
    M3 upperTurn = turnX(-v[at + 1]) * turnY(-v[at]), foreTurn = upperTurn * turnX(-v[at + 2]);
    V3 S = shoulder, E = S + upperTurn * V3{0, 0, -0.186 * arms}, W = E + foreTurn * V3{0, 0, -0.146 * arms};
    double rS = mix(0.030, 0.026) * bulk * girth, rE = mix(0.024, 0.020) * bulk, rW = mix(0.0165, 0.0145) * bulk;
    int deltoid = mk.ellipsoid(f.at(S + upperTurn * V3{0.006, 0, -0.024}), V3{mix(0.030, 0.025), mix(0.034, 0.029), mix(0.046, 0.040)} * ((0.75 + 0.25 * muscle) * bulk),
                               f.turn(upperTurn));
    mk.join(yoke[side], deltoid, f.at(S), 0.03, 0.07);
    int upper = mk.cone(f.at(S), rS, f.at(E), rE);
    mk.join(deltoid, upper, f.at(S + upperTurn * V3{0, 0, -0.04}), 0.02, 0.06);
    V3 b = S + upperTurn * V3{0, -0.010, -0.09 * arms};
    int biceps = mk.ellipsoid(f.at(b), V3{0.021, 0.022, 0.05 * arms} * (mix(1, 0.7) * muscle * bulk), f.turn(upperTurn));
    mk.join(upper, biceps, f.at(b), 0.015, 0.06);
    int fore = mk.cone(f.at(E), rE, f.at(W), rW);
    // The hand (made finer), bent at the wrist toward the palm: the palm (facing the body as the arm hangs; the thumb in
    // front) and the pad of the thumb; four fingers of three bones each, apart by the spread, each joint bent by the
    // curl; the thumb's three bones the same, turned to face the fingers.
    size_t handFrom = mk.body.parts.size();
    int hp = side ? BK_FIG_RIGHT_WRIST : BK_FIG_LEFT_WRIST;
    double curl = v[hp + 1] / 100, spread = v[hp + 2] / 100;
    M3 handTurn = foreTurn * turnY(v[hp]);
    auto inHand = [&](V3 p) { return f.at(W + handTurn * (p * small)); };
    int palm = mk.slab(inHand({-0.001, -0.002, -0.031}), V3{0.0015, 0.0165, 0.0205} * small, 0.0085 * small, f.turn(handTurn));
    mk.join(fore, palm, f.at(W), 0.006, 0.03);
    int pad = mk.ellipsoid(inHand({-0.006, -0.014, -0.022}), V3{0.0075, 0.0095, 0.016} * small, f.turn(handTurn));
    mk.join(palm, pad, inHand({-0.004, -0.012, -0.022}), 0.004 * small, 0.025 * small);
    int heel = mk.ellipsoid(inHand({-0.005, 0.014, -0.028}), V3{0.006, 0.008, 0.018} * small, f.turn(handTurn));
    mk.join(palm, heel, inHand({-0.004, 0.012, -0.028}), 0.004 * small, 0.025 * small);
    int dome = mk.ellipsoid(inHand({0.003, -0.002, -0.033}), V3{0.0075, 0.021, 0.027} * small, f.turn(handTurn));
    mk.join(palm, dome, inHand({0.003, -0.002, -0.033}), 0.006 * small, 0.04 * small);
    // Index, middle, ring, little: knuckle across the palm, its bones' lengths and radii.
    const double across[4] = {-0.0160, -0.0053, 0.0053, 0.0155}, down[4] = {-0.0575, -0.0595, -0.0580, -0.0545};
    const double scale[4] = {0.92, 1, 0.95, 0.75}, thick[4] = {1, 1.04, 0.97, 0.85}, fan[4] = {-1, -0.3, 0.3, 1};
    for (int finger = 0; finger < 4; finger++) {
      V3 J{0, across[finger], down[finger]};
      M3 turnSoFar = turnX(fan[finger] * 20 * spread);
      const double length[3] = {0.025, 0.0160, 0.0120}, bend[3] = {85, 100, 70};
      double r0 = 0.0052 * thick[finger];
      int prev = palm;
      for (int bone = 0; bone < 3; bone++) {
        turnSoFar = turnSoFar * turnY(bend[bone] * curl);
        V3 next = J + turnSoFar * V3{0, 0, -length[bone] * scale[finger]};
        double r1 = r0 * (bone == 2 ? 0.80 : 0.90);
        int part = mk.cone(inHand(J), r0 * small, inHand(next), r1 * small);
        if (bone == 0) mk.join(prev, part, inHand(J), 0.003 * small, 0.010 * small);
        prev = part, J = next, r0 = r1;
      }
    }
    {
      M3 thumbTurn = turnX(-24) * turnY(12) * turnZ(-80);
      V3 J{-0.004, -0.016, -0.014};
      const double length[3] = {0.021, 0.017, 0.014}, bend[3] = {15, 40, 50}, radius[4] = {0.0076, 0.0064, 0.0056, 0.0047};
      M3 turnSoFar = thumbTurn;
      for (int bone = 0; bone < 3; bone++) {
        turnSoFar = turnSoFar * turnY(bend[bone] * curl);
        V3 next = J + turnSoFar * V3{0, 0, -length[bone]};
        int part = mk.cone(inHand(J), radius[bone] * small, inHand(next), radius[bone + 1] * small);
        if (bone == 0) mk.join(pad, part, inHand(J + turnSoFar * V3{0, 0, -0.010}), 0.003 * small, 0.02 * small);
        J = next;
      }
    }
    mk.fine(handFrom);
  }

  // The legs, each from its hip joint: forward, then out; the shin bent back at the knee; the foot kept level unless the
  // shin leans further than 35° (then leaning the rest of the way).
  for (int side = 0; side < 2; side++) {
    int at = side ? BK_FIG_RIGHT_HIP : BK_FIG_LEFT_HIP;
    Frame f = root.side(side);
    M3 thighTurn = turnX(-v[at]) * turnY(-v[at + 1]), shinTurn = thighTurn * turnX(v[at + 2]);
    V3 P{mix(0.045, 0.050) * (0.5 + 0.5 * hips), 0, hipZ}, K = P + thighTurn * V3{0, 0, -thigh}, A = K + shinTurn * V3{0, 0, -shin};
    double rK = mix(0.036, 0.034) * bulk;
    int upper = mk.cone(f.at(P), mix(0.051, 0.055) * bulk * girth, f.at(K), rK);
    mk.join(pelvis, upper, f.at(P), 0.025, 0.08);
    mk.join(buttock[side], upper, f.at(P + V3{0, 0.03, -0.03}), 0.02, 0.06);
    int lower = mk.cone(f.at(K), rK, f.at(A), mix(0.022, 0.020) * bulk);
    // The kneecap in front of the knee.
    int knee = mk.ellipsoid(f.at(K + thighTurn * V3{0, -0.78 * rK, 0.004}), V3{0.016, 0.009, 0.019} * bulk, f.turn(thighTurn));
    mk.join(upper, knee, f.at(K + thighTurn * V3{0, -0.6 * rK, 0.004}), 0.008, 0.035);
    mk.join(lower, knee, f.at(K + thighTurn * V3{0, -0.6 * rK, -0.006}), 0.008, 0.035);
    V3 c = K + shinTurn * V3{0, 0.012, -0.075 * legs};
    int calf = mk.ellipsoid(f.at(c), V3{0.032, 0.030, 0.075 * legs} * (mix(1, 0.9) * (0.8 + 0.2 * muscle) * bulk), f.turn(shinTurn));
    mk.join(lower, calf, f.at(c), 0.02, 0.08);
    V3 d = shinTurn * V3{0, 0, -1};
    auto past = [](double a) { return a > 35 ? a - 35 : a < -35 ? a + 35 : 0.0; };
    M3 footTurn = turnX(past(trig::atan2(d.y, -d.z) / degree)) * turnY(past(-trig::atan2(d.x, -d.z) / degree)) * turnZ(8);
    V3 heel = A + footTurn * (V3{0, 0.016, -0.039} * small), toe = A + footTurn * (V3{0, -0.064, -0.039} * small);
    int foot = mk.foot(f.at(heel), 0.026 * small, f.at(toe), 0.022 * small, f.turn(footTurn));
    // The ball of the foot: wider than tall, flat on the sole.
    int ball = mk.slab(f.at(A + footTurn * V3{0.003 * small, -0.066 * small, -0.039 * small + 0.0105 * small}), V3{0.0125, 0.012, 0} * small, 0.0105 * small,
                       f.turn(footTurn));
    mk.join(foot, ball, f.at(A + footTurn * (V3{0, -0.06, -0.03} * small)), 0.010, 0.04);
    mk.join(lower, foot, f.at(A), 0.012, 0.04);
    V3 i = A + footTurn * (V3{0, -0.034, -0.020} * small);
    int instep = mk.ellipsoid(f.at(i), V3{0.024, 0.042, 0.0185} * small, f.turn(footTurn));
    mk.join(foot, instep, f.at(i), 0.01, 0.05);
    mk.join(lower, instep, f.at(A), 0.012, 0.04);
    // The ankle's bones each side, the inner higher.
    for (int k = 0; k < 2; k++) {
      int bone = mk.ellipsoid(f.at(A + footTurn * (V3{k ? 0.016 : -0.015, 0.002, k ? -0.002 : 0.004} * small)), V3{0.0075, 0.0080, 0.0085} * small,
                              f.turn(footTurn));
      mk.join(lower, bone, f.at(A + footTurn * (V3{k ? 0.012 : -0.011, 0.002, 0} * small)), 0.004, 0.02);
    }
    // The toes (made finer), resting on the sole: the big toe on the inside, the rest smaller outward.
    size_t toesFrom = mk.body.parts.size();
    const double tx[5] = {-0.0125, -0.0012, 0.0068, 0.0142, 0.0208}, ty[5] = {-0.078, -0.081, -0.079, -0.075, -0.070};
    const double tl[5] = {0.030, 0.023, 0.020, 0.017, 0.014}, tr[5] = {0.0088, 0.0054, 0.0051, 0.0048, 0.0045};
    int lastToe = -1;
    V3 lastBase{0, 0, 0};
    for (int k = 0; k < 5; k++) {
      // (Their radii as made, none thinner than the least: each ball's lowest point on the sole.)
      double r0 = std::max(tr[k] * small, mk.least / H), r1 = std::max(tr[k] * 0.85 * small, mk.least / H);
      V3 t0 = A + footTurn * V3{tx[k] * small, ty[k] * small, -0.039 * small + r0};
      V3 t1 = A + footTurn * V3{tx[k] * 1.08 * small, (ty[k] - tl[k]) * small, -0.039 * small + r1};
      int toePart = mk.cone(f.at(t0), r0, f.at(t1), r1);
      mk.join(ball, toePart, f.at(t0), 0.004 * small, 0.014 * small);
      // (Each toe joined to the one beside it at their roots.)
      if (lastToe >= 0) mk.join(lastToe, toePart, f.at((lastBase + t0) * 0.5), 0.003 * small, 0.010 * small);
      lastToe = toePart, lastBase = t0;
    }
    mk.fine(toesFrom);
  }

  // Centred on its box; the point between the hips on the ground kept.
  Body &B = mk.body;
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (const Part &P : B.parts)
    if (P.layer != 1) lo = vmin(lo, P.lo), hi = vmax(hi, P.hi);
  V3 mid = (lo + hi) * 0.5;
  for (Part &P : B.parts) P.a = P.a - mid, P.b = P.b - mid, P.lo = P.lo - mid, P.hi = P.hi - mid, P.cutAt -= dot(P.cut, mid);
  for (Joint &J : B.joints) J.at = J.at - mid;
  for (auto &z : B.fine) z.first = z.first - mid, z.second = z.second - mid;
  B.lo = lo - mid, B.hi = hi - mid;
  B.anchor = V3{0, 0, hipZ * H} - mid;
  B.anchor.z = B.lo.z;
  return B;
}

// The last few figures made.
struct Made {
  bool draft;
  FigureSpec spec;
  std::shared_ptr<const Model> model;
};
std::mutex keptLock;
std::list<Made> kept;

bool same(const FigureSpec &a, const FigureSpec &b) {
  for (int i = 0; i < FigureNumbers; i++)
    if (!(a.v[i] == b.v[i])) return false;
  return true;
}

}  // namespace

void figureDefaults(double sex, double *out) {
  sex = std::isfinite(sex) ? std::min(1.0, std::max(0.0, sex)) : 0;
  out[BK_FIG_SEX] = sex, out[BK_FIG_HEIGHT] = 100 - 6 * sex;
  for (int i = BK_FIG_BUILD; i <= BK_FIG_HEAD; i++) out[i] = 1;
  figurePose(BK_POSE_STAND, out);
  out[BK_FIG_HAIR] = BK_HAIR_NONE, out[BK_FIG_HAIR_VOLUME] = 1;
}

void figureRange(int field, double &lo, double &hi) {
  if (field < 0 || field >= FigureNumbers) lo = hi = 0;
  else lo = lowest[field], hi = highest[field];
}

void figurePose(int pose, double *v) {
  if (pose < 0 || pose >= BK_POSE_COUNT) return;
  for (int i = 0; i < posed; i++) v[BK_FIG_NOD + i] = poses[pose][i];
}

int figurePoseOf(const double *v) {
  for (int p = 0; p < BK_POSE_COUNT; p++) {
    bool all = true;
    for (int i = 0; i < posed && all; i++) all = std::fabs(v[BK_FIG_NOD + i] - poses[p][i]) <= 1e-9;
    if (all) return p;
  }
  return -1;
}

bool figureSpec(const double *p, int count, FigureSpec &out, std::string &why) {
  if (!p || count < 0) return why = "figure: no numbers", false;
  double sex = count > 0 ? p[0] : 0;
  if (!std::isfinite(sex)) return why = "figure: sizes must be numbers", false;
  figureDefaults(sex, out.v);
  for (int i = 0; i < std::min(count, (int)FigureNumbers); i++) out.v[i] = p[i];
  for (int i = 0; i < FigureNumbers; i++) {
    if (!std::isfinite(out.v[i])) return why = "figure: sizes must be numbers", false;
    if (out.v[i] < lowest[i] - 1e-9 || out.v[i] > highest[i] + 1e-9) return why = std::string("figure: the ") + names[i] + " is out of its range", false;
    out.v[i] = std::min(highest[i], std::max(lowest[i], out.v[i])) + 0.0;  // (and −0 as 0)
  }
  return true;
}

void figureBox(const FigureSpec &s, V3 &size, V3 &anchor) {
  Body B = build(s);
  size = B.hi - B.lo, anchor = B.anchor;
}

bool figure(const FigureSpec &s, bool draft, Shape &out, std::string &why) {
  {
    std::lock_guard<std::mutex> hold(keptLock);
    for (auto it = kept.begin(); it != kept.end(); ++it)
      if (it->draft == draft && same(it->spec, s)) {
        kept.splice(kept.begin(), kept, it);
        out = shapeOf(it->model);
        return true;
      }
  }
  Body B = build(s);
  double H = s.v[BK_FIG_HEIGHT], h = H / (draft ? 80 : 160);
  int n[3];
  for (int a = 0; a < 3; a++) n[a] = (int)std::ceil((B.hi[a] - B.lo[a]) / h) + 5;
  // (A grid point looks at the parts within a step and a half, and as far again as any blend adds: whatever is further
  // can't be nearest at either end of a step the surface crosses. A block looks further, up to a block's half diagonal
  // and a step more: the bins hold every part and joint within that of them, over the whole grid.)
  double reach = 1.5 * h + B.kmax / 4;
  binned(B, 5.5 * h + B.kmax / 4, 3.5 * h);
  Eval E(B.parts.size());
  auto value = [&](V3 p) { return field(B, p, reach, E); };
  // A block is all outside where its middle is further than its corners (by the most a blend can change the distance
  // across a step), all inside where it's that far in.
  auto block = [&](V3 lo, V3 hi) {
    V3 c = (lo + hi) * 0.5;
    double far = 1.25 * 0.5 * norm(hi - lo);
    double f = field(B, c, far + B.kmax / 4 + h, E);
    return f > far * 1.000001 ? 1 : f < -far * 1.000001 ? -1 : 0;
  };
  // The hands, the face and the toes (and a step round them) worked out finer each way: as fine as three steps across the
  // thinnest part's radius there, at most three times (a draft's, twice). (A small figure's thin parts, made thicker so
  // as to print, need less.)
  int k = std::max(1, std::min(draft ? 2 : 3, (int)std::ceil(3 * h / B.finest)));
  auto fine = [&](V3 lo, V3 hi) {
    for (const auto &z : B.fine)
      if (lo.x <= z.second.x + h && hi.x >= z.first.x - h && lo.y <= z.second.y + h && hi.y >= z.first.y - h && lo.z <= z.second.z + h &&
          hi.z >= z.first.z - h)
        return true;
    return false;
  };
  std::vector<V3> pts;
  std::vector<uint32_t> tris;
  // (A full figure's evened out only where that doesn't make it pass through itself; a draft is only to be seen, and
  // isn't looked at for that.)
  bool uncrossed = draft;
  if (!isoSurface(V3{0, 0, 0}, h, n, value, block, 2, pts, tris, why, 1500000, k, fine, draft ? nullptr : &uncrossed))
    return why = "figure: " + why, false;
  if (pts.empty()) return why = "figure: nothing inside", false;
  // Within its box, and its farthest points on it (so its box is the one worked out from its parts).
  for (V3 &p : pts) p = vmax(B.lo, vmin(B.hi, p));
  for (int a = 0; a < 3; a++) {
    size_t top = 0, bottom = 0;
    for (size_t i = 1; i < pts.size(); i++) {
      if (pts[i][a] > pts[top][a]) top = i;
      if (pts[i][a] < pts[bottom][a]) bottom = i;
    }
    if (B.hi[a] - pts[top][a] <= h) pts[top][a] = B.hi[a];
    if (pts[bottom][a] - B.lo[a] <= h) pts[bottom][a] = B.lo[a];
  }
  auto m = meshModel(pts, tris, why, uncrossed);
  if (!m) return why = "figure: " + why, false;
  {
    std::lock_guard<std::mutex> hold(keptLock);
    kept.push_front({draft, s, m});
    if (kept.size() > 8) kept.pop_back();
  }
  out = shapeOf(m);
  return true;
}

}  // namespace bce
