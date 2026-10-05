// Human figures: a skeleton posed by its numbers, smooth convex parts round its bones (ellipsoids, and round cones from a
// ball at one end to a ball at the other), each blended into the next only near the joint they share; the surface where
// the nearest part (or blend) is at no distance, found on a grid a 160th of the height apart. Everything is worked out
// with + − × ÷ and √ (and Trig's sines) in a fixed order, so every machine makes the same figure to the bit.
#include "Engine/Figure.hpp"

#include "BcadKernel.h"
#include "Engine/Sculpt.hpp"

#include <list>
#include <memory>
#include <mutex>
#include <vector>

namespace bce {

namespace {

const double degree = 3.14159265358979323846 / 180;

const double lowest[FigureNumbers] = {0, 10, 0.7, 0.5, 0.8, 0.5, 0.75, 0.8, 0.85, 0.85, 0.8, -40, -70, -30, -20, -40, -25,
                                      0, -50, 0, 0, -50, 0, -30, -10, 0, -30, -10, 0};
const double highest[FigureNumbers] = {1, 5000, 1.4, 1.6, 1.25, 1.6, 1.4, 1.3, 1.15, 1.15, 1.25, 50, 70, 30, 60, 40, 25,
                                       170, 170, 150, 170, 170, 150, 110, 50, 140, 110, 50, 140};
const char *names[FigureNumbers] = {"sex", "height", "build", "muscle", "shoulders", "chest", "waist", "hips", "arms", "legs",
                                    "head", "nod", "turn", "tilt", "bend", "twist", "lean", "left arm's raise",
                                    "left arm's forward", "left elbow", "right arm's raise", "right arm's forward", "right elbow",
                                    "left hip", "left leg's out", "left knee", "right hip", "right leg's out", "right knee"};

// Each pose's numbers from the nod on: the head's nod, turn, tilt; the torso's bend, twist, lean; the left arm's raise,
// forward, elbow; the right arm's; the left leg's hip, out, knee; the right leg's.
const int posed = FigureNumbers - BK_FIG_NOD;
const double poses[BK_POSE_COUNT][posed] = {
    {0, 0, 0, 0, 0, 0, 8, 0, 10, 8, 0, 10, 0, 0, 0, 0, 0, 0},           // standing
    {0, 0, 0, 0, 0, 0, 90, 0, 0, 90, 0, 0, 0, 4, 0, 0, 4, 0},           // T
    {0, 0, 0, 0, 4, 0, 8, -20, 15, 8, 25, 25, 20, 0, 5, -15, 0, 5},     // walking
    {0, 0, 0, 0, 0, 0, 8, 20, 62, 8, 20, 62, 90, 6, 90, 90, 6, 90},     // sitting
    {0, 0, 5, 0, 0, 0, 8, 0, 10, 35, 95, 100, 0, 0, 0, 0, 0, 0},        // waving
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
  enum Kind { Ellipsoid, Cone, Foot } kind = Ellipsoid;
  V3 a, b;          // an ellipsoid's middle (a); a cone's (or a foot's) balls' middles
  double ra = 0, rb = 0;  // their radii
  V3 r;             // an ellipsoid's semi-axes (along R's columns)
  M3 R;             // an ellipsoid's axes; a foot's up (its third column: its sole the plane through a and b across it)
  V3 lo, hi;        // its box
  double grow = 1;  // how much further than its distance it may say it is (an ellipsoid's longest axis over its shortest)
  double rmin = 0;
  V3 ba;
  double l2 = 0, rr = 0, a2 = 0, il2 = 0;
};

// Two parts blended together near a joint: by k at the joint, less further off, none from R away.
struct Joint {
  int a, b;
  V3 at;
  double k, R2;
};

struct Body {
  std::vector<Part> parts;
  std::vector<Joint> joints;
  V3 lo, hi, anchor;
  double kmax = 0;
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
      return (std::sqrt(x * x + y * y + z * z) - 1) * P.rmin;
    }
    case Part::Cone: return coneDistance(P, p);
    case Part::Foot: return std::max(coneDistance(P, p), -dot(P.R.column(2), p - P.a));
  }
  return 0;
}

// The two smoothly together (as much as k rounds them where they meet).
double smoothMin(double a, double b, double k) {
  if (!(k > 0)) return std::min(a, b);
  double h = std::min(1.0, std::max(0.0, 0.5 + 0.5 * (b - a) / k));
  return b + (a - b) * h - k * h * (1 - h);
}

// The body's distance at p, as far as `margin` (further: just that): only parts within margin of p looked at.
double field(const Body &B, V3 p, double margin) {
  double best = margin;
  double val[64];
  uint64_t got = 0;
  for (size_t i = 0; i < B.parts.size(); i++) {
    const Part &P = B.parts[i];
    double m = margin * P.grow;
    if (p.x < P.lo.x - m || p.x > P.hi.x + m || p.y < P.lo.y - m || p.y > P.hi.y + m || p.z < P.lo.z - m || p.z > P.hi.z + m) continue;
    val[i] = distance(P, p), got |= 1ull << i;
    best = std::min(best, val[i]);
  }
  for (const Joint &J : B.joints) {
    double d2 = norm2(p - J.at);
    if (!(d2 < J.R2)) continue;
    double w = 1 - d2 / J.R2;
    for (int i : {J.a, J.b})
      if (!(got >> i & 1)) val[i] = distance(B.parts[i], p), got |= 1ull << i;
    best = std::min(best, smoothMin(val[J.a], val[J.b], J.k * w * w));
  }
  return best;
}

// Parts made at a height (all sizes given as parts of it), none thinner than 0.4 mm across… a radius of 0.4 mm.
struct Maker {
  Body body;
  double H;
  const double least = 0.4;
  explicit Maker(double height) : H(height) {}

  int add(Part &P) {
    body.parts.push_back(P);
    return (int)body.parts.size() - 1;
  }
  int ellipsoid(V3 c, V3 r, const M3 &R) {
    Part P;
    P.kind = Part::Ellipsoid, P.a = c * H, P.R = R;
    P.r = {std::max(r.x * H, least), std::max(r.y * H, least), std::max(r.z * H, least)};
    P.rmin = std::min({P.r.x, P.r.y, P.r.z});
    P.grow = std::max({P.r.x, P.r.y, P.r.z}) / P.rmin;
    for (int k = 0; k < 3; k++) {
      double ex = R.m[3 * k] * P.r.x, ey = R.m[3 * k + 1] * P.r.y, ez = R.m[3 * k + 2] * P.r.z;
      double e = std::sqrt(ex * ex + ey * ey + ez * ez);
      P.lo[k] = P.a[k] - e, P.hi[k] = P.a[k] + e;
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
  // How far forward the ribs' front is at (x, z) (in the torso's own frame).
  auto front = [&](double x, double z) {
    double a = x / ribsSize.x, c = (z - ribsAt.z) / ribsSize.z;
    return ribsAt.y - ribsSize.y * std::sqrt(std::max(0.0, 1 - a * a - c * c));
  };
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
  }

  // The head, turned on the neck.
  V3 nape{0, 0.008, Z(0.845)};
  M3 nod = turnZ(v[BK_FIG_TURN]) * turnY(v[BK_FIG_TILT]) * turnX(v[BK_FIG_NOD]);
  auto onHead = [&](V3 p) { return torso.at(nape + nod * (p - nape)); };
  M3 headTurn = torso.R * nod;
  double crown = 0.062 * head;
  int skull = mk.ellipsoid(onHead({0, 0.006 * head, 1 - crown}), {0.045 * head * mix(1, 0.96), 0.053 * head, crown}, headTurn);
  int jaw = mk.ellipsoid(onHead({0, -0.004 * head, chin + 0.032 * head}), {0.034 * head * mix(1, 0.9), 0.039 * head, 0.032 * head}, headTurn);
  mk.join(skull, jaw, onHead({0, -0.004 * head, chin + 0.045 * head}), 0.035, 0.08);
  mk.join(neck, jaw, onHead({0, 0.004, chin + 0.012 * head}), 0.02, 0.05);
  mk.join(neck, skull, onHead({0, 0.01, chin + 0.03 * head}), 0.02, 0.06);

  // The arms, each from its shoulder: raised out to the side, then forward; the forearm bent forward at the elbow; a
  // mitten of a hand, its palm toward the body when hanging, the thumb in front.
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
    int hand = mk.ellipsoid(f.at(W + foreTurn * (V3{0, -0.003, -0.05} * small)), V3{0.012, 0.025, 0.05} * small, f.turn(foreTurn));
    mk.join(fore, hand, f.at(W), 0.006, 0.03);
    int thumb = mk.cone(f.at(W + foreTurn * (V3{0.002, -0.018, -0.018} * small)), 0.0085 * small, f.at(W + foreTurn * (V3{0.004, -0.030, -0.048} * small)),
                        0.007 * small);
    mk.join(hand, thumb, f.at(W + foreTurn * (V3{0.002, -0.018, -0.02} * small)), 0.006, 0.025);
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
    V3 c = K + shinTurn * V3{0, 0.012, -0.075 * legs};
    int calf = mk.ellipsoid(f.at(c), V3{0.032, 0.030, 0.075 * legs} * (mix(1, 0.9) * (0.8 + 0.2 * muscle) * bulk), f.turn(shinTurn));
    mk.join(lower, calf, f.at(c), 0.02, 0.08);
    V3 d = shinTurn * V3{0, 0, -1};
    auto past = [](double a) { return a > 35 ? a - 35 : a < -35 ? a + 35 : 0.0; };
    M3 footTurn = turnX(past(trig::atan2(d.y, -d.z) / degree)) * turnY(past(-trig::atan2(d.x, -d.z) / degree)) * turnZ(8);
    V3 heel = A + footTurn * (V3{0, 0.016, -0.039} * small), toe = A + footTurn * (V3{0, -0.092, -0.039} * small);
    int foot = mk.foot(f.at(heel), 0.029 * small, f.at(toe), 0.021 * small, f.turn(footTurn));
    mk.join(lower, foot, f.at(A), 0.012, 0.04);
    V3 i = A + footTurn * (V3{0, -0.034, -0.020} * small);
    int instep = mk.ellipsoid(f.at(i), V3{0.024, 0.042, 0.0185} * small, f.turn(footTurn));
    mk.join(foot, instep, f.at(i), 0.01, 0.05);
    mk.join(lower, instep, f.at(A), 0.012, 0.04);
  }

  // Centred on its box; the point between the hips on the ground kept.
  Body &B = mk.body;
  V3 lo{INFINITY, INFINITY, INFINITY}, hi{-INFINITY, -INFINITY, -INFINITY};
  for (const Part &P : B.parts) lo = vmin(lo, P.lo), hi = vmax(hi, P.hi);
  V3 mid = (lo + hi) * 0.5;
  for (Part &P : B.parts) P.a = P.a - mid, P.b = P.b - mid, P.lo = P.lo - mid, P.hi = P.hi - mid;
  for (Joint &J : B.joints) J.at = J.at - mid;
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
  // can't be nearest at either end of a step the surface crosses.)
  double reach = 1.5 * h + B.kmax / 4;
  auto value = [&](V3 p) { return field(B, p, reach); };
  // A block is all outside where its middle is further than its corners (by the most a blend can change the distance
  // across a step), all inside where it's that far in.
  auto block = [&](V3 lo, V3 hi) {
    V3 c = (lo + hi) * 0.5;
    double far = 1.25 * 0.5 * norm(hi - lo);
    double f = field(B, c, far + B.kmax / 4 + h);
    return f > far * 1.000001 ? 1 : f < -far * 1.000001 ? -1 : 0;
  };
  std::vector<V3> pts;
  std::vector<uint32_t> tris;
  if (!isoSurface(V3{0, 0, 0}, h, n, value, block, 2, pts, tris, why)) return why = "figure: " + why, false;
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
  // (A draft is only to be seen: not looked at for passing through itself, which a grid's surface hardly ever does.)
  auto m = meshModel(pts, tris, why, draft);
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
