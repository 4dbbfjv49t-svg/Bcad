// Bcad's geometry engine: vectors, placements and exact orientation. All sizes are millimetres.
#pragma once
#include <algorithm>
#include <cmath>

#include "Trig.hpp"

namespace bce {

struct V3 {
  double x = 0, y = 0, z = 0;
  V3() = default;
  constexpr V3(double x, double y, double z) : x(x), y(y), z(z) {}
  double operator[](int i) const { return i == 0 ? x : i == 1 ? y : z; }
  double &operator[](int i) { return i == 0 ? x : i == 1 ? y : z; }
};

inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator-(V3 a) { return {-a.x, -a.y, -a.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline V3 operator*(double s, V3 a) { return a * s; }
inline V3 operator/(V3 a, double s) { return a * (1 / s); }
inline V3 &operator+=(V3 &a, V3 b) { return a = a + b; }
inline bool operator==(V3 a, V3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double norm2(V3 a) { return dot(a, a); }
inline double norm(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 unit(V3 a) {
  double l = norm(a);
  return l > 1e-300 ? a / l : a;
}
inline V3 vmin(V3 a, V3 b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline V3 vmax(V3 a, V3 b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }

// An affine placement: row-major 3x4 (linear part | translation), as the C API passes it.
struct Affine {
  double m[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};

  static Affine from(const double *a) {
    Affine t;
    std::copy(a, a + 12, t.m);
    return t;
  }
  static Affine translation(V3 v) {
    Affine t;
    t.m[3] = v.x, t.m[7] = v.y, t.m[11] = v.z;
    return t;
  }
  V3 point(V3 p) const {
    return {m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3], m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
            m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]};
  }
  V3 vector(V3 v) const {
    return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[4] * v.x + m[5] * v.y + m[6] * v.z, m[8] * v.x + m[9] * v.y + m[10] * v.z};
  }
  V3 row(int i) const { return {m[4 * i], m[4 * i + 1], m[4 * i + 2]}; }
  V3 column(int j) const { return {m[j], m[4 + j], m[8 + j]}; }
  // This placement followed by b.
  Affine then(const Affine &b) const {
    Affine r;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        double s = b.m[4 * i] * m[j] + b.m[4 * i + 1] * m[4 + j] + b.m[4 * i + 2] * m[8 + j];
        r.m[4 * i + j] = s + (j == 3 ? b.m[4 * i + 3] : 0);
      }
    }
    return r;
  }
  double det() const { return dot(column(0), cross(column(1), column(2))); }
  // The inverse; the placement must not be flat.
  Affine inverse() const {
    V3 c0 = column(0), c1 = column(1), c2 = column(2);
    V3 r0 = cross(c1, c2), r1 = cross(c2, c0), r2 = cross(c0, c1);
    double d = dot(c0, r0);
    Affine r;
    V3 rows[3] = {r0 / d, r1 / d, r2 / d};
    V3 t{m[3], m[7], m[11]};
    for (int i = 0; i < 3; i++) r.m[4 * i] = rows[i].x, r.m[4 * i + 1] = rows[i].y, r.m[4 * i + 2] = rows[i].z, r.m[4 * i + 3] = -dot(rows[i], t);
    return r;
  }
  // A normal carried along (by the inverse transpose), made unit.
  V3 normal(V3 n) const {
    V3 c0 = column(0), c1 = column(1), c2 = column(2);
    V3 r0 = cross(c1, c2), r1 = cross(c2, c0), r2 = cross(c0, c1);
    return unit(r0 * n.x + r1 * n.y + r2 * n.z) * (det() < 0 ? -1.0 : 1.0);
  }
  // The largest factor any length grows by (the largest singular value).
  double stretch() const {
    // Power iteration on AᵀA from a start no column is orthogonal to.
    V3 v{0.57735, 0.57735, 0.57735};
    double l = 0;
    for (int k = 0; k < 40; k++) {
      V3 a = vector(v);
      V3 w{dot(column(0), a), dot(column(1), a), dot(column(2), a)};
      double n = norm(w);
      if (n < 1e-300) return 0;
      v = w / n;
      if (std::fabs(n - l) <= 1e-14 * n) break;
      l = n;
    }
    return norm(vector(v));
  }
  // Uniform scale with a turn (no stretch along one axis more than another): *s gets the scale.
  bool similarity(double *s = nullptr) const {
    V3 c0 = column(0), c1 = column(1), c2 = column(2);
    double l0 = norm(c0), l1 = norm(c1), l2 = norm(c2);
    double tol = 1e-9 * l0;
    bool ok = std::fabs(l0 - l1) <= tol && std::fabs(l0 - l2) <= tol && std::fabs(dot(c0, c1)) <= tol * l0 &&
              std::fabs(dot(c0, c2)) <= tol * l0 && std::fabs(dot(c1, c2)) <= tol * l0;
    if (s) *s = l0;
    return ok;
  }
};

// The sign of the volume of tetrahedron (a, b, c, d) — positive when d lies below the plane through a, b, c seen
// counter-clockwise from above — exactly, whatever the rounding of the inputs' differences.
int orient3d(V3 a, V3 b, V3 c, V3 d);
// The sign of the turn a → b → c in the plane (positive counter-clockwise), exactly.
int orient2d(double ax, double ay, double bx, double by, double cx, double cy);
// The values themselves (six times the tetrahedron's volume, twice the triangle's area), to within a rounding of the
// exact value (from the exact sum where the plain sum can't be trusted): their signs are always orient3d's and
// orient2d's. For where a line crosses a plane, or two lines cross, as near the exact place as a double holds.
double orient3dValue(V3 a, V3 b, V3 c, V3 d);
double orient2dValue(double ax, double ay, double bx, double by, double cx, double cy);
// Which side of the plane through p with normal n the point v lies on (+1 where n points, -1 the other, 0 on it), exactly.
int planeSide(V3 v, V3 p, V3 n);

}  // namespace bce
