// Exact orientation: a quick answer in floating point when it can't be wrong, otherwise an exact one from expansions
// (a number kept as a sum of doubles that don't overlap, so no bit is ever rounded away).
#include "Engine/Math.hpp"

#include <cmath>

namespace bce {

namespace {

// a + b exactly, as x + y with x the rounded sum.
inline void twoSum(double a, double b, double &x, double &y) {
  x = a + b;
  double bv = x - a, av = x - bv;
  y = (a - av) + (b - bv);
}

// a − b exactly.
inline void twoDiff(double a, double b, double &x, double &y) {
  x = a - b;
  double bv = a - x, av = x + bv;
  y = (a - av) + (bv - b);
}

// a · b exactly (the fused multiply-add rounds once, so its remainder is exact).
inline void twoProduct(double a, double b, double &x, double &y) {
  x = a * b;
  y = std::fma(a, b, -x);
}

// A sum of doubles held exactly: increasing in magnitude, no two overlapping.
struct Expansion {
  double t[256];
  int n = 0;

  // Adds b exactly.
  void add(double b) {
    double q = b;
    int k = 0;
    for (int i = 0; i < n; i++) {
      double h;
      twoSum(q, t[i], q, h);
      if (h != 0) t[k++] = h;
    }
    if (q != 0) t[k++] = q;
    n = k;
  }

  // Adds a · b · c exactly.
  void addProduct(double a, double b, double c) {
    double h, l, x, y;
    twoProduct(a, b, h, l);
    twoProduct(h, c, x, y);
    add(x), add(y);
    twoProduct(l, c, x, y);
    add(x), add(y);
  }

  int sign() const { return n == 0 ? 0 : t[n - 1] > 0 ? 1 : -1; }
  // The value, rounded (the parts summed from the smallest, so all but the last rounding is exact).
  double value() const {
    double v = 0;
    for (int i = 0; i < n; i++) v += t[i];
    return v;
  }
};

}  // namespace

namespace {

// orient3d's determinant: its sign, and with `value` the value too.
int orient3dOf(V3 a, V3 b, V3 c, V3 d, double *value) {
  double adx = a.x - d.x, ady = a.y - d.y, adz = a.z - d.z;
  double bdx = b.x - d.x, bdy = b.y - d.y, bdz = b.z - d.z;
  double cdx = c.x - d.x, cdy = c.y - d.y, cdz = c.z - d.z;
  double bc = bdx * cdy - bdy * cdx, ca = cdx * ady - cdy * adx, ab = adx * bdy - ady * bdx;
  double det = adz * bc + bdz * ca + cdz * ab;
  double permanent = std::fabs(adz) * (std::fabs(bdx * cdy) + std::fabs(bdy * cdx)) + std::fabs(bdz) * (std::fabs(cdx * ady) + std::fabs(cdy * adx)) +
                     std::fabs(cdz) * (std::fabs(adx * bdy) + std::fabs(ady * bdx));
  // The rounding of the differences and the products is at most about 7 units in the last place of the permanent.
  double bound = 1e-15 * permanent;
  if (value) *value = det;
  // (Its value is wanted to more than its sign: the plain sum only when far enough from the rounding.)
  if ((det > bound || det < -bound) && (!value || std::fabs(det) > 1e-8 * permanent)) return det > 0 ? 1 : -1;
  if (permanent == 0) return 0;

  // Exactly: each difference as two doubles, the determinant as the sum of every product of their parts.
  double dx[3][2], dy[3][2], dz[3][2];
  const V3 p[3] = {a, b, c};
  for (int i = 0; i < 3; i++) {
    twoDiff(p[i].x, d.x, dx[i][0], dx[i][1]);
    twoDiff(p[i].y, d.y, dy[i][0], dy[i][1]);
    twoDiff(p[i].z, d.z, dz[i][0], dz[i][1]);
  }
  // det = Σ over the even permutations (i, j, k) of x_i y_j z_k − Σ over the odd ones.
  static const int perms[6][3] = {{0, 1, 2}, {1, 2, 0}, {2, 0, 1}, {0, 2, 1}, {1, 0, 2}, {2, 1, 0}};
  Expansion e;
  for (int q = 0; q < 6; q++) {
    double s = q < 3 ? 1 : -1;
    const int *pi = perms[q];
    for (int u = 0; u < 2; u++)
      for (int v = 0; v < 2; v++)
        for (int w = 0; w < 2; w++) e.addProduct(s * dx[pi[0]][u], dy[pi[1]][v], dz[pi[2]][w]);
  }
  if (value) *value = e.value();
  return e.sign();
}

int orient2dOf(double ax, double ay, double bx, double by, double cx, double cy, double *value) {
  double l = (ax - cx) * (by - cy), r = (ay - cy) * (bx - cx), det = l - r;
  double bound = 1e-15 * (std::fabs(l) + std::fabs(r));
  if (value) *value = det;
  if ((det > bound || det < -bound) && (!value || std::fabs(det) > 1e-8 * (std::fabs(l) + std::fabs(r)))) return det > 0 ? 1 : -1;
  if (l == 0 && r == 0) return 0;
  double x[2][2], y[2][2];
  twoDiff(ax, cx, x[0][0], x[0][1]);
  twoDiff(bx, cx, x[1][0], x[1][1]);
  twoDiff(ay, cy, y[0][0], y[0][1]);
  twoDiff(by, cy, y[1][0], y[1][1]);
  Expansion e;
  for (int u = 0; u < 2; u++)
    for (int v = 0; v < 2; v++) e.addProduct(x[0][u], y[1][v], 1), e.addProduct(-y[0][u], x[1][v], 1);
  if (value) *value = e.value();
  return e.sign();
}

}  // namespace

int orient3d(V3 a, V3 b, V3 c, V3 d) { return orient3dOf(a, b, c, d, nullptr); }
double orient3dValue(V3 a, V3 b, V3 c, V3 d) {
  double v = 0;
  orient3dOf(a, b, c, d, &v);
  return v;
}
int orient2d(double ax, double ay, double bx, double by, double cx, double cy) { return orient2dOf(ax, ay, bx, by, cx, cy, nullptr); }
double orient2dValue(double ax, double ay, double bx, double by, double cx, double cy) {
  double v = 0;
  orient2dOf(ax, ay, bx, by, cx, cy, &v);
  return v;
}

int planeSide(V3 v, V3 p, V3 n) {
  double dx = v.x - p.x, dy = v.y - p.y, dz = v.z - p.z;
  double s = dx * n.x + dy * n.y + dz * n.z;
  double bound = 1e-15 * (std::fabs(dx * n.x) + std::fabs(dy * n.y) + std::fabs(dz * n.z));
  if (s > bound) return 1;
  if (s < -bound) return -1;
  Expansion e;
  const double a[3] = {v.x, v.y, v.z}, b[3] = {p.x, p.y, p.z}, m[3] = {n.x, n.y, n.z};
  for (int i = 0; i < 3; i++) {
    double h, l;
    twoDiff(a[i], b[i], h, l);
    e.addProduct(h, m[i], 1), e.addProduct(l, m[i], 1);
  }
  return e.sign();
}

}  // namespace bce
