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
};

}  // namespace

int orient3d(V3 a, V3 b, V3 c, V3 d) {
  double adx = a.x - d.x, ady = a.y - d.y, adz = a.z - d.z;
  double bdx = b.x - d.x, bdy = b.y - d.y, bdz = b.z - d.z;
  double cdx = c.x - d.x, cdy = c.y - d.y, cdz = c.z - d.z;
  double bc = bdx * cdy - bdy * cdx, ca = cdx * ady - cdy * adx, ab = adx * bdy - ady * bdx;
  double det = adz * bc + bdz * ca + cdz * ab;
  double permanent = std::fabs(adz) * (std::fabs(bdx * cdy) + std::fabs(bdy * cdx)) + std::fabs(bdz) * (std::fabs(cdx * ady) + std::fabs(cdy * adx)) +
                     std::fabs(cdz) * (std::fabs(adx * bdy) + std::fabs(ady * bdx));
  // The rounding of the differences and the products is at most about 7 units in the last place of the permanent.
  double bound = 1e-15 * permanent;
  if (det > bound) return 1;
  if (det < -bound) return -1;
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
  return e.sign();
}

}  // namespace bce
