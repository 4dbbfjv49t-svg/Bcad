// Sines, cosines and the rest worked out the same way on every machine. A platform's own (macOS's, Linux's) may differ
// in the last bit, which is enough to make a merge come out differently on one than on the other; these use only
// + − × ÷ and √ (exact by IEEE 754), in a fixed order, so every machine gets every bit alike. Kernels and constants
// after fdlibm (Sun Microsystems, 1993: "Permission to use, copy, modify, and distribute this software is freely
// granted, provided that this notice is preserved."), within an ulp or so of the exact value.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

namespace trig {

namespace detail {

inline uint32_t high(double x) {
  uint64_t b;
  std::memcpy(&b, &x, 8);
  return (uint32_t)(b >> 32);
}

// sin(x + y) for |x + y| ≤ π/4, y the tail of a reduced argument (when tail).
inline double kernelSin(double x, double y, bool tail) {
  const double S1 = -1.66666666666666324348e-01, S2 = 8.33333333332248946124e-03, S3 = -1.98412698298579493134e-04,
               S4 = 2.75573137070700676789e-06, S5 = -2.50507602534068634195e-08, S6 = 1.58969099521155010221e-10;
  double z = x * x, v = z * x, r = S2 + z * (S3 + z * (S4 + z * (S5 + z * S6)));
  if (!tail) return x + v * (S1 + z * r);
  return x - ((z * (0.5 * y - v * r) - y) - v * S1);
}

// cos(x + y) for |x + y| ≤ π/4.
inline double kernelCos(double x, double y) {
  const double C1 = 4.16666666666666019037e-02, C2 = -1.38888888888741095749e-03, C3 = 2.48015872894767294178e-05,
               C4 = -2.75573143513906633035e-07, C5 = 2.08757232129817482790e-09, C6 = -1.13596475577881948265e-11;
  double z = x * x, w = z * z, r = z * (C1 + z * (C2 + z * C3)) + w * w * (C4 + z * (C5 + z * C6));
  double hz = 0.5 * z;
  w = 1.0 - hz;
  return w + (((1.0 - w) - hz) + (z * r - x * y));
}

// x less a whole number n of quarter turns, as y0 + y1 (|y0 + y1| ≤ π/4 or a hair more); returns n. Exact to well past a
// double for |x| up to about a million; past that still the same on every machine, if less exact.
inline int reduce(double x, double &y0, double &y1) {
  const double invpio2 = 6.36619772367581382433e-01, pio2_1 = 1.57079632673412561417e+00, pio2_1t = 6.07710050650619224932e-11,
               pio2_2 = 6.07710050630396597660e-11, pio2_2t = 2.02226624879595063154e-21, pio2_3 = 2.02226624871116645580e-21,
               pio2_3t = 8.47842766036889956997e-32;
  double fn = std::nearbyint(x * invpio2);
  double r = x - fn * pio2_1, w = fn * pio2_1t;
  int j = (int)((high(x) & 0x7fffffff) >> 20);
  y0 = r - w;
  int i = j - (int)((high(y0) >> 20) & 0x7ff);
  if (i > 16) {
    double t = r;
    w = fn * pio2_2, r = t - w, w = fn * pio2_2t - ((t - r) - w), y0 = r - w;
    i = j - (int)((high(y0) >> 20) & 0x7ff);
    if (i > 49) {
      t = r;
      w = fn * pio2_3, r = t - w, w = fn * pio2_3t - ((t - r) - w), y0 = r - w;
    }
  }
  y1 = (r - y0) - w;
  return (int)(int64_t)fn;
}

}  // namespace detail

inline double sin(double x) {
  uint32_t ix = detail::high(x) & 0x7fffffff;
  if (ix <= 0x3fe921fb) return ix < 0x3e400000 ? x : detail::kernelSin(x, 0, false);
  if (ix >= 0x7ff00000) return x - x;
  double y0, y1;
  switch (detail::reduce(x, y0, y1) & 3) {
  case 0: return detail::kernelSin(y0, y1, true);
  case 1: return detail::kernelCos(y0, y1);
  case 2: return -detail::kernelSin(y0, y1, true);
  default: return -detail::kernelCos(y0, y1);
  }
}

inline double cos(double x) {
  uint32_t ix = detail::high(x) & 0x7fffffff;
  if (ix <= 0x3fe921fb) return ix < 0x3e46a09e ? 1.0 : detail::kernelCos(x, 0);
  if (ix >= 0x7ff00000) return x - x;
  double y0, y1;
  switch (detail::reduce(x, y0, y1) & 3) {
  case 0: return detail::kernelCos(y0, y1);
  case 1: return -detail::kernelSin(y0, y1, true);
  case 2: return -detail::kernelCos(y0, y1);
  default: return detail::kernelSin(y0, y1, true);
  }
}

inline double tan(double x) {
  uint32_t ix = detail::high(x) & 0x7fffffff;
  if (ix <= 0x3fe921fb) return ix < 0x3e400000 ? x : detail::kernelSin(x, 0, false) / detail::kernelCos(x, 0);
  if (ix >= 0x7ff00000) return x - x;
  double y0, y1;
  int n = detail::reduce(x, y0, y1);
  double s = detail::kernelSin(y0, y1, true), c = detail::kernelCos(y0, y1);
  return (n & 1) ? -c / s : s / c;
}

inline double atan(double x) {
  const double atanhi[] = {4.63647609000806093515e-01, 7.85398163397448278999e-01, 9.82793723247329054082e-01, 1.57079632679489655800e+00};
  const double atanlo[] = {2.26987774529616870924e-17, 3.06161699786838301793e-17, 1.39033110312309984516e-17, 6.12323399573676603587e-17};
  const double aT[] = {3.33333333333329318027e-01,  -1.99999999998764832476e-01, 1.42857142725034663711e-01, -1.11111104054623557880e-01,
                       9.09088713343650656196e-02,  -7.69187620504482999495e-02, 6.66107313738753120669e-02, -5.83357013379057348645e-02,
                       4.97687799461593236017e-02,  -3.65315727442169155270e-02, 1.62858201153657823623e-02};
  uint32_t hx = detail::high(x), ix = hx & 0x7fffffff;
  bool negative = hx >> 31;
  if (ix >= 0x44100000) {
    if (x != x) return x + x;
    return negative ? -atanhi[3] - atanlo[3] : atanhi[3] + atanlo[3];
  }
  int id;
  if (ix < 0x3fdc0000) {
    if (ix < 0x3e400000) return x;
    id = -1;
  } else {
    x = std::fabs(x);
    if (ix < 0x3ff30000) {
      if (ix < 0x3fe60000) id = 0, x = (2.0 * x - 1.0) / (2.0 + x);
      else id = 1, x = (x - 1.0) / (x + 1.0);
    } else {
      if (ix < 0x40038000) id = 2, x = (x - 1.5) / (1.0 + 1.5 * x);
      else id = 3, x = -1.0 / x;
    }
  }
  double z = x * x, w = z * z;
  double s1 = z * (aT[0] + w * (aT[2] + w * (aT[4] + w * (aT[6] + w * (aT[8] + w * aT[10])))));
  double s2 = w * (aT[1] + w * (aT[3] + w * (aT[5] + w * (aT[7] + w * aT[9]))));
  if (id < 0) return x - x * (s1 + s2);
  z = atanhi[id] - ((x * (s1 + s2) - atanlo[id]) - x);
  return negative ? -z : z;
}

inline double atan2(double y, double x) {
  const double pi_o_2 = 1.5707963267948965580E+00, pi = 3.1415926535897931160E+00, pi_lo = 1.2246467991473531772E-16;
  if (x != x || y != y) return x + y;
  if (x == 1.0) return atan(y);
  bool yNeg = std::signbit(y), xNeg = std::signbit(x);
  int m = (yNeg ? 1 : 0) | (xNeg ? 2 : 0);
  if (y == 0) return m == 0 || m == 1 ? y : m == 2 ? pi : -pi;
  if (x == 0) return yNeg ? -pi_o_2 : pi_o_2;
  if (std::isinf(x)) {
    if (std::isinf(y)) {
      double q = pi_o_2 / 2;
      return m == 0 ? q : m == 1 ? -q : m == 2 ? 3 * q : -3 * q;
    }
    return m == 0 ? 0.0 : m == 1 ? -0.0 : m == 2 ? pi : -pi;
  }
  if (std::isinf(y)) return yNeg ? -pi_o_2 : pi_o_2;
  int k = (int)((detail::high(y) & 0x7fffffff) >> 20) - (int)((detail::high(x) & 0x7fffffff) >> 20);
  double z;
  if (k > 60) z = pi_o_2 + 0.5 * pi_lo, m &= 1;
  else if (xNeg && k < -60) z = 0.0;
  else z = atan(std::fabs(y / x));
  switch (m) {
  case 0: return z;
  case 1: return -z;
  case 2: return pi - (z - pi_lo);
  default: return (z - pi_lo) - pi;
  }
}

inline double asin(double x) { return atan2(x, std::sqrt((1.0 - x) * (1.0 + x))); }
inline double acos(double x) { return atan2(std::sqrt((1.0 - x) * (1.0 + x)), x); }

inline double hypot(double x, double y) { return std::sqrt(x * x + y * y); }

inline double cbrt(double x) {
  if (x == 0 || !std::isfinite(x)) return x;
  int e;
  double m = std::frexp(std::fabs(x), &e);  // [0.5, 1) · 2^e
  while (e % 3) m *= 2, e--;
  // m in [0.5, 4): a first guess, then Newton's steps to the last bit.
  double y = 0.6 + 0.4 * m;
  for (int i = 0; i < 6; i++) y = y - (y * y * y - m) / (3 * y * y);
  y = std::ldexp(y, e / 3);
  return x < 0 ? -y : y;
}

}  // namespace trig
