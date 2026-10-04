// Exact points for merging meshes (see Implicit.hpp).
#include "Engine/Implicit.hpp"

#include <cmath>

namespace bce {

thread_local long bigOverflows = 0;

// MARK: - whole numbers

Big::Big(int64_t v) {
  if (v == 0) return;
  s = v < 0 ? -1 : 1;
  m[0] = v < 0 ? 0 - (uint64_t)v : (uint64_t)v;
  n = 1;
}

Big::Big(__int128 v) {
  if (v == 0) return;
  s = v < 0 ? -1 : 1;
  unsigned __int128 u = v < 0 ? 0 - (unsigned __int128)v : (unsigned __int128)v;
  m[0] = (uint64_t)u, m[1] = (uint64_t)(u >> 64);
  n = m[1] ? 2 : 1;
}

double Big::approx() const {
  if (n == 0) return 0;
  // The top two places hold its first 64 bits at least: what's below shifts it by under 2^-64 of itself.
  double v = std::ldexp((double)m[n - 1], 64 * (n - 1));
  if (n >= 2) v += std::ldexp((double)m[n - 2], 64 * (n - 2));
  return s < 0 ? -v : v;
}

namespace {

int compareSize(const Big &a, const Big &b) {
  if (a.n != b.n) return a.n < b.n ? -1 : 1;
  for (int i = a.n - 1; i >= 0; i--)
    if (a.m[i] != b.m[i]) return a.m[i] < b.m[i] ? -1 : 1;
  return 0;
}

// |a| + |b|.
void addSizes(const Big &a, const Big &b, Big &r) {
  const Big &x = a.n >= b.n ? a : b, &y = a.n >= b.n ? b : a;
  uint64_t carry = 0;
  int i = 0;
  for (; i < y.n; i++) {
    unsigned __int128 t = (unsigned __int128)x.m[i] + y.m[i] + carry;
    r.m[i] = (uint64_t)t, carry = (uint64_t)(t >> 64);
  }
  for (; i < x.n; i++) {
    unsigned __int128 t = (unsigned __int128)x.m[i] + carry;
    r.m[i] = (uint64_t)t, carry = (uint64_t)(t >> 64);
  }
  r.n = x.n;
  if (carry) {
    if (r.n < Big::N) r.m[r.n++] = carry;
    else bigOverflows++;
  }
}

// |a| − |b|, where |a| ≥ |b|.
void subtractSizes(const Big &a, const Big &b, Big &r) {
  uint64_t borrow = 0;
  for (int i = 0; i < a.n; i++) {
    uint64_t bi = i < b.n ? b.m[i] : 0;
    unsigned __int128 t = (unsigned __int128)a.m[i] - bi - borrow;
    r.m[i] = (uint64_t)t;
    borrow = (uint64_t)(t >> 64) ? 1 : 0;
  }
  r.n = a.n;
  while (r.n > 0 && r.m[r.n - 1] == 0) r.n--;
}

}  // namespace

Big operator+(const Big &a, const Big &b) {
  if (a.s == 0) return b;
  if (b.s == 0) return a;
  Big r;
  if (a.s == b.s) {
    addSizes(a, b, r);
    r.s = a.s;
    return r;
  }
  int c = compareSize(a, b);
  if (c == 0) return Big();
  if (c > 0) subtractSizes(a, b, r), r.s = a.s;
  else subtractSizes(b, a, r), r.s = b.s;
  return r;
}

Big operator-(const Big &a, const Big &b) { return a + (-b); }

Big operator*(const Big &a, const Big &b) {
  if (a.s == 0 || b.s == 0) return Big();
  Big r;
  int n = a.n + b.n;
  if (n > Big::N) bigOverflows++, n = Big::N;
  for (int i = 0; i < n; i++) r.m[i] = 0;
  for (int i = 0; i < a.n && i < n; i++) {
    uint64_t carry = 0;
    int j = 0;
    for (; j < b.n && i + j < n; j++) {
      unsigned __int128 t = (unsigned __int128)a.m[i] * b.m[j] + r.m[i + j] + carry;
      r.m[i + j] = (uint64_t)t, carry = (uint64_t)(t >> 64);
    }
    if (i + j < n) r.m[i + j] = carry;
  }
  r.n = n;
  while (r.n > 0 && r.m[r.n - 1] == 0) r.n--;
  r.s = r.n ? a.s * b.s : 0;
  return r;
}

// MARK: - points

uint32_t ExactPoints::add(const Def &d) {
  uint32_t id = (uint32_t)at.size();
  def.push_back(d);
  I.push_back({0, 0, 0});
  Hom h = make(id);
  double w = h.w.approx(), x = h.x[0].approx() / w * step, y = h.x[1].approx() / w * step, z = h.x[2].approx() / w * step;
  at.push_back({x, y, z});
  // Each of the four within three units in the last place, so their quotients within seven: 10⁻¹⁵ of the largest
  // coordinate holds every coordinate's.
  err.push_back(1e-15 * std::max({std::fabs(x), std::fabs(y), std::fabs(z)}));
  return id;
}

uint32_t ExactPoints::grid(V3 p) {
  uint32_t id = (uint32_t)at.size();
  at.push_back(p);
  def.push_back({Grid, {-1, -1, -1}, {0, 0, 0, 0, 0, 0, 0, 0, 0}});
  err.push_back(0);
  I.push_back({(int64_t)(p.x / step), (int64_t)(p.y / step), (int64_t)(p.z / step)});
  return id;
}

uint32_t ExactPoints::line(uint32_t p, uint32_t q, const Plane3 &pl) {
  return add({Line, {(int8_t)pl.lift, -1, -1}, {p, q, pl.a, pl.b, pl.c, 0, 0, 0, 0}});
}

uint32_t ExactPoints::planes(const Plane3 &a, const Plane3 &b, const Plane3 &c) {
  return add({Planes, {(int8_t)a.lift, (int8_t)b.lift, (int8_t)c.lift}, {a.a, a.b, a.c, b.a, b.b, b.c, c.a, c.b, c.c}});
}

uint32_t ExactPoints::middle(uint32_t a, uint32_t b, uint32_t c) { return add({Middle, {-1, -1, -1}, {a, b, c, 0, 0, 0, 0, 0, 0}}); }

std::array<int64_t, 3> ExactPoints::coords(uint32_t i, int lift) const {
  std::array<int64_t, 3> c = I[i];
  if (lift >= 0) c[lift] += 1;
  return c;
}

// The normal (b − a) × (c − a) of a plane through grid points, exactly (each part under 2^123), and a's place.
void ExactPoints::normal(uint32_t a, uint32_t b, uint32_t c, int lift, __int128 n[3], std::array<int64_t, 3> &a0) const {
  a0 = I[a];
  std::array<int64_t, 3> pb = I[b], pc = lift >= 0 ? coords(a, lift) : I[c];
  __int128 u[3], v[3];
  for (int k = 0; k < 3; k++) u[k] = (__int128)pb[k] - a0[k], v[k] = (__int128)pc[k] - a0[k];
  n[0] = u[1] * v[2] - u[2] * v[1], n[1] = u[2] * v[0] - u[0] * v[2], n[2] = u[0] * v[1] - u[1] * v[0];
}

ExactPoints::Hom ExactPoints::hom(uint32_t i) const {
  if (def[i].kind == Grid) return make(i);
  if (held.size() <= i) held.resize(i + 1, -1);
  if (held[i] < 0) {
    Hom h = make(i);
    held[i] = (int)homs.size();
    homs.push_back(h);
  }
  return homs[held[i]];
}

ExactPoints::Hom ExactPoints::make(uint32_t i) const {
  const Def &d = def[i];
  Hom h;
  if (d.kind == Grid) {
    for (int k = 0; k < 3; k++) h.x[k] = Big((int64_t)I[i][k]);
    h.w = Big((int64_t)1);
    return h;
  }
  if (d.kind == Line) {
    // p + (q − p)·dp / (dp − dq), dp and dq the ends' heights over the plane (n · (end − r)).
    __int128 n[3];
    std::array<int64_t, 3> r;
    normal(d.v[2], d.v[3], d.v[4], d.lift[0], n, r);
    const std::array<int64_t, 3> &p = I[d.v[0]], &q = I[d.v[1]];
    Big dp, dq;
    for (int k = 0; k < 3; k++) {
      Big nk(n[k]);
      dp = dp + nk * Big((int64_t)(p[k] - r[k]));
      dq = dq + nk * Big((int64_t)(q[k] - r[k]));
    }
    h.w = dp - dq;
    for (int k = 0; k < 3; k++) h.x[k] = Big((int64_t)p[k]) * h.w + Big((int64_t)(q[k] - p[k])) * dp;
    return h;
  }
  if (d.kind == Middle) {
    // (a + b + c) / 3: over the product of their w's.
    Hom a = hom(d.v[0]), b = hom(d.v[1]), c = hom(d.v[2]);
    Big bc = b.w * c.w, ac = a.w * c.w, ab = a.w * b.w;
    for (int k = 0; k < 3; k++) h.x[k] = a.x[k] * bc + b.x[k] * ac + c.x[k] * ab;
    h.w = Big((int64_t)3) * a.w * bc;
    return h;
  }
  // Three planes n_i · x = d_i: x = (d1 (n2 × n3) + d2 (n3 × n1) + d3 (n1 × n2)) / (n1 · (n2 × n3)).
  Big n[3][3], dd[3];
  for (int j = 0; j < 3; j++) {
    __int128 nn[3];
    std::array<int64_t, 3> r;
    normal(d.v[3 * j], d.v[3 * j + 1], d.v[3 * j + 2], d.lift[j], nn, r);
    for (int k = 0; k < 3; k++) n[j][k] = Big(nn[k]);
    dd[j] = n[j][0] * Big((int64_t)r[0]) + n[j][1] * Big((int64_t)r[1]) + n[j][2] * Big((int64_t)r[2]);
  }
  auto crossOf = [&](int a, int b, Big out[3]) {
    out[0] = n[a][1] * n[b][2] - n[a][2] * n[b][1];
    out[1] = n[a][2] * n[b][0] - n[a][0] * n[b][2];
    out[2] = n[a][0] * n[b][1] - n[a][1] * n[b][0];
  };
  Big c23[3], c31[3], c12[3];
  crossOf(1, 2, c23), crossOf(2, 0, c31), crossOf(0, 1, c12);
  h.w = n[0][0] * c23[0] + n[0][1] * c23[1] + n[0][2] * c23[2];
  for (int k = 0; k < 3; k++) h.x[k] = dd[0] * c23[k] + dd[1] * c31[k] + dd[2] * c12[k];
  return h;
}

// MARK: - questions

int ExactPoints::orient3d(uint32_t a, uint32_t b, uint32_t c, uint32_t d) const {
  V3 A = at[a], B = at[b], C = at[c], D = at[d];
  if (err[d] == 0) return bce::orient3d(A, B, C, D);
  // In doubles, with d where it's held: as orient3d's own quick answer, and d's distance from where it is moved the
  // determinant by no more than that times the plane's normal's size.
  double adx = A.x - D.x, ady = A.y - D.y, adz = A.z - D.z, bdx = B.x - D.x, bdy = B.y - D.y, bdz = B.z - D.z, cdx = C.x - D.x, cdy = C.y - D.y,
         cdz = C.z - D.z;
  double det = adz * (bdx * cdy - bdy * cdx) + bdz * (cdx * ady - cdy * adx) + cdz * (adx * bdy - ady * bdx);
  double permanent = std::fabs(adz) * (std::fabs(bdx * cdy) + std::fabs(bdy * cdx)) + std::fabs(bdz) * (std::fabs(cdx * ady) + std::fabs(cdy * adx)) +
                     std::fabs(cdz) * (std::fabs(adx * bdy) + std::fabs(ady * bdx));
  double ux = B.x - A.x, uy = B.y - A.y, uz = B.z - A.z, vx = C.x - A.x, vy = C.y - A.y, vz = C.z - A.z;
  double size = std::fabs(uy * vz) + std::fabs(uz * vy) + std::fabs(uz * vx) + std::fabs(ux * vz) + std::fabs(ux * vy) + std::fabs(uy * vx);
  double bound = 8e-16 * permanent + 1.0001 * err[d] * size;
  if (det > bound) return 1;
  if (det < -bound) return -1;
  // Exactly: n · (a w − x), n = (b − a) × (c − a), the sign turned with w's.
  __int128 n[3];
  std::array<int64_t, 3> a0;
  normal(a, b, c, -1, n, a0);
  Hom h = hom(d);
  Big sum;
  for (int k = 0; k < 3; k++) sum = sum + Big(n[k]) * (Big((int64_t)a0[k]) * h.w - h.x[k]);
  return sum.sign() * h.w.sign();
}

int ExactPoints::orient2d(int axis, bool swap, uint32_t a, uint32_t b, uint32_t c) const {
  int iu = (axis + 1) % 3, iv = (axis + 2) % 3;
  if (swap) std::swap(iu, iv);
  V3 A = at[a], B = at[b], C = at[c];
  double ua = A[iu], va = A[iv], ub = B[iu], vb = B[iv], uc = C[iu], vc = C[iv];
  if (err[a] == 0 && err[b] == 0 && err[c] == 0) return bce::orient2d(ua, va, ub, vb, uc, vc);
  double acx = ua - uc, bcx = ub - uc, acy = va - vc, bcy = vb - vc;
  double l = acx * bcy, r = acy * bcx, det = l - r;
  double eac = err[a] + err[c], ebc = err[b] + err[c];
  double bound = 3.4e-16 * (std::fabs(l) + std::fabs(r)) +
                 1.0001 * (eac * (std::fabs(bcx) + std::fabs(bcy)) + ebc * (std::fabs(acx) + std::fabs(acy)) + 2 * eac * ebc);
  if (det > bound) return 1;
  if (det < -bound) return -1;
  // Exactly: the determinant of the three points' (u, v, w), its sign turned with each w's.
  Hom ha = hom(a), hb = hom(b), hc = hom(c);
  const Big &Ua = ha.x[iu], &Va = ha.x[iv], &Wa = ha.w, &Ub = hb.x[iu], &Vb = hb.x[iv], &Wb = hb.w, &Uc = hc.x[iu], &Vc = hc.x[iv], &Wc = hc.w;
  Big d = Ua * (Vb * Wc - Vc * Wb) - Va * (Ub * Wc - Uc * Wb) + Wa * (Ub * Vc - Uc * Vb);
  return d.sign() * Wa.sign() * Wb.sign() * Wc.sign();
}

int ExactPoints::compare(int k, uint32_t a, uint32_t b) const {
  double xa = at[a][k], xb = at[b][k];
  if (err[a] == 0 && err[b] == 0) return xa < xb ? -1 : xa > xb ? 1 : 0;
  double d = xa - xb, e = 1.0000001 * (err[a] + err[b]);
  if (d > e) return 1;
  if (d < -e) return -1;
  Hom ha = hom(a), hb = hom(b);
  return (ha.x[k] * hb.w - hb.x[k] * ha.w).sign() * ha.w.sign() * hb.w.sign();
}

int ExactPoints::lex(uint32_t a, uint32_t b) const {
  if (a == b) return 0;
  for (int k = 0; k < 3; k++)
    if (int c = compare(k, a, b)) return c;
  return 0;
}

int ExactPoints::facing(uint32_t a, uint32_t b, uint32_t c, const int64_t d[3]) const {
  __int128 n[3];
  std::array<int64_t, 3> a0;
  normal(a, b, c, -1, n, a0);
  // The normal's parts are exact; each made a double within a unit in its last place, the products and sum within a
  // few more.
  double sum = 0, size = 0;
  for (int k = 0; k < 3; k++) {
    double t = (double)n[k] * (double)d[k];
    sum += t, size += std::fabs(t);
  }
  if (sum > 1e-15 * size) return 1;
  if (sum < -1e-15 * size) return -1;
  return (Big(n[0]) * Big(d[0]) + Big(n[1]) * Big(d[1]) + Big(n[2]) * Big(d[2])).sign();
}

int ExactPoints::passes(uint32_t q, const int64_t d[3], uint32_t a, uint32_t b) const {
  V3 A = at[a], B = at[b], Q = at[q], D{(double)d[0], (double)d[1], (double)d[2]};
  // det(a − q, b − q, d) = (a × b + (b − a) × q) · d: in doubles first, q's distance from where it's held moving it by
  // no more than that times |d × (b − a)|.
  V3 u = A - Q, v = B - Q;
  double det = (u.y * v.z - u.z * v.y) * D.x + (u.z * v.x - u.x * v.z) * D.y + (u.x * v.y - u.y * v.x) * D.z;
  double permanent = (std::fabs(u.y * v.z) + std::fabs(u.z * v.y)) * std::fabs(D.x) + (std::fabs(u.z * v.x) + std::fabs(u.x * v.z)) * std::fabs(D.y) +
                     (std::fabs(u.x * v.y) + std::fabs(u.y * v.x)) * std::fabs(D.z);
  if (err[q] == 0 && permanent == 0) return 0;
  V3 e = B - A;
  double size = std::fabs(D.y * e.z) + std::fabs(D.z * e.y) + std::fabs(D.z * e.x) + std::fabs(D.x * e.z) + std::fabs(D.x * e.y) + std::fabs(D.y * e.x);
  double bound = 8e-16 * permanent + 1.0001 * err[q] * size;
  if (det > bound) return 1;
  if (det < -bound) return -1;
  // Exactly: ((a × b) · d) w + ((b − a) × x) · d, the sign turned with w's.
  Hom h = hom(q);
  const std::array<int64_t, 3> &pa = I[a], &pb = I[b];
  Big ab[3], ba[3];
  for (int k = 0; k < 3; k++) ba[k] = Big((int64_t)(pb[k] - pa[k]));
  Big ax(pa[0]), ay(pa[1]), az(pa[2]), bx(pb[0]), by(pb[1]), bz(pb[2]);
  ab[0] = ay * bz - az * by, ab[1] = az * bx - ax * bz, ab[2] = ax * by - ay * bx;
  Big dx(d[0]), dy(d[1]), dz(d[2]);
  Big first = (ab[0] * dx + ab[1] * dy + ab[2] * dz) * h.w;
  Big cx = ba[1] * h.x[2] - ba[2] * h.x[1], cy = ba[2] * h.x[0] - ba[0] * h.x[2], cz = ba[0] * h.x[1] - ba[1] * h.x[0];
  return (first + cx * dx + cy * dy + cz * dz).sign() * h.w.sign();
}

void ExactPoints::flat(uint32_t a, uint32_t b, uint32_t c, int &axis, bool &swap) const {
  __int128 n[3];
  std::array<int64_t, 3> a0;
  normal(a, b, c, -1, n, a0);
  auto mag = [](__int128 v) { return v < 0 ? -v : v; };
  axis = mag(n[0]) >= mag(n[1]) && mag(n[0]) >= mag(n[2]) ? 0 : mag(n[1]) >= mag(n[2]) ? 1 : 2;
  swap = n[axis] < 0;
}

}  // namespace bce
