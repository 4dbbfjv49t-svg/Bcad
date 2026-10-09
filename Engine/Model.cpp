// Bcad's geometry engine: the primitives, exactly (bounding boxes and volumes from formulas), and their meshes.
#include "Engine/Model.hpp"
#include "Engine/Sculpt.hpp"

#include "Engine/Print.hpp"
#include "Engine/Radial.hpp"
#include "Engine/Triangulate.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <utility>

namespace bce {

namespace {

constexpr double pi = M_PI;
// No chord turns by more than this, however coarse the mesh asked for.
constexpr double maxTurn = 0.35;

// The angle a chord may span on a circle of radius r with a sagitta of at most d.
double chordAngle(double r, double d) {
  if (r <= 0) return maxTurn;
  double c = 1 - d / r;
  double a = c <= -1 ? pi : 2 * trig::acos(c);
  return std::min(a, maxTurn);
}

// (At most a million: a radius so large its chords barely turn counted as such, never past what an int holds.)
int countFor(double span, double r, double d) {
  double n = std::ceil(std::fabs(span) / chordAngle(r, d) - 1e-9);
  return n >= 1e6 ? 1000000 : std::max(1, (int)(n > 0 ? n : 1));
}

}  // namespace

// MARK: - profile pieces

void Elem::at(double t, double &r, double &z) const {
  if (t <= 0) {
    r = r0, z = z0;
  } else if (t >= 1) {
    r = r1, z = z1;
  } else if (arc) {
    double a = a0 + (a1 - a0) * t;
    r = cr + rad * trig::cos(a), z = cz + rad * trig::sin(a);
  } else {
    r = r0 + (r1 - r0) * t, z = z0 + (z1 - z0) * t;
  }
  if (r < 0) r = 0;
}

void Elem::point(double t, double &x, double &y) const {
  if (t <= 0) {
    x = r0, y = z0;
  } else if (t >= 1) {
    x = r1, y = z1;
  } else if (arc) {
    double a = a0 + (a1 - a0) * t;
    x = cr + rad * trig::cos(a), y = cz + rad * trig::sin(a);
  } else {
    x = r0 + (r1 - r0) * t, y = z0 + (z1 - z0) * t;
  }
}

void Elem::normalAt(double t, double &nr, double &nz) const {
  double dr, dz;
  if (arc) {
    double a = a0 + (a1 - a0) * t, s = a1 > a0 ? 1 : -1;
    dr = -s * trig::sin(a), dz = s * trig::cos(a);
  } else {
    dr = r1 - r0, dz = z1 - z0;
  }
  double l = trig::hypot(dr, dz);
  nr = dz / l, nz = -dr / l;
}

double Elem::nearest(double r, double z) const {
  if (!arc) {
    double dr = r1 - r0, dz = z1 - z0, l2 = dr * dr + dz * dz;
    return l2 > 0 ? std::clamp(((r - r0) * dr + (z - z0) * dz) / l2, 0.0, 1.0) : 0;
  }
  double lo = std::min(a0, a1), hi = std::max(a0, a1);
  double ang = trig::atan2(z - cz, r - cr);
  while (ang < lo) ang += 2 * pi;
  while (ang >= lo + 2 * pi) ang -= 2 * pi;
  if (ang <= hi) return (ang - a0) / (a1 - a0);
  // Outside the arc: the nearer end.
  double d0 = trig::hypot(r - r0, z - z0), d1 = trig::hypot(r - r1, z - z1);
  return d0 <= d1 ? 0 : 1;
}

int Elem::pieces(double deflection) const { return arc ? countFor(a1 - a0, rad, deflection) : 1; }

// MARK: - meshes

void Solid::centroids() {
  std::vector<double> area(faces.size(), 0);
  std::vector<V3> sum(faces.size());
  for (size_t k = 0; k < triFace.size(); k++) {
    V3 a = p[tri[3 * k]], b = p[tri[3 * k + 1]], c = p[tri[3 * k + 2]];
    double w = norm(cross(b - a, c - a));
    area[triFace[k]] += w;
    sum[triFace[k]] += (a + b + c) * (w / 3);
  }
  for (size_t f = 0; f < faces.size(); f++)
    if (area[f] > 0) faces[f].centroid = sum[f] / area[f];
}

double Solid::meshVolume() const {
  double v = 0;
  for (size_t k = 0; k < tri.size(); k += 3) v += dot(p[tri[k]], cross(p[tri[k + 1]], p[tri[k + 2]]));
  return v / 6;
}

void Solid::slivers() {
  size_t nt = triFace.size();
  gap.assign(6 * nt, 0);
  std::vector<double> total(faces.size(), 0), area(faces.size(), 0);
  for (size_t t = 0; t < nt; t++) {
    uint32_t v[3] = {tri[3 * t], tri[3 * t + 1], tri[3 * t + 2]};
    for (int k = 0; k < 3; k++) {
      uint32_t i = v[k], j = v[(k + 1) % 3];
      gap[6 * t + 3 + k] = dot(n[j] - n[i], p[j] - p[i]) / 8;
    }
    total[triFace[t]] += sliver(t);
    area[triFace[t]] += norm(cross(p[v[1]] - p[v[0]], p[v[2]] - p[v[0]])) / 2;
  }
  // Scaled so each face's slivers add up to its deficit exactly; spread by area where the normals don't say (a face
  // flat in its mesh, though not in fact).
  for (size_t t = 0; t < nt; t++) {
    const Face &f = faces[triFace[t]];
    double sum = total[triFace[t]];
    for (int k = 3; k < 6; k++) {
      double &g = gap[6 * t + k];
      if (f.deficit == 0) g = 0;
      else if (sum * f.deficit > 0 && std::fabs(sum) > 1e-6 * std::fabs(f.deficit)) g *= f.deficit / sum;
      else g = area[triFace[t]] > 0 ? f.deficit / area[triFace[t]] : 0;
    }
  }
}

double Solid::sliver(size_t t) const {
  if (gap.empty()) return 0;
  V3 a = p[tri[3 * t]], b = p[tri[3 * t + 1]], c = p[tri[3 * t + 2]];
  return norm(cross(b - a, c - a)) / 6 * (gap[6 * t + 3] + gap[6 * t + 4] + gap[6 * t + 5]);
}

void Solid::transform(const Affine &a) {
  bool still = true;
  for (int i = 0; i < 12; i++) still = still && a.m[i] == Affine().m[i];
  if (!still) grid = 0, sound.clear();
  // A triangle's slivers grow with volume while its area grows its own way: the gap between grows by the difference.
  std::vector<double> before;
  if (!gap.empty()) {
    before.resize(triFace.size());
    for (size_t t = 0; t < triFace.size(); t++) before[t] = norm(cross(p[tri[3 * t + 1]] - p[tri[3 * t]], p[tri[3 * t + 2]] - p[tri[3 * t]]));
  }
  for (auto &q : p) q = a.point(q);
  for (auto &q : n) q = a.normal(q);
  for (auto &e : edges) {
    for (auto &q : e.pts) q = a.point(q);
    e.geom.place = e.geom.place.then(a);
  }
  for (auto &q : corners) q = a.point(q);
  double s, grow = std::fabs(a.det());
  bool similar = a.similarity(&s);
  if (similar) {
    for (auto &c : circles) c.centre = a.point(c.centre), c.axis = unit(a.vector(c.axis)), c.radius *= s;
  } else {
    // A circle stretched unevenly is a circle no longer, nor a turned surface turned.
    circles.clear();
    for (auto &e : edges) e.geom.exact = false;
  }
  if (a.det() < 0) {
    for (size_t k = 0; k < tri.size(); k += 3) std::swap(tri[k + 1], tri[k + 2]);
    // Corners 0, 2, 1: the midpoints 0–2, 2–1, 1–0.
    for (size_t k = 0; k < gap.size(); k += 6) std::swap(gap[k + 1], gap[k + 2]), std::swap(gap[k + 3], gap[k + 5]);
  }
  for (size_t t = 0; t < before.size(); t++) {
    double after = norm(cross(p[tri[3 * t + 1]] - p[tri[3 * t]], p[tri[3 * t + 2]] - p[tri[3 * t]]));
    double f = after > 0 ? grow * before[t] / after : 0;
    for (int k = 0; k < 6; k++) gap[6 * t + k] *= f;
  }
  for (auto &f : faces) {
    f.normal = a.normal(f.normal);
    f.deficit *= grow;
    f.geom.place = f.geom.place.then(a);
    if (!similar) f.geom.exact = false;
    if (f.geom.flat) {
      V3 on = a.point(f.geom.pn * f.geom.pd);
      f.geom.pn = a.normal(f.geom.pn);
      f.geom.pd = dot(f.geom.pn, on);
    }
  }
  centroids();
}

namespace {

double moment(const Elem &e);
void closeUp(std::vector<Elem> &loop);
// A sketch's regions stood up or turned (below, with the primitives).
void buildExtruded(const Model &m, Solid &out, double d);
void buildRevolved(const Model &m, Solid &out, double d);
double extrudedSupport(const Model &m, V3 d, V3 &where);
double revolvedSupport(const Model &m, V3 d, V3 &where);

// Two face numbers for an edge, without the same face twice.
void sides(Solid::Edge &e, int a, int b) {
  e.f0 = a >= 0 ? a : b;
  e.f1 = a >= 0 && b >= 0 && b != a ? b : -1;
}

// MARK: flat-sided

void buildPoly(const Model &m, Solid &out) {
  std::map<std::pair<int, int>, int> edgeOf;
  for (size_t f = 0; f < m.loops.size(); f++) {
    const auto &loop = m.loops[f];
    // Newell's normal, exact for a flat polygon.
    V3 nrm;
    for (size_t i = 0; i < loop.size(); i++) {
      V3 a = m.verts[loop[i]], b = m.verts[loop[(i + 1) % loop.size()]];
      nrm += V3{(a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)};
    }
    nrm = unit(nrm);
    Solid::Face face{nrm, {}, {}};
    face.geom.kind = FaceGeom::Flat, face.geom.flat = true, face.geom.pn = nrm, face.geom.pd = dot(nrm, m.verts[loop[0]]);
    out.faces.push_back(face);
    uint32_t base = (uint32_t)out.p.size();
    for (int v : loop) out.vertex(m.verts[v], nrm);
    for (size_t i = 1; i + 1 < loop.size(); i++) out.triangle(base, base + (uint32_t)i, base + (uint32_t)i + 1, (int)f);
    for (size_t i = 0; i < loop.size(); i++) {
      int a = loop[i], b = loop[(i + 1) % loop.size()];
      auto key = std::minmax(a, b);
      auto it = edgeOf.find(key);
      if (it == edgeOf.end()) {
        edgeOf[key] = (int)out.edges.size();
        Solid::Edge e;
        e.pts = {m.verts[a], m.verts[b]};
        e.f0 = (int)f;
        e.geom.kind = EdgeGeom::Line;
        out.edges.push_back(e);
      } else if (out.edges[it->second].f0 != (int)f) {
        out.edges[it->second].f1 = (int)f;
      }
    }
  }
  out.corners = m.verts;
}

// MARK: turned

// Where a profile piece's mesh rings go, as t along it: evenly within the chord error, and also (on a slope or an arc)
// at each of the heights asked for, so a cut across there at that height (a part on the same axis ending there) falls
// on a ring.
std::vector<double> stepsOf(const Elem &e, double d, const std::vector<double> &levels) {
  std::vector<double> cuts{0, 1};
  for (double z : levels) {
    if (!e.arc) {
      if (e.r0 != e.r1 && e.z0 != e.z1) cuts.push_back((z - e.z0) / (e.z1 - e.z0));
    } else if (std::fabs(z - e.cz) < e.rad) {
      double v = trig::asin((z - e.cz) / e.rad);
      for (double a : {v, pi - v})
        for (int k = -2; k <= 2; k++) cuts.push_back((a + 2 * pi * k - e.a0) / (e.a1 - e.a0));
    }
  }
  std::sort(cuts.begin(), cuts.end());
  std::vector<double> ts{0};
  for (double c : cuts)
    if (c > ts.back() + 1e-6 && c < 1 - 1e-6) ts.push_back(c);
  ts.push_back(1);
  std::vector<double> out;
  for (size_t i = 0; i + 1 < ts.size(); i++) {
    int n = e.arc ? countFor((ts[i + 1] - ts[i]) * (e.a1 - e.a0), e.rad, d) : 1;
    for (int j = 0; j < n; j++) out.push_back(j == 0 ? ts[i] : ts[i] + (ts[i + 1] - ts[i]) * j / n);
  }
  out.push_back(1);
  return out;
}

void buildTurned(const Model &m, Solid &out, double d) {
  // One count round the axis for every face, so the faces meet point for point.
  int count = m.columns(d);
  std::vector<double> cs(count + 1), sn(count + 1), cm(count), sm(count);
  for (int j = 0; j <= count; j++) {
    double a = 2 * pi * j / count;
    cs[j] = j == 0 || j == count ? 1 : trig::cos(a), sn[j] = j == 0 || j == count ? 0 : trig::sin(a);
  }
  for (int j = 0; j < count; j++) cm[j] = trig::cos(2 * pi * (j + 0.5) / count), sm[j] = trig::sin(2 * pi * (j + 0.5) / count);

  const size_t ne = m.profile.size();
  std::vector<int> faceOf(ne, -1);
  for (size_t k = 0; k < ne; k++) {
    const Elem &e = m.profile[k];
    if (e.onAxis()) continue;
    int f = (int)out.faces.size();
    faceOf[k] = f;
    double nr, nz;
    e.normalAt(0.5, nr, nz);
    Solid::Face face{{-nr, 0, nz}, {}, {}};
    face.geom.kind = FaceGeom::Turned, face.geom.elem = e;
    if (e.flat()) face.geom.flat = true, face.geom.pn = {0, 0, nz > 0 ? 1.0 : -1.0}, face.geom.pd = nz > 0 ? e.z0 : -e.z0;
    std::vector<double> ts = stepsOf(e, d, m.levels);
    int pieces = (int)ts.size() - 1;
    // What the mesh misses: the exact volume this piece turns round, less the polygon-sided one its chords turn round.
    double meshed = 0;
    for (int i = 0; i < pieces; i++) {
      double r0, z0, r1, z1;
      e.at(ts[i], r0, z0);
      e.at(ts[i + 1], r1, z1);
      meshed += moment(Elem::line(r0, z0, r1, z1));
    }
    face.deficit = 2 * pi * (moment(e) - meshed * count * trig::sin(2 * pi / count) / (2 * pi));
    out.faces.push_back(face);

    // Each ring's first vertex (a pole has one, or one per column where the surface comes to a point at an angle).
    std::vector<uint32_t> ring(pieces + 1);
    std::vector<char> pole(pieces + 1), fan(pieces + 1);
    for (int i = 0; i <= pieces; i++) {
      double t = ts[i], r, z;
      e.at(t, r, z);
      e.normalAt(t, nr, nz);
      ring[i] = (uint32_t)out.p.size();
      if (r == 0) {
        pole[i] = 1;
        if (std::fabs(nr) < 1e-12) {
          out.vertex({0, 0, z}, {0, 0, nz > 0 ? 1.0 : -1.0});
        } else {
          fan[i] = 1;
          for (int j = 0; j < count; j++) out.vertex({0, 0, z}, {nr * cm[j], nr * sm[j], nz});
        }
      } else {
        for (int j = 0; j < count; j++) out.vertex({r * cs[j], r * sn[j], z}, {nr * cs[j], nr * sn[j], nz});
      }
    }
    for (int i = 0; i < pieces; i++) {
      for (int j = 0; j < count; j++) {
        int jn = (j + 1) % count;
        if (pole[i] && pole[i + 1]) continue;
        uint32_t A = ring[i] + (pole[i] ? (fan[i] ? j : 0) : j), B = ring[i] + (pole[i] ? (fan[i] ? j : 0) : jn);
        uint32_t C = ring[i + 1] + (pole[i + 1] ? (fan[i + 1] ? j : 0) : jn), D = ring[i + 1] + (pole[i + 1] ? (fan[i + 1] ? j : 0) : j);
        if (pole[i]) {
          out.triangle(A, C, D, f);
        } else if (pole[i + 1]) {
          out.triangle(A, B, C, f);
        } else {
          out.triangle(A, B, C, f);
          out.triangle(A, C, D, f);
        }
      }
    }
    // The seam, where the turned face meets itself (none on a flat face).
    if (!e.flat()) {
      Solid::Edge seam;
      for (int i = 0; i <= pieces; i++) {
        double r, z;
        e.at(ts[i], r, z);
        seam.pts.push_back({r, 0, z});
      }
      seam.f0 = f;
      seam.geom.kind = EdgeGeom::Profile, seam.geom.elem = e;
      out.edges.push_back(seam);
      if (e.arc && std::fabs(e.a1 - e.a0) >= pi - 1e-6) out.circles.push_back({{e.cr, 0, e.cz}, {0, -1, 0}, e.rad});
    }
  }
  // Where pieces meet: a circle off the axis, a point on it (with a corner where the surface comes to a point).
  for (size_t k = 0; k < ne; k++) {
    size_t next = (k + 1) % ne;
    const Elem &e = m.profile[k];
    double r = e.r1, z = e.z1;
    if (r > 0) {
      Solid::Edge c;
      c.pts.reserve(count + 1);
      for (int j = 0; j <= count; j++) c.pts.push_back({r * cs[j], r * sn[j], z});
      sides(c, faceOf[k], faceOf[next]);
      c.geom.kind = EdgeGeom::Circle, c.geom.r = r, c.geom.z = z;
      out.edges.push_back(c);
      out.circles.push_back({{0, 0, z}, {0, 0, 1}, r});
      out.corners.push_back({r, 0, z});
    } else {
      for (size_t q : {k, next}) {
        const Elem &x = m.profile[q];
        if (x.onAxis() || x.flat()) continue;
        Solid::Edge point;
        point.f0 = faceOf[q];
        out.edges.push_back(point);
        out.corners.push_back({0, 0, z});
        break;
      }
    }
  }
}

// MARK: swept along an oval

struct Oval {
  double a, b, c, s;
  V3 at(double t) const {
    double x = a * trig::cos(t), y = b * trig::sin(t);
    return {c * x - s * y, s * x + c * y, 0};
  }
  V3 velocity(double t) const {
    double x = -a * trig::sin(t), y = b * trig::cos(t);
    return {c * x - s * y, s * x + c * y, 0};
  }
  V3 normal(double t) const {
    double x = b * trig::cos(t), y = a * trig::sin(t), l = trig::hypot(x, y);
    return {(c * x - s * y) / l, (s * x + c * y) / l, 0};
  }
  // Radius of curvature.
  double bend(double t) const {
    double v = norm(velocity(t));
    return v * v * v / (a * b);
  }
};

void buildSwept(const Model &m, Solid &out, double d) {
  Oval o{m.a, m.b, trig::cos(m.phi), trig::sin(m.phi)};
  double reach = 0;
  for (const auto &e : m.section) reach = std::max({reach, e.r0, e.r1, e.arc ? e.cr + e.rad : 0.0});
  // Steps along the oval: the outermost line's chords within d, and turning by no more than maxTurn.
  double step = 2 * pi;
  for (int k = 0; k < 256; k++) {
    double t = 2 * pi * k / 256, v = norm(o.velocity(t)), rho = o.bend(t), outer = rho + reach;
    // The tangent turns at v / rho on every line along the oval; the outermost needs the smallest turns.
    double dt = chordAngle(outer, d) * rho / v;
    step = std::min(step, dt);
  }
  int count = std::max(8, (int)std::ceil(2 * pi / step));
  std::vector<V3> at(count), nm(count);
  for (int j = 0; j < count; j++) {
    double t = 2 * pi * j / count;
    at[j] = o.at(t), nm[j] = o.normal(t);
  }
  const size_t ne = m.section.size();
  std::vector<int> faceOf(ne);
  for (size_t k = 0; k < ne; k++) {
    const Elem &e = m.section[k];
    int f = (int)out.faces.size();
    faceOf[k] = f;
    double nr, nz;
    e.normalAt(0.5, nr, nz);
    out.faces.push_back({unit(nm[0] * nr + V3{0, 0, nz}), {}, {}});
    int pieces = e.pieces(d);
    std::vector<uint32_t> ring(pieces + 1);
    for (int i = 0; i <= pieces; i++) {
      double t = (double)i / pieces, u, z;
      if (e.arc) {
        double a = e.a0 + (e.a1 - e.a0) * t;
        u = e.cr + e.rad * trig::cos(a), z = e.cz + e.rad * trig::sin(a);
        if (i == pieces) u = e.r1, z = e.z1;
        if (i == 0) u = e.r0, z = e.z0;
      } else {
        u = e.r0 + (e.r1 - e.r0) * t, z = e.z0 + (e.z1 - e.z0) * t;
      }
      e.normalAt(t, nr, nz);
      ring[i] = (uint32_t)out.p.size();
      for (int j = 0; j < count; j++) out.vertex(at[j] + nm[j] * u + V3{0, 0, z}, unit(nm[j] * nr + V3{0, 0, nz}));
    }
    for (int i = 0; i < pieces; i++)
      for (int j = 0; j < count; j++) {
        int jn = (j + 1) % count;
        uint32_t A = ring[i] + j, B = ring[i] + jn, C = ring[i + 1] + jn, D = ring[i + 1] + j;
        out.triangle(A, B, C, f);
        out.triangle(A, C, D, f);
      }
    Solid::Edge seam;
    for (int i = 0; i <= pieces; i++) seam.pts.push_back(out.p[ring[i]]);
    seam.f0 = f;
    out.edges.push_back(seam);
  }
  for (size_t k = 0; k < ne; k++) {
    const Elem &e = m.section[k];
    Solid::Edge rail;
    rail.pts.reserve(count + 1);
    for (int j = 0; j <= count; j++) rail.pts.push_back(at[j % count] + nm[j % count] * e.r1 + V3{0, 0, e.z1});
    sides(rail, faceOf[k], faceOf[(k + 1) % ne]);
    out.edges.push_back(rail);
    out.corners.push_back(at[0] + nm[0] * e.r1 + V3{0, 0, e.z1});
  }
}

}  // namespace

int Model::columns(double deflection) const {
  double d = std::isfinite(deflection) ? std::max(deflection, 1e-4) : 0.05;
  return around >= 3 ? around : std::max(3, countFor(2 * pi, support({1, 0, 0}), d));
}

void Model::build(Solid &out, double deflection) const {
  double d = std::isfinite(deflection) ? std::max(deflection, 1e-4) : 0.05;
  if (kind == Mesh) out = *mesh;
  if (kind == Poly) buildPoly(*this, out);
  if (kind == Turned) buildTurned(*this, out, d);
  if (kind == Extruded) buildExtruded(*this, out, d);
  if (kind == Revolved) buildRevolved(*this, out, d);
  if (kind == Radial) {
    // Checked when it was made at the details asked most; should another fail, nothing rather than a broken mesh.
    std::string why;
    if (!radialMesh(*radial, d, out, why)) out = Solid();
  }
  if (kind == Swept) {
    buildSwept(*this, out, d);
    // What the mesh misses, shared out by area (a tube's every face curves alike).
    std::vector<double> area(out.faces.size(), 0);
    double total = 0;
    for (size_t k = 0; k < out.triFace.size(); k++) {
      double w = norm(cross(out.p[out.tri[3 * k + 1]] - out.p[out.tri[3 * k]], out.p[out.tri[3 * k + 2]] - out.p[out.tri[3 * k]]));
      area[out.triFace[k]] += w, total += w;
    }
    double miss = volume - out.meshVolume();
    for (size_t f = 0; f < out.faces.size(); f++) out.faces[f].deficit = total > 0 ? miss * area[f] / total : 0;
  }
}

// MARK: - exact sizes

bool Model::exactAlong(V3 d) const {
  if (kind != Radial) return true;
  double h = trig::hypot(d.x, d.y), len = norm(d);
  return std::fabs(d.z) <= 1e-12 * len || h <= 1e-12 * len;
}

double Model::support(V3 d, V3 *at) const {
  double best = -INFINITY;
  V3 where;
  if (kind == Radial) return radialSupport(*radial, d, at, nullptr);
  if (kind == Extruded || kind == Revolved) {
    best = kind == Extruded ? extrudedSupport(*this, d, where) : revolvedSupport(*this, d, where);
  } else if (kind == Poly || kind == Mesh) {
    for (const auto &v : kind == Mesh ? mesh->p : verts)
      if (dot(v, d) > best) best = dot(v, d), where = v;
  } else if (kind == Turned) {
    double D = trig::hypot(d.x, d.y), dz = d.z, c = D > 0 ? d.x / D : 1, sn = D > 0 ? d.y / D : 0, br = 0, bz = 0;
    auto take = [&](double r, double z) {
      if (r * D + z * dz > best) best = r * D + z * dz, br = r, bz = z;
    };
    for (const auto &e : profile) {
      take(e.r0, e.z0), take(e.r1, e.z1);
      if (e.arc) {
        double a = trig::atan2(dz, D), lo = std::min(e.a0, e.a1), hi = std::max(e.a0, e.a1);
        while (a < lo) a += 2 * pi;
        while (a >= lo + 2 * pi) a -= 2 * pi;
        if (a <= hi) take(e.cr + e.rad * trig::cos(a), e.cz + e.rad * trig::sin(a));
      }
    }
    where = {br * c, br * sn, bz};
  } else {
    // Swept: the best along the oval, sampled and then narrowed down.
    Oval o{a, b, trig::cos(phi), trig::sin(phi)};
    auto value = [&](double t, V3 *pt) {
      V3 c = o.at(t), nm = o.normal(t);
      double s = dot(nm, d), m = -INFINITY, bu = 0, bz = 0;
      for (const auto &e : section) {
        if (e.r0 * s + e.z0 * d.z > m) m = e.r0 * s + e.z0 * d.z, bu = e.r0, bz = e.z0;
        if (e.r1 * s + e.z1 * d.z > m) m = e.r1 * s + e.z1 * d.z, bu = e.r1, bz = e.z1;
        double h = trig::hypot(s, d.z);
        if (e.arc && h > 0 && e.cr * s + e.cz * d.z + e.rad * h > m)
          m = e.cr * s + e.cz * d.z + e.rad * h, bu = e.cr + e.rad * s / h, bz = e.cz + e.rad * d.z / h;
      }
      if (pt) *pt = c + nm * bu + V3{0, 0, bz};
      return dot(c, d) + m;
    };
    const int n = 720;
    int k = 0;
    for (int i = 0; i < n; i++)
      if (value(2 * pi * i / n, nullptr) > value(2 * pi * k / n, nullptr)) k = i;
    double lo = 2 * pi * (k - 1) / n, hi = 2 * pi * (k + 1) / n;
    const double g = (std::sqrt(5.0) - 1) / 2;
    for (int i = 0; i < 80; i++) {
      double x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo);
      if (value(x1, nullptr) < value(x2, nullptr)) lo = x1; else hi = x2;
    }
    double tm = (lo + hi) / 2, tk = 2 * pi * k / n;
    best = value(tm, nullptr) >= value(tk, nullptr) ? value(tm, &where) : value(tk, &where);
  }
  if (at) *at = where;
  return best;
}

// MARK: - primitives

namespace {

// ∮ r²/2 dz along a piece: its share of the area moment that, times 2π, is the volume it turns round.
double moment(const Elem &e) {
  if (!e.arc) return (e.z1 - e.z0) * (e.r0 * e.r0 + e.r0 * e.r1 + e.r1 * e.r1) / 6;
  double c = e.cr, R = e.rad;
  auto F = [&](double t) {
    double s = trig::sin(t);
    return R / 2 * (c * c * s + 2 * c * R * (t / 2 + trig::sin(2 * t) / 4) + R * R * (s - s * s * s / 3));
  };
  return F(e.a1) - F(e.a0);
}

// Joins a closed outline exactly: each piece starts where the one before ends (rounding in an arc's ends left out), and
// whatever is within rounding of the axis is on it.
void closeUp(std::vector<Elem> &loop) {
  for (auto &e : loop) {
    double tiny = 1e-12 * (std::fabs(e.r0) + std::fabs(e.r1) + std::fabs(e.z0) + std::fabs(e.z1) + e.rad);
    for (double *v : {&e.r0, &e.z0, &e.r1, &e.z1})
      if (std::fabs(*v) < tiny) *v = 0;
  }
  for (size_t k = 1; k < loop.size(); k++) loop[k].r0 = loop[k - 1].r1, loop[k].z0 = loop[k - 1].z1;
  if (!loop.empty()) loop.back().r1 = loop.front().r0, loop.back().z1 = loop.front().z0;
}

// The profile through these points (a straight piece between each two, closing back to the first), skipping repeats.
std::vector<Elem> polyline(const std::vector<std::pair<double, double>> &pts) {
  std::vector<Elem> out;
  for (size_t i = 0; i < pts.size(); i++) {
    auto a = pts[i], b = pts[(i + 1) % pts.size()];
    if (a != b) out.push_back(Elem::line(a.first, a.second, b.first, b.second));
  }
  return out;
}

std::shared_ptr<Model> turned(std::vector<Elem> profile) {
  auto m = std::make_shared<Model>();
  m->kind = Model::Turned;
  m->profile = std::move(profile);
  closeUp(m->profile);
  double v = 0;
  for (const auto &e : m->profile) v += moment(e);
  m->volume = 2 * pi * std::fabs(v);
  return m;
}

double polyVolume(const Model &m) {
  double v = 0;
  for (const auto &loop : m.loops)
    for (size_t i = 1; i + 1 < loop.size(); i++) v += dot(m.verts[loop[0]], cross(m.verts[loop[i]], m.verts[loop[i + 1]]));
  return std::fabs(v) / 6;
}

// Regular n-gon at height z, corners on a circle of radius r, one flat facing -y (as BcadKernel's).
std::vector<V3> ngon(int n, double r, double z) {
  std::vector<V3> pts;
  double start = -pi / 2 + pi / n;
  for (int i = 0; i < n; i++) {
    double a = start + 2 * pi * i / n;
    pts.push_back({r * trig::cos(a), r * trig::sin(a), z});
  }
  return pts;
}

// Sides between two loops of n (bottom i, top n + i), bottom and top.
std::shared_ptr<Model> prismOf(const std::vector<V3> &bottom, const std::vector<V3> &top) {
  auto m = std::make_shared<Model>();
  int n = (int)bottom.size();
  m->verts = bottom;
  m->verts.insert(m->verts.end(), top.begin(), top.end());
  for (int i = 0; i < n; i++) m->loops.push_back({i, (i + 1) % n, n + (i + 1) % n, n + i});
  std::vector<int> lo, hi;
  for (int i = n - 1; i >= 0; i--) lo.push_back(i);
  for (int i = 0; i < n; i++) hi.push_back(n + i);
  m->loops.push_back(lo);
  m->loops.push_back(hi);
  m->volume = polyVolume(*m);
  return m;
}

// The outline of a polygon tube round radius r, as BcadKernel's: a triangle with its point up (3) or a hexagon lying flat
// (6), 2w wide and w·√3 tall.
std::vector<std::pair<double, double>> tubeOutline(int sides, double w, double r) {
  double h = w * std::sqrt(3.0) / 2;
  if (sides == 3) return {{r - w, -h}, {r + w, -h}, {r, h}};
  return {{r - w, 0}, {r - w / 2, -h}, {r + w / 2, -h}, {r + w, 0}, {r + w / 2, h}, {r - w / 2, h}};
}

// The ellipse with conjugate semi-diameters (a, 0) and b·(cos t, sin t): its semi-axes and the major one's angle.
void conjugate(double a, double b, double t, double &major, double &minor, double &phi) {
  double sxx = a * a + b * b * trig::cos(t) * trig::cos(t), sxy = b * b * trig::cos(t) * trig::sin(t), syy = b * b * trig::sin(t) * trig::sin(t);
  double mean = (sxx + syy) / 2, spread = trig::hypot((sxx - syy) / 2, sxy);
  major = std::sqrt(mean + spread), minor = std::sqrt(std::max(mean - spread, 0.0)), phi = trig::atan2(2 * sxy, sxx - syy) / 2;
}

double ellipseLength(double a, double b) {
  // The trapezoid rule is exact to rounding for a smooth periodic integrand with enough points.
  const int n = 1024;
  double sum = 0;
  for (int i = 0; i < n; i++) {
    double t = 2 * pi * i / n;
    sum += trig::hypot(a * trig::sin(t), b * trig::cos(t));
  }
  return sum * 2 * pi / n;
}

Affine turnZ(double phi) {
  Affine r;
  double c = trig::cos(phi), s = trig::sin(phi);
  r.m[0] = c, r.m[1] = -s, r.m[4] = s, r.m[5] = c;
  return r;
}

}  // namespace

std::shared_ptr<Model> turnedModel(std::vector<Elem> profile) { return turned(std::move(profile)); }

std::shared_ptr<Model> polyModel(std::vector<V3> verts, std::vector<std::vector<int>> loops) {
  auto m = std::make_shared<Model>();
  m->verts = std::move(verts);
  m->loops = std::move(loops);
  double v = 0;
  for (const auto &loop : m->loops)
    for (size_t i = 1; i + 1 < loop.size(); i++) v += dot(m->verts[loop[0]], cross(m->verts[loop[i]], m->verts[loop[i + 1]]));
  if (v < 0)
    for (auto &loop : m->loops) std::reverse(loop.begin(), loop.end());
  m->volume = std::fabs(v) / 6;
  return m;
}

std::shared_ptr<Model> meshModel(const std::vector<V3> &pts, const std::vector<uint32_t> &tris, std::string &why, bool uncrossed) {
  size_t nv = pts.size(), nt = tris.size() / 3;
  if (nv < 4 || nt < 4 || tris.size() % 3) return why = "a mesh needs 4 points and 4 triangles at least", nullptr;
  for (V3 q : pts)
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) return why = "points must be numbers", nullptr;
  for (size_t t = 0; t < nt; t++) {
    uint32_t a = tris[3 * t], b = tris[3 * t + 1], c = tris[3 * t + 2];
    if (a >= nv || b >= nv || c >= nv) return why = "a triangle's corner isn't one of the points", nullptr;
    if (a == b || b == c || c == a) return why = "a triangle has a corner twice", nullptr;
  }
  // Each side (from a corner to the next) once, and met by one running back along it: closed, facing one way throughout.
  std::vector<std::pair<uint64_t, uint32_t>> sides(3 * nt);
  for (size_t t = 0; t < nt; t++)
    for (int k = 0; k < 3; k++) sides[3 * t + k] = {(uint64_t)tris[3 * t + k] << 32 | tris[3 * t + (k + 1) % 3], (uint32_t)t};
  std::sort(sides.begin(), sides.end());
  Find piece(nt);
  for (size_t i = 0; i < sides.size(); i++) {
    if (i + 1 < sides.size() && sides[i].first == sides[i + 1].first) return why = "two triangles run the same way along a side", nullptr;
    uint64_t back = sides[i].first << 32 | sides[i].first >> 32;
    auto it = std::lower_bound(sides.begin(), sides.end(), std::make_pair(back, (uint32_t)0));
    if (it == sides.end() || it->first != back) return why = "open", nullptr;
    piece.join(sides[i].second, it->second);
  }
  Solid s;
  // (Points no triangle uses left out, the rest in their order: they'd count in its box.)
  std::vector<uint32_t> at(nv, 0);
  for (uint32_t i : tris) at[i] = 1;
  for (size_t i = 0; i < nv; i++)
    if (at[i]) at[i] = (uint32_t)s.p.size(), s.p.push_back(pts[i]);
  s.tri = tris;
  for (uint32_t &i : s.tri) i = at[i];
  nv = s.p.size();
  s.n.assign(nv, V3{0, 0, 0});
  s.triFace.resize(nt);
  // A face per connected piece, numbered by its first triangle; each point's normal its triangles' by their area.
  std::vector<int> faceOf(nt, -1);
  for (uint32_t t = 0; t < nt; t++) {
    uint32_t r = piece(t);
    V3 a = s.p[s.tri[3 * t]], b = s.p[s.tri[3 * t + 1]], c = s.p[s.tri[3 * t + 2]], w = cross(b - a, c - a);
    if (faceOf[r] < 0) {
      faceOf[r] = (int)s.faces.size();
      s.faces.push_back({});
    }
    s.triFace[t] = (uint32_t)faceOf[r];
    Solid::Face &f = s.faces[faceOf[r]];
    // (The face's normal: its first triangle's that has an area, as a curved face has no one normal.)
    if (norm(f.normal) == 0 && norm(w) > 0) f.normal = unit(w);
    for (int k = 0; k < 3; k++) s.n[s.tri[3 * t + k]] += w;
  }
  for (V3 &q : s.n) q = norm(q) > 0 ? unit(q) : V3{0, 0, 1};
  for (auto &f : s.faces)
    if (norm(f.normal) == 0) f.normal = {0, 0, 1};
  s.centroids();
  double v = s.meshVolume();
  if (!(v > 0)) return why = "inside out", nullptr;
  // Each piece the right way out, or a hollow inside one that is (a piece inside out beside the rest, a larger one
  // hiding it in the sum, was let through, and the next merge saw a solid less than nothing there).
  if (s.faces.size() > 1) {
    std::vector<double> vol(s.faces.size(), 0);
    for (size_t t = 0; t < nt; t++) {
      V3 a = s.p[s.tri[3 * t]], b = s.p[s.tri[3 * t + 1]], c = s.p[s.tri[3 * t + 2]];
      vol[s.triFace[t]] += dot(a, cross(b, c)) / 6;
    }
    Solid outer;
    outer.p = s.p;
    for (size_t t = 0; t < nt; t++)
      if (vol[s.triFace[t]] > 0) outer.tri.insert(outer.tri.end(), {s.tri[3 * t], s.tri[3 * t + 1], s.tri[3 * t + 2]});
    for (size_t t = 0; t < nt; t++) {
      uint32_t f = s.triFace[t];
      if (!(vol[f] < 0)) continue;
      V3 q = (s.p[s.tri[3 * t]] + s.p[s.tri[3 * t + 1]] + s.p[s.tri[3 * t + 2]]) / 3;
      if (!inside(outer, q)) return why = "inside out", nullptr;
      vol[f] = 0;  // (one point of each is enough)
    }
  }
  // Passing through itself (a sculpt pulled through itself, pieces overlapping): the solid it encloses, its outer skin.
  if (!uncrossed && selfCrossing(s.p, s.tri)) {
    Solid r = resolved(s);
    double rv = r.meshVolume();
    if (r.tri.empty() || !(rv > 0)) return why = "it passes through itself", nullptr;
    s = std::move(r), v = rv;
  }
  auto m = std::make_shared<Model>();
  m->kind = Model::Mesh, m->volume = v;
  m->mesh = std::make_shared<const Solid>(std::move(s));
  return m;
}

Shape shapeOf(std::shared_ptr<const Model> m, const Affine &place) {
  auto node = std::make_shared<Node>();
  node->model = std::move(m);
  return {node, place};
}

Shape merged(int op, const Shape &a, const Shape &b) {
  auto node = std::make_shared<Node>();
  node->kind = Node::Bool, node->op = op, node->a = a, node->b = b;
  return {node, Affine()};
}

bool primitive(int kind, const double *p, Shape &out, std::string &why) {
  // How many numbers each kind takes, and which must be above zero (a cone end and a ring's hole may be 0).
  static const int counts[] = {3, 2, 3, 1, 3, 3, 3, 3, 1, 2, 3, 4, 4, 5};
  static const std::vector<int> above[] = {{0, 1, 2}, {0, 1}, {2}, {0}, {1, 2}, {1, 2}, {0, 1, 2}, {1, 2}, {0}, {0, 1}, {0, 2}, {0, 1, 2, 3}, {0, 1, 3}, {1, 2, 4}};
  auto fail = [&](const char *w) {
    why = std::string("shape: ") + w;
    return false;
  };
  if (kind < BK_BOX || kind > BK_OVAL_TORUS) return fail("unknown shape");
  for (int i = 0; i < counts[kind]; i++)
    if (!std::isfinite(p[i])) return fail("sizes must be numbers");
  for (int i = 0; i < counts[kind]; i++)
    if (p[i] < 0) return fail("sizes can't be below zero");
  // (Past 100 m: no printer's, and finer than its size allows, its mesh would take more memory than any machine has.)
  for (int i = 0; i < counts[kind]; i++)
    if (p[i] > 1e5 && !((kind == BK_PRISM || kind == BK_PYRAMID || kind == BK_TORUS || kind == BK_OVAL_TORUS) && i == 0))
      return fail("a shape is at most 100 m across");
  for (int i : above[kind])
    if (p[i] < 0.001) return fail("sizes must be above zero");
  if (kind == BK_CONE && std::max(p[0], p[1]) < 0.001) return fail("a cone needs one end wider than zero");
  // (As the app takes them: up to 24. Rounding every edge of 64 took minutes; of thousands, far longer.)
  if ((kind == BK_PRISM || kind == BK_PYRAMID) && !(p[0] < 24.5)) return fail("a prism or pyramid has at most 24 sides");

  std::shared_ptr<Model> m;
  Affine pre;
  switch (kind) {
  case BK_BOX: {
    double x = p[0] / 2, y = p[1] / 2, z = p[2] / 2;
    m = prismOf({{-x, -y, -z}, {x, -y, -z}, {x, y, -z}, {-x, y, -z}}, {{-x, -y, z}, {x, -y, z}, {x, y, z}, {-x, y, z}});
    break;
  }
  case BK_CYLINDER: {
    double r = p[0] / 2, h = p[1] / 2;
    m = turned(polyline({{0, -h}, {r, -h}, {r, h}, {0, h}}));
    break;
  }
  case BK_CONE: {
    double r1 = p[0] / 2, r2 = p[1] / 2, h = p[2] / 2;
    if (std::fabs(r1 - r2) < 1e-6) r2 = r1;
    // (An end narrower than a hundredth of a millimetre made a point, as no printer tells them apart: a sliver of an
    // end that thin took minutes to hollow.)
    if (r1 < 0.005 && r2 >= 0.005) r1 = 0;
    if (r2 < 0.005 && r1 >= 0.005) r2 = 0;
    m = turned(polyline({{0, -h}, {r1, -h}, {r2, h}, {0, h}}));
    break;
  }
  case BK_SPHERE: {
    double r = p[0] / 2;
    m = turned({Elem::arcOf(0, 0, r, -pi / 2, pi / 2), Elem::line(0, r, 0, -r)});
    break;
  }
  case BK_PRISM: {
    int n = std::max(3, (int)std::lround(p[0]));
    m = prismOf(ngon(n, p[1] / 2, 0), ngon(n, p[1] / 2, p[2]));
    break;
  }
  case BK_TORUS: {
    // (A number of sides past what an int holds is no tube shape: not rounded into one.)
    if (!(std::fabs(p[0]) < 1000)) return fail("unknown tube shape");
    int sides = (int)std::lround(p[0]);
    if (sides != 0 && sides != 3 && sides != 6) return fail("unknown tube shape");
    double tube = p[2] / 2, ring = p[1] / 2 - tube;
    if (sides == 0) {
      if (!(ring > tube * 0.05)) return fail("the tube is too thick for this torus");
      m = turned({Elem::arcOf(ring, 0, tube, 0, 2 * pi)});
    } else {
      // A polygon tube keeps a hole: crossing the axis would make the turned outline overlap itself.
      if (!(ring - tube > tube * 0.05)) return fail("the tube is too thick for this torus");
      m = turned(polyline(tubeOutline(sides, tube, ring)));
    }
    break;
  }
  case BK_WEDGE: {
    // A right triangle across x and y (the right angle at the low corners, sloping down along +x) stood up along z.
    double x = p[0], y = p[1], z = p[2];
    m = prismOf({{0, 0, 0}, {x, 0, 0}, {0, y, 0}}, {{0, 0, z}, {x, 0, z}, {0, y, z}});
    break;
  }
  case BK_PYRAMID: {
    int n = std::max(3, (int)std::lround(p[0]));
    m = std::make_shared<Model>();
    m->verts = ngon(n, p[1] / 2, 0);
    m->verts.push_back({0, 0, p[2]});
    for (int i = 0; i < n; i++) m->loops.push_back({i, (i + 1) % n, n});
    std::vector<int> base;
    for (int i = n - 1; i >= 0; i--) base.push_back(i);
    m->loops.push_back(base);
    m->volume = polyVolume(*m);
    break;
  }
  case BK_HEMISPHERE: {
    double r = p[0] / 2;
    m = turned({Elem::line(0, 0, r, 0), Elem::arcOf(0, 0, r, 0, pi / 2), Elem::line(0, r, 0, 0)});
    break;
  }
  case BK_BOWL: {
    // The lower half of a spherical shell (centre at rim height), its rim flat at the top.
    double r = p[0] / 2, t = std::min(std::max(p[1], 0.05), r * 0.95), ri = r - t;
    m = turned({Elem::arcOf(0, r, r, -pi / 2, 0), Elem::line(r, r, ri, r), Elem::arcOf(0, r, ri, 0, -pi / 2), Elem::line(0, t, 0, 0)});
    break;
  }
  case BK_RING: {
    double ro = p[0] / 2, ri = std::min(std::max(p[1] / 2, 0.0), ro - 0.05), h = p[2];
    if (ri < 0.005) ri = 0;  // (as a cone's end)
    m = turned(polyline({{ri, 0}, {ro, 0}, {ro, h}, {ri, h}}));
    break;
  }
  case BK_GLASS: {
    double r = p[0] / 2, h = p[1], w = std::min(std::max(p[2], 0.05), r * 0.95), b = std::min(std::max(p[3], 0.05), h * 0.95);
    m = turned(polyline({{0, 0}, {r, 0}, {r, h}, {r - w, h}, {r - w, b}, {0, b}}));
    break;
  }
  case BK_OVAL: {
    double a = std::max(p[0], 0.01) / 2, b = std::max(p[1], 0.01) / 2, t = std::min(std::max(p[2], 5.0), 175.0) * pi / 180;
    double major, minor, phi, h = p[3] / 2;
    conjugate(a, b, t, major, minor, phi);
    if (major - minor <= 1e-9 * major) {
      m = turned(polyline({{0, -h}, {major, -h}, {major, h}, {0, h}}));
      pre = turnZ(phi);
    } else {
      // A cylinder of radius 1 stretched to the oval.
      m = turned(polyline({{0, -h}, {1, -h}, {1, h}, {0, h}}));
      Affine stretch;
      stretch.m[0] = major, stretch.m[5] = minor;
      pre = stretch.then(turnZ(phi));
    }
    break;
  }
  case BK_OVAL_TORUS: {
    // A tube along the oval through its middle, upright all the way round.
    // (A number of sides past what an int holds is no tube shape: not rounded into one.)
    if (!(std::fabs(p[0]) < 1000)) return fail("unknown tube shape");
    int sides = (int)std::lround(p[0]);
    if (sides != 0 && sides != 3 && sides != 6) return fail("unknown tube shape");
    double w = p[4] / 2, a = p[1] / 2 - w, b = p[2] / 2 - w, t = std::min(std::max(p[3], 5.0), 175.0) * pi / 180;
    if (!(a > 0 && b > 0)) return fail("the tube is too thick for this torus");
    double major, minor, phi;
    conjugate(a, b, t, major, minor, phi);
    // The tube must fit the tightest bend (radius minor²/major) or its inner side folds over itself.
    if (!(w * 1.05 < minor * minor / major)) return fail("the tube is too thick for this torus");
    m = std::make_shared<Model>();
    m->kind = Model::Swept;
    m->a = major, m->b = minor, m->phi = phi;
    double area;
    if (sides == 0) {
      m->section = {Elem::arcOf(0, 0, w, 0, 2 * pi)};
      closeUp(m->section);
      area = pi * w * w;
    } else {
      m->section = polyline(tubeOutline(sides, w, 0));
      area = 0;
      for (const auto &e : m->section) area += e.r0 * e.z1 - e.r1 * e.z0;
      area = std::fabs(area) / 2;
    }
    // The section's centroid lies on the oval's line, so the volume is its area times the line's length.
    m->volume = area * ellipseLength(major, minor);
    break;
  }
  }
  auto node = std::make_shared<Node>();
  node->model = m;
  out.node = node;
  out.place = pre;
  // Centred on its bounding box, as all Bcad's shapes are.
  V3 lo, hi;
  bounds(out, lo, hi);
  out.place = out.place.then(Affine::translation((lo + hi) * -0.5));
  return true;
}

std::shared_ptr<Model> sweptModel(double a, double b, double phi, std::vector<Elem> section) {
  auto m = std::make_shared<Model>();
  m->kind = Model::Swept;
  m->a = a, m->b = b, m->phi = phi;
  closeUp(section);
  // The volume: each piece's area about the oval's line (its centroid's offset off that line bends the length a little;
  // taken as the line's length, as the primitives are made).
  double area = 0;
  for (const auto &e : section) {
    if (!e.arc) {
      area += e.r0 * e.z1 - e.r1 * e.z0;
      continue;
    }
    for (int k = 0; k < 64; k++) {
      double r0, z0, r1, z1;
      e.at(k / 64.0, r0, z0), e.at((k + 1) / 64.0, r1, z1);
      area += r0 * z1 - r1 * z0;
    }
  }
  m->section = std::move(section);
  m->volume = std::fabs(area) / 2 * ellipseLength(a, b);
  return m;
}

double profileMoment(const Elem &e) { return moment(e); }

// MARK: - a sketch's regions, stood up or turned

namespace {

// The largest (x, y) · (dx, dy) over a piece in the plane so far, and where.
void pieceSupport(const Elem &e, double dx, double dy, double &best, double &bx, double &by) {
  auto take = [&](double x, double y) {
    if (x * dx + y * dy > best) best = x * dx + y * dy, bx = x, by = y;
  };
  take(e.r0, e.z0), take(e.r1, e.z1);
  if (e.arc && (dx != 0 || dy != 0)) {
    double a = trig::atan2(dy, dx), lo = std::min(e.a0, e.a1), hi = std::max(e.a0, e.a1);
    while (a < lo) a += 2 * pi;
    while (a >= lo + 2 * pi) a -= 2 * pi;
    if (a <= hi) take(e.cr + e.rad * trig::cos(a), e.cz + e.rad * trig::sin(a));
  }
}

double extrudedSupport(const Model &m, V3 d, V3 &where) {
  double best = -INFINITY, bx = 0, by = 0;
  for (const auto &loop : m.outline)
    for (const Elem &e : loop) pieceSupport(e, d.x, d.y, best, bx, by);
  double z = m.lo * d.z >= m.hi * d.z ? m.lo : m.hi;
  where = {bx, by, z};
  return best + z * d.z;
}

double revolvedSupport(const Model &m, V3 d, V3 &where) {
  // Every point at radius r reaches r·D·cos(φ − ψ) round the axis: furthest at the same angle φ for all of them (ψ itself
  // where the turn passes it, otherwise the end nearer it).
  double D = trig::hypot(d.x, d.y), psi = trig::atan2(d.y, d.x), phi = psi, g = 1;
  if (m.turn < 2 * pi) {
    double p = psi < 0 ? psi + 2 * pi : psi;
    if (p <= m.turn) {
      phi = p;
    } else {
      double c0 = trig::cos(psi), c1 = trig::cos(m.turn - psi);
      phi = c0 >= c1 ? 0 : m.turn, g = std::max(c0, c1);
    }
  }
  double best = -INFINITY, br = 0, bz = 0;
  for (const auto &loop : m.outline)
    for (const Elem &e : loop) pieceSupport(e, D * g, d.z, best, br, bz);
  where = {br * trig::cos(phi), br * trig::sin(phi), bz};
  return best;
}

}  // namespace

std::vector<std::vector<int>> fewestChords(const std::vector<std::vector<Elem>> &loops) {
  std::vector<std::vector<int>> out(loops.size());
  struct Ends {
    double v[4];
    int l, k;
  };
  std::vector<Ends> all;
  for (size_t l = 0; l < loops.size(); l++) {
    out[l].assign(loops[l].size(), 1);
    for (size_t k = 0; k < loops[l].size(); k++) {
      const Elem &e = loops[l][k];
      bool turn = e.r1 < e.r0 || (e.r1 == e.r0 && e.z1 < e.z0);
      all.push_back({{turn ? e.r1 : e.r0, turn ? e.z1 : e.z0, turn ? e.r0 : e.r1, turn ? e.z0 : e.z1}, (int)l, (int)k});
    }
  }
  auto same = [](const Ends &p, const Ends &q) { return std::equal(p.v, p.v + 4, q.v); };
  std::sort(all.begin(), all.end(), [&](const Ends &p, const Ends &q) {
    for (int i = 0; i < 4; i++)
      if (p.v[i] != q.v[i]) return p.v[i] < q.v[i];
    return p.l != q.l ? p.l < q.l : p.k < q.k;
  });
  for (size_t i = 0, j; i < all.size(); i = j) {
    for (j = i + 1; j < all.size() && same(all[i], all[j]);) j++;
    for (size_t t = i; j - i > 1 && t < j; t++)
      if (loops[all[t].l][all[t].k].arc) out[all[t].l][all[t].k] = 2;
  }
  return out;
}

namespace {

// A loop's points, each arc in chords (as many as its turn needs at deflection d, times `more`), with the piece each
// point's side to the next lies on and where each piece starts.
struct Loop2 {
  std::vector<double> x, y;
  std::vector<int> start;
};

Loop2 loopPoints(const std::vector<Elem> &loop, double d, int more, const std::vector<int> &least) {
  Loop2 r;
  for (size_t k = 0; k < loop.size(); k++) {
    const Elem &e = loop[k];
    r.start.push_back((int)r.x.size());
    r.x.push_back(e.r0), r.y.push_back(e.z0);
    if (!e.arc) continue;
    int n = std::max(countFor(e.a1 - e.a0, e.rad, d), least[k]) * more;
    for (int j = 1; j < n; j++) {
      double a = e.a0 + (e.a1 - e.a0) * j / n;
      r.x.push_back(e.cr + e.rad * trig::cos(a)), r.y.push_back(e.cz + e.rad * trig::sin(a));
    }
  }
  return r;
}

// The inside of a region's loops (their points numbered on from loop to loop) as triangles counter-clockwise, by a
// triangulation keeping every side; false where sides cross (chords of curves too near each other) or nothing is inside.
bool regionCap(const std::vector<const Loop2 *> &loops, std::vector<int> &out) {
  double lo[2] = {INFINITY, INFINITY}, hi[2] = {-INFINITY, -INFINITY};
  size_t total = 0;
  for (const Loop2 *l : loops) {
    for (size_t j = 0; j < l->x.size(); j++)
      lo[0] = std::min(lo[0], l->x[j]), lo[1] = std::min(lo[1], l->y[j]), hi[0] = std::max(hi[0], l->x[j]), hi[1] = std::max(hi[1], l->y[j]);
    total += l->x.size();
  }
  if (total < 3) return false;
  double w = std::max(hi[0] - lo[0], hi[1] - lo[1]) + 1, cx = (lo[0] + hi[0]) / 2, cy = (lo[1] + hi[1]) / 2;
  Tri2 tri(cx - 4 * w, cy - 3 * w, cx + 4 * w, cy - 3 * w, cx, cy + 5 * w);
  std::vector<int> id(total);
  std::map<int, int> back;
  size_t k = 0;
  for (const Loop2 *l : loops)
    for (size_t j = 0; j < l->x.size(); j++, k++) {
      id[k] = tri.insert(l->x[j], l->y[j]);
      back.insert({id[k], (int)k});
    }
  k = 0;
  for (const Loop2 *l : loops) {
    size_t n = l->x.size();
    for (size_t j = 0; j < n; j++) {
      int a = id[k + j], b = id[k + (j + 1) % n];
      if (a != b && !tri.keep(a, b)) return false;
    }
    k += n;
  }
  if (!tri.made().empty()) return false;
  out.clear();
  for (int q : tri.insideKept()) {
    auto it = back.find(q);
    if (it == back.end()) return false;
    out.push_back(it->second);
  }
  return !out.empty();
}

int regionCount(const Model &m) {
  int n = 0;
  for (int g : m.region) n = std::max(n, g + 1);
  return n;
}

bool extrudeAt(const Model &m, Solid &out, double d, int more) {
  size_t nl = m.outline.size();
  int regions = regionCount(m);
  std::vector<Loop2> rings(nl);
  auto least = fewestChords(m.outline);
  for (size_t l = 0; l < nl; l++) rings[l] = loopPoints(m.outline[l], d, more, least[l]);
  // The ends: a flat face each, top and bottom, per region.
  std::vector<std::array<int, 2>> capFace(regions, {-1, -1});
  for (int g = 0; g < regions; g++) {
    std::vector<const Loop2 *> mine;
    std::vector<V3> pts;
    for (size_t l = 0; l < nl; l++)
      if (m.region[l] == g) {
        mine.push_back(&rings[l]);
        for (size_t j = 0; j < rings[l].x.size(); j++) pts.push_back({rings[l].x[j], rings[l].y[j], 0});
      }
    std::vector<int> tris;
    if (mine.empty() || !regionCap(mine, tris)) return false;
    for (int up = 0; up < 2; up++) {
      double z = up ? m.hi : m.lo;
      V3 nrm{0, 0, up ? 1.0 : -1.0};
      int f = (int)out.faces.size();
      Solid::Face face{nrm, {}, {}};
      face.geom.kind = FaceGeom::Flat, face.geom.flat = true, face.geom.pn = nrm, face.geom.pd = up ? z : -z;
      out.faces.push_back(face);
      capFace[g][up] = f;
      uint32_t base = (uint32_t)out.p.size();
      for (V3 q : pts) out.vertex({q.x, q.y, z}, nrm);
      for (size_t t = 0; t + 2 < tris.size(); t += 3) {
        uint32_t a = base + tris[t], b = base + tris[t + 1], c = base + tris[t + 2];
        if (up) out.triangle(a, b, c, f);
        else out.triangle(a, c, b, f);
      }
    }
  }
  // The sides: a flat face per straight piece, a cylinder's per arc (what its chords miss worked out).
  for (size_t l = 0; l < nl; l++) {
    const auto &loop = m.outline[l];
    const Loop2 &r = rings[l];
    int g = m.region[l], np = (int)loop.size(), n = (int)r.x.size();
    std::vector<int> faceOf(np);
    for (int k = 0; k < np; k++) {
      const Elem &e = loop[k];
      int s = r.start[k], count = (k + 1 < np ? r.start[k + 1] : n) - s + 1;
      double sign = e.arc && e.a1 < e.a0 ? -1 : 1;
      // Outward: to the right of the way the loop runs.
      auto normal = [&](double x, double y) -> V3 {
        double dx = e.arc ? sign * (x - e.cr) : e.z1 - e.z0, dy = e.arc ? sign * (y - e.cz) : e.r0 - e.r1, l = trig::hypot(dx, dy);
        return l > 0 ? V3{dx / l, dy / l, 0} : V3{1, 0, 0};
      };
      int f = (int)out.faces.size();
      faceOf[k] = f;
      Solid::Face face;
      if (!e.arc) {
        V3 nrm = normal(0, 0);
        face.normal = nrm;
        face.geom.kind = FaceGeom::Flat, face.geom.flat = true, face.geom.pn = nrm, face.geom.pd = nrm.x * e.r0 + nrm.y * e.z0;
      } else {
        double am = (e.a0 + e.a1) / 2;
        face.normal = {sign * trig::cos(am), sign * trig::sin(am), 0};
        face.geom.kind = FaceGeom::Turned;
        face.geom.elem = sign > 0 ? Elem::line(e.rad, m.lo, e.rad, m.hi) : Elem::line(e.rad, m.hi, e.rad, m.lo);
        face.geom.place = Affine::translation({e.cr, e.cz, 0});
        double a = std::fabs(e.a1 - e.a0) / (count - 1);
        face.deficit = sign * (m.hi - m.lo) * (count - 1) * e.rad * e.rad / 2 * (a - trig::sin(a));
      }
      out.faces.push_back(face);
      uint32_t base = (uint32_t)out.p.size();
      for (int j = 0; j < count; j++) {
        int q = (s + j) % n;
        V3 nrm = normal(r.x[q], r.y[q]);
        out.vertex({r.x[q], r.y[q], m.lo}, nrm);
        out.vertex({r.x[q], r.y[q], m.hi}, nrm);
      }
      for (int j = 0; j + 1 < count; j++) {
        uint32_t aLo = base + 2 * j, aHi = aLo + 1, bLo = aLo + 2, bHi = aLo + 3;
        out.triangle(aLo, bLo, bHi, f);
        out.triangle(aLo, bHi, aHi, f);
      }
      // Its rims, bottom and top.
      for (int up = 0; up < 2; up++) {
        Solid::Edge rim;
        double z = up ? m.hi : m.lo;
        for (int j = 0; j < count; j++) rim.pts.push_back({r.x[(s + j) % n], r.y[(s + j) % n], z});
        sides(rim, f, capFace[g][up]);
        if (e.arc) {
          rim.geom.kind = EdgeGeom::Circle, rim.geom.r = e.rad, rim.geom.z = z, rim.geom.place = Affine::translation({e.cr, e.cz, 0});
        } else {
          rim.geom.kind = EdgeGeom::Line;
        }
        out.edges.push_back(rim);
      }
      if (e.arc && std::fabs(e.a1 - e.a0) >= pi - 1e-6)
        for (double z : {m.lo, m.hi}) out.circles.push_back({{e.cr, e.cz, z}, {0, 0, 1}, e.rad});
    }
    // Upright edges where pieces meet (a seam where the loop is one piece), with a corner at each end.
    for (int k = 0; k < np; k++) {
      int q = r.start[k];
      Solid::Edge up;
      up.pts = {{r.x[q], r.y[q], m.lo}, {r.x[q], r.y[q], m.hi}};
      up.geom.kind = EdgeGeom::Line;
      sides(up, faceOf[(k + np - 1) % np], faceOf[k]);
      out.edges.push_back(up);
      out.corners.push_back({r.x[q], r.y[q], m.lo});
      out.corners.push_back({r.x[q], r.y[q], m.hi});
    }
  }
  return true;
}

void buildExtruded(const Model &m, Solid &out, double d) {
  // (Where an end's triangulation fails, the arcs' chords of loops very near each other crossed: finer ones are tried.)
  for (int more = 1; more <= 16; more *= 2) {
    out = Solid();
    if (extrudeAt(m, out, d, more)) return;
  }
  out = Solid();
}

// Where a profile piece's mesh rings go, as t along it: evenly within the chord error (times `more` on an arc).
std::vector<double> ringsOf(const Elem &e, double d, int more, int least) {
  int n = e.arc ? std::max(countFor(e.a1 - e.a0, e.rad, d), least) * more : 1;
  std::vector<double> ts;
  for (int j = 0; j <= n; j++) ts.push_back(j == n ? 1.0 : (double)j / n);
  return ts;
}

bool revolveAt(const Model &m, Solid &out, double d, int more) {
  bool whole = m.turn >= 2 * pi;
  double T = whole ? 2 * pi : m.turn, reach = 0;
  for (const auto &loop : m.outline)
    for (const Elem &e : loop) reach = std::max({reach, e.r0, e.r1, e.arc ? e.cr + e.rad : 0.0});
  // One count round the axis for every face, so the faces meet point for point.
  int count = std::max(whole ? 3 : 1, countFor(T, reach, d)), cols = whole ? count : count + 1;
  std::vector<double> cs(count + 1), sn(count + 1), cm(count), sm(count);
  for (int j = 0; j <= count; j++) {
    bool start = j == 0 || (whole && j == count);
    cs[j] = start ? 1 : trig::cos(T * j / count), sn[j] = start ? 0 : trig::sin(T * j / count);
  }
  for (int j = 0; j < count; j++) cm[j] = trig::cos(T * (j + 0.5) / count), sm[j] = trig::sin(T * (j + 0.5) / count);
  // A point of the profile at step j round (on the axis, one point whatever the step).
  auto at = [&](double r, double z, int j) { return r == 0 ? V3{0, 0, z} : V3{r * cs[j], r * sn[j], z}; };
  size_t nl = m.outline.size();
  std::vector<std::vector<std::vector<double>>> steps(nl);
  auto least = fewestChords(m.outline);
  for (size_t l = 0; l < nl; l++)
    for (size_t k = 0; k < m.outline[l].size(); k++) steps[l].push_back(ringsOf(m.outline[l][k], d, more, least[l][k]));
  // A part turn's flat ends: a face each per region, at angle 0 and at the turn.
  int regions = whole ? 0 : regionCount(m);
  std::vector<std::array<int, 2>> capFace(regions, {-1, -1});
  for (int g = 0; g < regions; g++) {
    std::vector<Loop2> mine;
    for (size_t l = 0; l < nl; l++) {
      if (m.region[l] != g) continue;
      Loop2 q;
      for (size_t k = 0; k < m.outline[l].size(); k++) {
        const auto &ts = steps[l][k];
        for (size_t i = 0; i + 1 < ts.size(); i++) {
          double r, z;
          m.outline[l][k].at(ts[i], r, z);
          q.x.push_back(r), q.y.push_back(z);
        }
      }
      mine.push_back(std::move(q));
    }
    std::vector<const Loop2 *> refs;
    std::vector<V3> pts;
    for (const Loop2 &q : mine) {
      refs.push_back(&q);
      for (size_t j = 0; j < q.x.size(); j++) pts.push_back({q.x[j], 0, q.y[j]});
    }
    std::vector<int> tris;
    if (refs.empty() || !regionCap(refs, tris)) return false;
    for (int end = 0; end < 2; end++) {
      int j = end ? count : 0;
      V3 nrm = end ? V3{-sn[count], cs[count], 0} : V3{0, -1, 0};
      int f = (int)out.faces.size();
      Solid::Face face{nrm, {}, {}};
      face.geom.kind = FaceGeom::Flat, face.geom.flat = true, face.geom.pn = nrm, face.geom.pd = 0;
      out.faces.push_back(face);
      capFace[g][end] = f;
      uint32_t base = (uint32_t)out.p.size();
      for (V3 q : pts) out.vertex(at(q.x, q.z, j), nrm);
      for (size_t t = 0; t + 2 < tris.size(); t += 3) {
        uint32_t a = base + tris[t], b = base + tris[t + 1], c = base + tris[t + 2];
        if (end) out.triangle(a, c, b, f);
        else out.triangle(a, b, c, f);
      }
    }
  }
  Affine far;
  far.m[0] = cs[count], far.m[1] = -sn[count], far.m[4] = sn[count], far.m[5] = cs[count];
  for (size_t l = 0; l < nl; l++) {
    const auto &loop = m.outline[l];
    const size_t ne = loop.size();
    int g = m.region[l];
    std::vector<int> faceOf(ne, -1);
    for (size_t k = 0; k < ne; k++) {
      const Elem &e = loop[k];
      const auto &ts = steps[l][k];
      int pieces = (int)ts.size() - 1;
      if (e.onAxis()) {
        // Along the axis, where the two flat ends meet.
        if (!whole) {
          Solid::Edge axis;
          axis.pts = {{0, 0, e.z0}, {0, 0, e.z1}};
          axis.geom.kind = EdgeGeom::Line;
          sides(axis, capFace[g][0], capFace[g][1]);
          out.edges.push_back(axis);
        }
        continue;
      }
      int f = (int)out.faces.size();
      faceOf[k] = f;
      double nr, nz;
      e.normalAt(0.5, nr, nz);
      Solid::Face face{whole ? V3{-nr, 0, nz} : V3{nr * trig::cos(T / 2), nr * trig::sin(T / 2), nz}, {}, {}};
      face.geom.kind = FaceGeom::Turned, face.geom.elem = e;
      if (e.flat()) face.geom.flat = true, face.geom.pn = {0, 0, nz > 0 ? 1.0 : -1.0}, face.geom.pd = nz > 0 ? e.z0 : -e.z0;
      // What the mesh misses: the exact volume this piece turns round, less the polygon-sided one its chords turn round.
      double meshed = 0;
      for (int i = 0; i < pieces; i++) {
        double r0, z0, r1, z1;
        e.at(ts[i], r0, z0);
        e.at(ts[i + 1], r1, z1);
        meshed += moment(Elem::line(r0, z0, r1, z1));
      }
      face.deficit = T * moment(e) - meshed * count * trig::sin(T / count);
      out.faces.push_back(face);
      // Each ring's first vertex (a pole has one, or one per column where the surface comes to a point at an angle).
      std::vector<uint32_t> ring(pieces + 1);
      std::vector<char> pole(pieces + 1), fan(pieces + 1);
      for (int i = 0; i <= pieces; i++) {
        double r, z;
        e.at(ts[i], r, z);
        e.normalAt(ts[i], nr, nz);
        ring[i] = (uint32_t)out.p.size();
        if (r == 0) {
          pole[i] = 1;
          if (std::fabs(nr) < 1e-12) {
            out.vertex({0, 0, z}, {0, 0, nz > 0 ? 1.0 : -1.0});
          } else {
            fan[i] = 1;
            for (int j = 0; j < count; j++) out.vertex({0, 0, z}, {nr * cm[j], nr * sm[j], nz});
          }
        } else {
          for (int j = 0; j < cols; j++) out.vertex({r * cs[j], r * sn[j], z}, {nr * cs[j], nr * sn[j], nz});
        }
      }
      for (int i = 0; i < pieces; i++) {
        if (pole[i] && pole[i + 1]) continue;
        for (int j = 0; j < count; j++) {
          int jn = whole ? (j + 1) % count : j + 1;
          uint32_t A = ring[i] + (pole[i] ? (fan[i] ? j : 0) : j), B = ring[i] + (pole[i] ? (fan[i] ? j : 0) : jn);
          uint32_t C = ring[i + 1] + (pole[i + 1] ? (fan[i + 1] ? j : 0) : jn), D = ring[i + 1] + (pole[i + 1] ? (fan[i + 1] ? j : 0) : j);
          if (pole[i]) {
            out.triangle(A, C, D, f);
          } else if (pole[i + 1]) {
            out.triangle(A, B, C, f);
          } else {
            out.triangle(A, B, C, f);
            out.triangle(A, C, D, f);
          }
        }
      }
      if (whole) {
        // The seam, where the turned face meets itself (none on a flat face).
        if (!e.flat()) {
          Solid::Edge seam;
          for (int i = 0; i <= pieces; i++) {
            double r, z;
            e.at(ts[i], r, z);
            seam.pts.push_back(at(r, z, 0));
          }
          seam.f0 = f;
          seam.geom.kind = EdgeGeom::Profile, seam.geom.elem = e;
          out.edges.push_back(seam);
        }
      } else {
        // Where it meets each flat end: the piece itself in the end's plane.
        for (int end = 0; end < 2; end++) {
          Solid::Edge side;
          for (int i = 0; i <= pieces; i++) {
            double r, z;
            e.at(ts[i], r, z);
            side.pts.push_back(at(r, z, end ? count : 0));
          }
          sides(side, f, capFace[g][end]);
          side.geom.kind = EdgeGeom::Profile, side.geom.elem = e;
          if (end) side.geom.place = far;
          out.edges.push_back(side);
        }
      }
      if (e.arc && std::fabs(e.a1 - e.a0) >= pi - 1e-6) {
        out.circles.push_back({{e.cr, 0, e.cz}, {0, -1, 0}, e.rad});
        if (!whole) out.circles.push_back({at(e.cr, e.cz, count), {-sn[count], cs[count], 0}, e.rad});
      }
    }
    // Where pieces meet: a circle (or an arc of one) off the axis, a point on it (with a corner where the surface comes to
    // a point).
    for (size_t k = 0; k < ne; k++) {
      size_t next = (k + 1) % ne;
      const Elem &e = loop[k];
      double r = e.r1, z = e.z1;
      if (r > 0) {
        Solid::Edge c;
        c.pts.reserve(count + 1);
        for (int j = 0; j <= count; j++) c.pts.push_back(at(r, z, whole ? j % count : j));
        sides(c, faceOf[k], faceOf[next]);
        c.geom.kind = EdgeGeom::Circle, c.geom.r = r, c.geom.z = z;
        out.edges.push_back(c);
        if (whole || T >= pi - 1e-6) out.circles.push_back({{0, 0, z}, {0, 0, 1}, r});
        out.corners.push_back(at(r, z, 0));
        if (!whole) out.corners.push_back(at(r, z, count));
      } else {
        for (size_t q : {k, next}) {
          const Elem &x = loop[q];
          if (x.onAxis() || x.flat()) continue;
          Solid::Edge point;
          point.f0 = faceOf[q];
          out.edges.push_back(point);
          out.corners.push_back({0, 0, z});
          break;
        }
      }
    }
  }
  return true;
}

void buildRevolved(const Model &m, Solid &out, double d) {
  for (int more = 1; more <= 16; more *= 2) {
    out = Solid();
    if (revolveAt(m, out, d, more)) return;
  }
  out = Solid();
}

}  // namespace

double pieceArea(const Elem &e) {
  double a = (e.r0 * e.z1 - e.r1 * e.z0) / 2;
  if (e.arc) {
    double t = e.a1 - e.a0;
    a += e.rad * e.rad / 2 * (t - trig::sin(t));
  }
  return a;
}

std::shared_ptr<Model> extrudedModel(std::vector<std::vector<Elem>> loops, std::vector<int> region, double lo, double hi) {
  auto m = std::make_shared<Model>();
  m->kind = Model::Extruded;
  if (region.size() != loops.size()) region.assign(loops.size(), 0);
  double v = 0;
  for (auto &loop : loops) {
    closeUp(loop);
    for (const Elem &e : loop) v += pieceArea(e);
  }
  m->outline = std::move(loops), m->region = std::move(region), m->lo = lo, m->hi = hi;
  m->volume = std::fabs(v) * (hi - lo);
  return m;
}

std::shared_ptr<Model> revolvedModel(std::vector<std::vector<Elem>> loops, std::vector<int> region, double turn) {
  auto m = std::make_shared<Model>();
  m->kind = Model::Revolved;
  if (region.size() != loops.size()) region.assign(loops.size(), 0);
  double v = 0;
  for (auto &loop : loops) {
    closeUp(loop);
    for (const Elem &e : loop) v += moment(e);
  }
  m->outline = std::move(loops), m->region = std::move(region);
  m->turn = turn >= 2 * pi - 1e-12 ? 2 * pi : turn;
  m->volume = std::fabs(v) * m->turn;
  return m;
}

}  // namespace bce
