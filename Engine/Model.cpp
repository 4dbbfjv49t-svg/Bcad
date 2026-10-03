// Bcad's geometry engine: the primitives, exactly (bounding boxes and volumes from formulas), and their meshes.
#include "Engine/Model.hpp"

#include "BcadKernel.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace bce {

namespace {

constexpr double pi = M_PI;
// No chord turns by more than this, however coarse the mesh asked for (as OpenCascade's meshes).
constexpr double maxTurn = 0.35;

// The angle a chord may span on a circle of radius r with a sagitta of at most d.
double chordAngle(double r, double d) {
  if (r <= 0) return maxTurn;
  double c = 1 - d / r;
  double a = c <= -1 ? pi : 2 * std::acos(c);
  return std::min(a, maxTurn);
}

int countFor(double span, double r, double d) { return std::max(1, (int)std::ceil(std::fabs(span) / chordAngle(r, d) - 1e-9)); }

}  // namespace

// MARK: - profile pieces

void Elem::at(double t, double &r, double &z) const {
  if (t <= 0) {
    r = r0, z = z0;
  } else if (t >= 1) {
    r = r1, z = z1;
  } else if (arc) {
    double a = a0 + (a1 - a0) * t;
    r = cr + rad * std::cos(a), z = cz + rad * std::sin(a);
  } else {
    r = r0 + (r1 - r0) * t, z = z0 + (z1 - z0) * t;
  }
  if (r < 0) r = 0;
}

void Elem::normalAt(double t, double &nr, double &nz) const {
  double dr, dz;
  if (arc) {
    double a = a0 + (a1 - a0) * t, s = a1 > a0 ? 1 : -1;
    dr = -s * std::sin(a), dz = s * std::cos(a);
  } else {
    dr = r1 - r0, dz = z1 - z0;
  }
  double l = std::hypot(dr, dz);
  nr = dz / l, nz = -dr / l;
}

double Elem::nearest(double r, double z) const {
  if (!arc) {
    double dr = r1 - r0, dz = z1 - z0, l2 = dr * dr + dz * dz;
    return l2 > 0 ? std::clamp(((r - r0) * dr + (z - z0) * dz) / l2, 0.0, 1.0) : 0;
  }
  double lo = std::min(a0, a1), hi = std::max(a0, a1);
  double ang = std::atan2(z - cz, r - cr);
  while (ang < lo) ang += 2 * pi;
  while (ang >= lo + 2 * pi) ang -= 2 * pi;
  if (ang <= hi) return (ang - a0) / (a1 - a0);
  // Outside the arc: the nearer end.
  double d0 = std::hypot(r - r0, z - z0), d1 = std::hypot(r - r1, z - z1);
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
  if (!still) grid = 0;
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
      double v = std::asin((z - e.cz) / e.rad);
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
    cs[j] = j == 0 || j == count ? 1 : std::cos(a), sn[j] = j == 0 || j == count ? 0 : std::sin(a);
  }
  for (int j = 0; j < count; j++) cm[j] = std::cos(2 * pi * (j + 0.5) / count), sm[j] = std::sin(2 * pi * (j + 0.5) / count);

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
    face.deficit = 2 * pi * (moment(e) - meshed * count * std::sin(2 * pi / count) / (2 * pi));
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
    double x = a * std::cos(t), y = b * std::sin(t);
    return {c * x - s * y, s * x + c * y, 0};
  }
  V3 velocity(double t) const {
    double x = -a * std::sin(t), y = b * std::cos(t);
    return {c * x - s * y, s * x + c * y, 0};
  }
  V3 normal(double t) const {
    double x = b * std::cos(t), y = a * std::sin(t), l = std::hypot(x, y);
    return {(c * x - s * y) / l, (s * x + c * y) / l, 0};
  }
  // Radius of curvature.
  double bend(double t) const {
    double v = norm(velocity(t));
    return v * v * v / (a * b);
  }
};

void buildSwept(const Model &m, Solid &out, double d) {
  Oval o{m.a, m.b, std::cos(m.phi), std::sin(m.phi)};
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
        u = e.cr + e.rad * std::cos(a), z = e.cz + e.rad * std::sin(a);
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
  if (kind == Poly) buildPoly(*this, out);
  if (kind == Turned) buildTurned(*this, out, d);
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

double Model::support(V3 d, V3 *at) const {
  double best = -INFINITY;
  V3 where;
  if (kind == Poly) {
    for (const auto &v : verts)
      if (dot(v, d) > best) best = dot(v, d), where = v;
  } else if (kind == Turned) {
    double D = std::hypot(d.x, d.y), dz = d.z, c = D > 0 ? d.x / D : 1, sn = D > 0 ? d.y / D : 0, br = 0, bz = 0;
    auto take = [&](double r, double z) {
      if (r * D + z * dz > best) best = r * D + z * dz, br = r, bz = z;
    };
    for (const auto &e : profile) {
      take(e.r0, e.z0), take(e.r1, e.z1);
      if (e.arc) {
        double a = std::atan2(dz, D), lo = std::min(e.a0, e.a1), hi = std::max(e.a0, e.a1);
        while (a < lo) a += 2 * pi;
        while (a >= lo + 2 * pi) a -= 2 * pi;
        if (a <= hi) take(e.cr + e.rad * std::cos(a), e.cz + e.rad * std::sin(a));
      }
    }
    where = {br * c, br * sn, bz};
  } else {
    // Swept: the best along the oval, sampled and then narrowed down.
    Oval o{a, b, std::cos(phi), std::sin(phi)};
    auto value = [&](double t, V3 *pt) {
      V3 c = o.at(t), nm = o.normal(t);
      double s = dot(nm, d), m = -INFINITY, bu = 0, bz = 0;
      for (const auto &e : section) {
        if (e.r0 * s + e.z0 * d.z > m) m = e.r0 * s + e.z0 * d.z, bu = e.r0, bz = e.z0;
        if (e.r1 * s + e.z1 * d.z > m) m = e.r1 * s + e.z1 * d.z, bu = e.r1, bz = e.z1;
        double h = std::hypot(s, d.z);
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
    double s = std::sin(t);
    return R / 2 * (c * c * s + 2 * c * R * (t / 2 + std::sin(2 * t) / 4) + R * R * (s - s * s * s / 3));
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
    pts.push_back({r * std::cos(a), r * std::sin(a), z});
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
  double sxx = a * a + b * b * std::cos(t) * std::cos(t), sxy = b * b * std::cos(t) * std::sin(t), syy = b * b * std::sin(t) * std::sin(t);
  double mean = (sxx + syy) / 2, spread = std::hypot((sxx - syy) / 2, sxy);
  major = std::sqrt(mean + spread), minor = std::sqrt(std::max(mean - spread, 0.0)), phi = std::atan2(2 * sxy, sxx - syy) / 2;
}

double ellipseLength(double a, double b) {
  // The trapezoid rule is exact to rounding for a smooth periodic integrand with enough points.
  const int n = 1024;
  double sum = 0;
  for (int i = 0; i < n; i++) {
    double t = 2 * pi * i / n;
    sum += std::hypot(a * std::sin(t), b * std::cos(t));
  }
  return sum * 2 * pi / n;
}

Affine turnZ(double phi) {
  Affine r;
  double c = std::cos(phi), s = std::sin(phi);
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
  for (int i : above[kind])
    if (p[i] < 0.001) return fail("sizes must be above zero");
  if (kind == BK_CONE && std::max(p[0], p[1]) < 0.001) return fail("a cone needs one end wider than zero");

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

}  // namespace bce
