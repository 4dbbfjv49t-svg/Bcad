// Bolts and nuts as Bcad's engine makes them: each shape told as radii round the axis
// (Radial.hpp), so each is meshed in one go, closed by how it's made. The sizes, and how they fit, are Fasteners.cpp's.
#include "Engine/Bolts.hpp"

#include "Engine/Fasteners.hpp"
#include "Engine/Radial.hpp"

#include <algorithm>

namespace bce {

namespace {

constexpr double twoPi = 2 * M_PI;

double wrap(double t) { return t - twoPi * std::floor(t / twoPi); }

struct Maker {
  RadialSpec &s;

  int face(const FaceGeom &g) {
    s.faces.push_back(g);
    return (int)s.faces.size() - 1;
  }
  int curved() { return face(FaceGeom{}); }
  // A cone or cylinder's face: the stretch of r = a + bz·z where it can show (within the zone and its own heights, out to
  // the zone's reach), running up the outside (down the inside, so its normal points out of the solid either way).
  int turned(const Atom &a, const Zone &z, bool inside) {
    double z0 = std::max(z.zb, a.lo), z1 = std::min(z.zt, a.hi);
    if (a.bz != 0) {
      double atReach = (z.reach - a.a) / a.bz, atAxis = -a.a / a.bz;
      z0 = std::max(z0, std::min(atReach, atAxis)), z1 = std::min(z1, std::max(atReach, atAxis));
    }
    if (!(z0 < z1)) return curved();
    FaceGeom g;
    g.kind = FaceGeom::Turned;
    Elem e = Elem::line(a.a + a.bz * z0, z0, a.a + a.bz * z1, z1);
    g.elem = inside ? Elem::line(e.r1, e.z1, e.r0, e.z0) : e;
    return face(g);
  }
  // A plane along the axis (or leaning): its exact plane, facing out of the solid.
  int wall(const Atom &a, bool inside) {
    V3 n{trig::cos(a.phi), trig::sin(a.phi), -a.d1};
    double l = norm(n), k = inside ? -1 : 1;
    FaceGeom g;
    g.kind = FaceGeom::Flat, g.flat = true, g.pn = n * (k / l), g.pd = k * a.d0 / l;
    return face(g);
  }
};

Atom linear(double a, double bz, double lo = -INFINITY, double hi = INFINITY) {
  Atom x;
  x.kind = Atom::Linear, x.a = a, x.bz = bz, x.lo = lo, x.hi = hi;
  return x;
}

// A star-shaped outline round the axis (corners by angle, counter-clockwise; upright walls between): each wall a sector.
void outline(Maker &mk, Zone &z, const std::vector<V3> &corners, bool inside) {
  size_t n = corners.size();
  for (size_t i = 0; i < n; i++) {
    V3 a = corners[i], b = corners[(i + 1) % n], e = b - a;
    V3 out = unit(V3{e.y, -e.x, 0});
    Atom w;
    w.kind = Atom::Wall, w.d0 = dot(a, out), w.phi = trig::atan2(out.y, out.x);
    w.face = mk.wall(w, inside);
    Zone::Sector s;
    s.t0 = wrap(trig::atan2(a.y, a.x));
    s.atoms.push_back(w);
    z.sectors.push_back(s);
    z.reach = std::max(z.reach, trig::hypot(a.x, a.y));
  }
  std::sort(z.sectors.begin(), z.sectors.end(), [](const Zone::Sector &x, const Zone::Sector &y) { return x.t0 < y.t0; });
}

// n corners round a circle of radius r, the first at angle `first`.
std::vector<V3> ngon(int n, double r, double first) {
  std::vector<V3> v;
  for (int i = 0; i < n; i++) v.push_back({r * trig::cos(first + twoPi * i / n), r * trig::sin(first + twoPi * i / n), 0});
  return v;
}

// Two hexagons across flats s turned 30° to each other: tips at s/√3, valleys between.
std::vector<V3> twelvePoint(double s) {
  std::vector<V3> v;
  double tip = s / std::sqrt(3.0), valley = s / 2 / trig::cos(M_PI / 12);
  for (int i = 0; i < 12; i++) {
    double a = i * M_PI / 6;
    v.push_back({tip * trig::cos(a), tip * trig::sin(a), 0});
    v.push_back({valley * trig::cos(a + M_PI / 12), valley * trig::sin(a + M_PI / 12), 0});
  }
  return v;
}

// A block's corners taken off at 30° from a flat end of radius r0 (rc reaching the outline's corners): on a thin block
// they start further out, so each takes at most 0.4 of its height (0.8 when only one end is chamfered).
void chamfers(Maker &mk, Zone &z, double r0, double rc, bool bottom, bool top) {
  double h = z.zt - z.zb, slope = trig::tan(M_PI / 6), most = (bottom && top ? 0.4 : 0.8) * h;
  r0 = std::max(r0, rc - most / slope);
  if (top) {
    Atom a = linear(r0 + z.zt / slope, -1 / slope);
    a.face = mk.turned(a, z, false);
    z.cuts.push_back(a);
  }
  if (bottom) {
    Atom a = linear(r0 - z.zb / slope, 1 / slope);
    a.face = mk.turned(a, z, false);
    z.cuts.push_back(a);
  }
}

Zone threaded(Maker &mk, double rMaj, double rMin, double p, double z0, double zb, double zt) {
  Zone z;
  z.zb = zb, z.zt = zt, z.threaded = true, z.reach = rMaj;
  z.thread.p = p, z.thread.z0 = z0, z.thread.rMaj = rMaj, z.thread.rMin = rMin;
  for (int i = 0; i < 4; i++) z.thread.face[i] = mk.curved();
  return z;
}

// A Torx outline (ISO 10664): six round lobes a across, six round valleys b across, each valley touching the lobes
// beside it.
void torx(Maker &mk, Zone &z, double a, double b) {
  double re = 0.1 * a, ro = a / 2 - re, h = b / 2, c30 = trig::cos(M_PI / 6);
  double ri = (ro * ro + h * h - 2 * ro * c30 * h - re * re) / (2 * re + 2 * ro * c30 - 2 * h);
  auto at = [](double r, double t) { return V3{r * trig::cos(t), r * trig::sin(t), 0}; };
  for (int i = 0; i < 6; i++) {
    double a0 = i * M_PI / 3;
    V3 co = at(ro, a0), cb = at(h + ri, a0 - M_PI / 6), ca = at(h + ri, a0 + M_PI / 6);
    V3 before = co + (cb - co) * (re / (re + ri)), after = co + (ca - co) * (re / (re + ri));
    Atom lobe, valley;
    lobe.kind = valley.kind = Atom::Arc;
    lobe.cx = co.x, lobe.cy = co.y, lobe.rho = re, lobe.side = 1, lobe.face = mk.curved();
    valley.cx = ca.x, valley.cy = ca.y, valley.rho = ri, valley.side = -1, valley.face = mk.curved();
    Zone::Sector s1, s2;
    s1.t0 = wrap(trig::atan2(before.y, before.x)), s1.atoms.push_back(lobe);
    s2.t0 = wrap(trig::atan2(after.y, after.x)), s2.atoms.push_back(valley);
    z.sectors.push_back(s1), z.sectors.push_back(s2);
  }
  z.reach = a / 2;
  std::sort(z.sectors.begin(), z.sectors.end(), [](const Zone::Sector &x, const Zone::Sector &y) { return x.t0 < y.t0; });
}

// A Phillips recess cut down from `top`: two crossed slots w wide, their ends sloping in from the recess diameter m at
// the top to w at the bottom (a w × w floor), and a cone opening up the middle.
void phillips(Maker &mk, Zone &z, int ph, double m, double depth, double top) {
  double w = phWing[ph], slope = (m - w) / 2 / depth, half = m / 2 + slope, floor = top - depth, k = (half - w / 2) / (depth + 1);
  for (int arm = 0; arm < 4; arm++) {
    double a = arm * M_PI / 2;
    Zone::Sector s;
    s.t0 = wrap(a - M_PI / 4);
    Atom end;
    end.kind = Atom::Wall, end.d0 = w / 2 - floor * k, end.d1 = k, end.phi = a, end.face = mk.wall(end, true);
    s.atoms.push_back(end);
    for (int side : {-1, 1}) {
      Atom wall;
      wall.kind = Atom::Wall, wall.d0 = w / 2, wall.phi = a + side * M_PI / 2, wall.face = mk.wall(wall, true);
      s.atoms.push_back(wall);
    }
    z.sectors.push_back(s);
  }
  std::sort(z.sectors.begin(), z.sectors.end(), [](const Zone::Sector &x, const Zone::Sector &y) { return x.t0 < y.t0; });
  double rc = 0.3 * m, drop = 0.9 * depth;
  Atom cone = linear(-rc * (top - drop) / drop, rc / drop, top - drop);
  z.reach = std::max(trig::hypot(m / 2, w / 2), rc);
  cone.face = mk.turned(cone, z, true);
  z.adds.push_back(cone);
}

}  // namespace

bool fastenerSpec(const BKFastener &f, double clearance, RadialSpec &s, double &height, std::string &why) {
  const ThreadSize &t = threadOf(f.size);
  double c = std::max(0.0, clearance), p = t.p, L = f.length;
  Maker mk{s};
  if (isNut(f.kind)) {
    // The outside from z = 0 up to its height, tapped through, the thread's ends countersunk (not at a cone nut's seat).
    double d = t.d + c, r = d / 2, w = f.width, face = 0.475 * w;
    if (f.kind == BK_SLEEVE) {
      Zone z;
      z.zb = 0, z.zt = L, z.reach = r + w;
      Atom a = linear(r + w, 0);
      a.face = mk.turned(a, z, false);
      z.sectors.push_back({0, {a}});
      s.outer.push_back(z);
      face = r + w;
    } else if (f.kind == BK_SQUARE_NUT) {
      Zone z;
      z.zb = 0, z.zt = L;
      outline(mk, z, ngon(4, w / std::sqrt(2.0), -M_PI / 4), false);
      chamfers(mk, z, face, w / std::sqrt(2.0), false, true);  // DIN 557: one side
      s.outer.push_back(z);
    } else if (f.kind == BK_CONE_NUT) {
      double narrow = 0.46 * w - f.seat * tanHalf(f.angle);
      Zone seat;
      seat.zb = 0, seat.zt = f.seat, seat.reach = 0.46 * w;
      Atom a = linear(narrow, (0.46 * w - narrow) / f.seat);
      a.face = mk.turned(a, seat, false);
      seat.sectors.push_back({0, {a}});
      s.outer.push_back(seat);
      Zone hex;
      hex.zb = f.seat, hex.zt = L;
      outline(mk, hex, ngon(6, w / std::sqrt(3.0), 0), false);
      chamfers(mk, hex, face, w / std::sqrt(3.0), true, true);
      s.outer.push_back(hex);
    } else {
      Zone z;
      z.zb = 0, z.zt = L;
      outline(mk, z, ngon(6, w / std::sqrt(3.0), 0), false);
      chamfers(mk, z, face, w / std::sqrt(3.0), true, true);
      s.outer.push_back(z);
    }
    // The tap: the thread from −1 (so the same phase), countersunk 45° down to the crest radius at each end, and on into
    // the thread a little steeper (rather than a flat ring exactly at the crest, touching it edge on).
    double sink = std::min({p * 0.6, L / 4, 0.6 * (face - r)});
    Zone tap = threaded(mk, r, r - rootDepth * p, p, -1, 0, L);
    tap.reach = r + sink;
    double zs = L - sink;
    Atom top1 = linear(r - zs, 1, zs), top2 = linear(r - 8 * zs, 8, -INFINITY, zs);
    top1.face = mk.turned(top1, tap, true), top2.face = mk.turned(top2, tap, true);
    tap.adds.push_back(top1), tap.adds.push_back(top2);
    if (f.kind != BK_CONE_NUT) {
      Atom bot1 = linear(r + sink, -1, -INFINITY, sink), bot2 = linear(r + 8 * sink, -8, sink);
      bot1.face = mk.turned(bot1, tap, true), bot2.face = mk.turned(bot2, tap, true);
      tap.adds.push_back(bot1), tap.adds.push_back(bot2);
    }
    s.inner.push_back(tap);
    height = L;
  } else {
    double d = t.d - c, rMaj = d / 2, rMin = rMaj - rootDepth * p;
    if (f.kind == BK_ROD) {
      // Thread on [0, L], chamfered 45° at both ends.
      double cc = std::min(rMaj - rMin + 0.05, L / 3);
      Zone z = threaded(mk, rMaj, rMin, p, 0, 0, L);
      Atom e0 = linear(rMaj - cc, 1), e1 = linear(rMaj - cc + L, -1);
      e0.face = mk.turned(e0, z, false), e1.face = mk.turned(e1, z, false);
      z.cuts = {e0, e1};
      s.outer.push_back(z);
      height = L;
    } else {
      // The thread from z = 0 up to the length, chamfered at its tip; a head on top, or a cone under the head reaching
      // down into the thread (countersunk: the cone is the head). The cone's narrow end lies inside the thread's core.
      bool csk = countersunk(f.kind), cone = csk || coneBelow(f.kind);
      double th = tanHalf(f.angle), rTop = coneTop(f), core = 0.9 * (d / 2 - rootDepth * p);
      double drop = cone ? (rTop - d / 2) / th : 0, rod = cone ? L - drop + p / 2 : L;
      double cc = std::min(rMaj - rMin + 0.05, rod / 3);
      Zone z = threaded(mk, rMaj, rMin, p, 0, 0, L);
      Atom tip = linear(rMaj - cc, 1);
      tip.face = mk.turned(tip, z, false);
      z.cuts.push_back(tip);
      if (cone) {
        double low = L - (rTop - core) / th;
        z.reach = std::max(rMaj, rTop);
        Atom a = linear(core - low * th, th, low);
        a.face = mk.turned(a, z, false);
        z.adds.push_back(a);
      }
      s.outer.push_back(z);
      double top = L;
      if (!csk) {
        double w = f.width, k = f.height;
        Zone head;
        head.zb = L, head.zt = L + k;
        bool below = coneBelow(f.kind);
        switch (f.kind) {
        case BK_TWELVE: case BK_TWELVE_CONE:
          outline(mk, head, twelvePoint(w), false);
          chamfers(mk, head, 0.475 * w, w / std::sqrt(3.0), below, true);
          break;
        case BK_SOCKET: case BK_TORX: {
          // Round, its top rim chamfered 45°.
          double ch = std::min({0.08 * w, 0.3 * k, 0.4 * (w - driveSpan(f))});
          head.reach = w / 2;
          Atom side = linear(w / 2, 0);
          side.face = mk.turned(side, head, false);
          head.sectors.push_back({0, {side}});
          Atom rim = linear(w / 2 + L + k - ch, -1);
          rim.face = mk.turned(rim, head, false);
          head.cuts.push_back(rim);
          break;
        }
        default:
          outline(mk, head, ngon(6, w / std::sqrt(3.0), 0), false);
          chamfers(mk, head, 0.475 * w, w / std::sqrt(3.0), below, true);
        }
        s.outer.push_back(head);
        top = L + k;
      }
      height = top;
      // The drive, cut down from the top.
      if (keyDrive(f.kind) || torxDrive(f.kind) || phillipsDrive(f.kind)) {
        Zone drive;
        drive.zb = top - f.depth, drive.zt = top;
        if (keyDrive(f.kind)) {
          outline(mk, drive, ngon(6, f.drive / std::sqrt(3.0), 0), true);
        } else if (torxDrive(f.kind)) {
          const TorxSize *x = torxOf(f.drive);
          if (!x) return why = "unknown Torx size", false;
          torx(mk, drive, x->a, x->b);
        } else {
          phillips(mk, drive, (int)lround(f.drive), f.recess, f.depth, top);
        }
        s.inner.push_back(drive);
      }
    }
  }
  s.volume = radialVolume(s);
  s.wall = radialWall(s);
  if (!(s.volume > 0)) return why = "nothing left", false;
  return true;
}

}  // namespace bce

namespace bce {

bool fastener(const BKFastener &f, double clearance, Shape &out, std::string &why) {
  auto spec = std::make_shared<RadialSpec>();
  double height = 0;
  if (!fastenerSpec(f, clearance, *spec, height, why)) return false;
  auto m = std::make_shared<Model>();
  m->kind = Model::Radial, m->radial = spec, m->volume = spec->volume;
  out = shapeOf(m, Affine::translation(V3{0, 0, -height / 2}));
  for (double d : {0.05, 0.01}) {
    auto solid = std::make_shared<Solid>();
    if (!radialMesh(*spec, d, *solid, why)) return false;
    solid->centroids();
    solid->slivers();
    std::lock_guard<std::mutex> hold(out.node->lock);
    out.node->made.push_back({d, solid});
  }
  return true;
}

}  // namespace bce
