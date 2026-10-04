// Bodies round the z axis whose radius changes with the angle too: at every angle t and height z one radius outside and,
// where hollow, one inside. Bolts and nuts are all such (Bolts.cpp): a thread's radius follows its helix, a head's its
// flats, a socket's its outline. They're meshed straight from those radii: the side laid out flat as an (angle, height)
// sheet cut into cells, each cell edge shared by the cells either side and its points worked out once, so the mesh comes
// out closed by how it's made, with no merging and nothing left to chance.
#pragma once
#include "Engine/Model.hpp"

#include <string>
#include <vector>

namespace bce {

// One surface, as a radius at angle t (radians, counter-clockwise from +x) and height z.
struct Atom {
  enum Kind { Linear, Wall, Arc } kind = Linear;
  // Linear: a + bz·z (a cone, a cylinder).
  double a = 0, bz = 0;
  // Wall: a plane along the axis (leaning where d1 isn't 0) at distance d0 + d1·z from it, facing angle phi:
  // (d0 + d1·z) / cos(t − phi), where it faces the angle at all.
  double d0 = 0, d1 = 0, phi = 0;
  // Arc: an upright cylinder round (cx, cy) of radius rho: its side near the axis (−1) or away from it (+1).
  double cx = 0, cy = 0, rho = 0;
  int side = 1;
  // Only between these heights (missing elsewhere).
  double lo = -INFINITY, hi = INFINITY;
  int face = 0;
  // The radius, and how fast it changes with t and with z; false where it isn't.
  bool at(double t, double z, double &r, double &rt, double &rz) const;
};

// A thread round the axis: crest radius rMaj (p/8 wide), flanks at 60°, root radius rMin (p/4 wide), rising p a turn
// counter-clockwise (right-handed); the crest's middle at height z0 at angle 0, and so at z0 + p·t/2π at angle t.
struct Thread {
  double p = 0, z0 = 0, rMaj = 0, rMin = 0;
  int face[4] = {0, 0, 0, 0};  // crest, the flank down to the root, root, the flank back up
};

// A stretch of a surface from height zb to zt: a thread, or by sectors of angle the least of each sector's atoms; then
// cut back to the least with `cuts` (chamfers) and grown to the most with `adds` (a cone under a head, countersinks).
struct Zone {
  double zb = 0, zt = 0;
  bool threaded = false;
  Thread thread;
  struct Sector {
    double t0 = 0;  // where it starts (in [0, 2π), in order); it ends where the next starts, the last at the first's + 2π
    std::vector<Atom> atoms;
  };
  std::vector<Sector> sectors;
  std::vector<Atom> cuts, adds;
  double reach = 0;  // its largest radius
};

// A body: outside zones stacked from the bottom (each one's top the next one's bottom) and inside ones (a socket, a
// nut's thread) within them; flat ends and steps between. Faces numbered as the atoms and threads give them, with their
// exact forms; the flat ones made between zones are added after.
struct RadialSpec {
  std::vector<Zone> outer, inner;
  std::vector<FaceGeom> faces;
  double volume = 0;
  // The thinnest wall between inside and outside (radialWall): meshes are made fine enough that no chord crosses it.
  double wall = INFINITY;
};

// The mesh, every chord within `deflection` of the exact surfaces, its faces' deficits set (what the chords miss) and
// its edges, corners and circles found; false (with why) when it can't be made one closed solid.
bool radialMesh(const RadialSpec &s, double deflection, Solid &out, std::string &why);
// ∫∫ (R_out² − R_in²)/2 dt dz: the volume, exactly but for the last few digits.
double radialVolume(const RadialSpec &s);
// The thinnest wall between the inside and the outside, sampled finely round and at every height either bends.
double radialWall(const RadialSpec &s);
// The largest d · x over the body (and where). Exact across the axis and along it (*exact true); otherwise a bound a
// hair over.
double radialSupport(const RadialSpec &s, V3 d, V3 *at, bool *exact);

}  // namespace bce
