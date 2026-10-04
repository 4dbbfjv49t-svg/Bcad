// Exact points for merging meshes. The shapes' own points sit on a grid (whole numbers of grid steps); the points made
// where they cut each other are kept as what they're made from (an edge and a plane, or three planes) rather than
// rounded to the nearest double. Every question about them (which side of a line or a plane, which comes first along a
// line, whether two are one) is answered exactly: in doubles where the answer can't be wrong, otherwise in whole
// numbers wide enough to hold every digit.
#pragma once
#include "Engine/Math.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace bce {

// A whole number of up to 2048 bits, with no allocation: wide enough for every exact test below on points up to 2^60
// grid steps from the origin (the widest, a ray from the middle of three points made from planes, needs under 1400).
struct Big {
  static constexpr int N = 32;
  uint64_t m[N];  // its size, lowest place first
  int n = 0;      // places in use
  int s = 0;      // its sign: −1, 0 or 1
  Big() {}
  Big(int64_t v);
  Big(__int128 v);
  int sign() const { return s; }
  Big operator-() const {
    Big r = *this;
    r.s = -r.s;
    return r;
  }
  // The nearest double, to within three units in its last place.
  double approx() const;
};
Big operator+(const Big &a, const Big &b);
Big operator-(const Big &a, const Big &b);
Big operator*(const Big &a, const Big &b);
// Sums that came out too wide to hold (never expected: counted, for the tests).
extern thread_local long bigOverflows;

// A plane through three grid points (by number). With `lift` 0, 1 or 2 the third is the first moved one step along
// that axis instead: a plane standing on the line through the first two, across the plane they lie in.
struct Plane3 {
  uint32_t a = 0, b = 0, c = 0;
  int lift = -1;
};

class ExactPoints {
 public:
  explicit ExactPoints(double step = 1) : step(step) {}
  double step;
  // Every point: the grid's exactly, a made one as the nearest double.
  std::vector<V3> at;

  // A grid point (its coordinates whole numbers of steps).
  uint32_t grid(V3 p);
  // Where the line through grid points p and q meets a plane (never along it).
  uint32_t line(uint32_t p, uint32_t q, const Plane3 &pl);
  // Where three planes meet (at one point).
  uint32_t planes(const Plane3 &a, const Plane3 &b, const Plane3 &c);
  // The middle of three points (any).
  uint32_t middle(uint32_t a, uint32_t b, uint32_t c);
  size_t size() const { return at.size(); }
  // How far `at[i]` may be off the exact point, in each coordinate.
  double error(uint32_t i) const { return err[i]; }
  bool isGrid(uint32_t i) const { return def[i].kind == Grid; }

  // Seen from the side the plane through grid points a, b, c faces (counter-clockwise), whether d lies below it (1),
  // on it (0) or above (−1): orient3d's sign.
  int orient3d(uint32_t a, uint32_t b, uint32_t c, uint32_t d) const;
  // a, b, c turning left (1), in a line (0) or right (−1), seen along `axis` (with the plane's other two coordinates
  // swapped by `swap`): as Flat in Boolean.cpp.
  int orient2d(int axis, bool swap, uint32_t a, uint32_t b, uint32_t c) const;
  // a's coordinate k against b's: −1, 0, 1.
  int compare(int k, uint32_t a, uint32_t b) const;
  // By x, then y, then z: along any line, its points in order (one way or the other); 0 when one point.
  int lex(uint32_t a, uint32_t b) const;
  // The plane of grid points a, b, c seen flat: the axis its normal lies most along, and whether it faces down it.
  void flat(uint32_t a, uint32_t b, uint32_t c, int &axis, bool &swap) const;
  // The sign of ((b − a) × (c − a)) · d: whether direction d runs the way the plane of grid points a, b, c faces.
  int facing(uint32_t a, uint32_t b, uint32_t c, const int64_t d[3]) const;
  // Which side of the edge a → b (grid points) the line through q along d passes: the sign of det(a − q, b − q, d).
  // A line passes through a triangle where it passes all three edges on one side.
  int passes(uint32_t q, const int64_t d[3], uint32_t a, uint32_t b) const;

 private:
  enum Kind : uint8_t { Grid, Line, Planes, Middle };
  struct Def {
    Kind kind;
    int8_t lift[3];
    uint32_t v[9];
  };
  struct Hom {
    Big x[3], w;  // the point is x / w
  };
  std::vector<Def> def;
  std::vector<double> err;                 // how far `at` may be off, in each coordinate (0: it's exact)
  std::vector<std::array<int64_t, 3>> I;  // grid points in steps (made ones: unused)
  // Made points' exact coordinates once worked out (only those an answer in doubles couldn't settle), by number.
  mutable std::vector<int> held;
  mutable std::vector<Hom> homs;

  std::array<int64_t, 3> coords(uint32_t i, int lift) const;
  void normal(uint32_t a, uint32_t b, uint32_t c, int lift, __int128 n[3], std::array<int64_t, 3> &at0) const;
  Hom hom(uint32_t i) const;
  Hom make(uint32_t i) const;
  uint32_t add(const Def &d);
};

}  // namespace bce
