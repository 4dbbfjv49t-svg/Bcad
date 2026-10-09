// Bcad's geometry engine: sketches — points in a plane, lines, arcs and circles through them, and rules on them
// (constraints and dimensions, Solve.cpp); the regions the curves bound; and solids made of chosen regions, stood up
// (extruded) or turned (revolved).
#pragma once
#include "Engine/Model.hpp"

#include <string>
#include <vector>

namespace bce {

// As BcadKernel.h's BKCurve and BKRule.
struct SketchCurve {
  int kind = 0;
  int p[3] = {-1, -1, -1};
  double radius = 0;
  int flags = 0;
};
struct SketchRule {
  int kind = 0;
  int p[3] = {-1, -1, -1};
  int c[2] = {-1, -1};
  double value = 0;
  int side = 0;
};
struct Sketch {
  std::vector<V3> points;  // (x, y, 0)
  std::vector<unsigned char> fixed;
  std::vector<SketchCurve> curves;
  std::vector<SketchRule> rules;
};

// What a sketch may hold: counts, indices in range, sizes numbers and at most 100 m out. `why` says what's wrong.
bool sketchValid(const Sketch &s, std::string &why);

// A region the curves bound: its area, a point inside it, its sides (sorted, as bk_sketch_regions says), its outline then
// holes as closed polylines, and triangles covering it.
struct SketchRegion {
  double area = 0;
  V3 seed;
  std::vector<int> sides;
  std::vector<std::vector<V3>> loops;
  std::vector<V3> triangles;
};
bool sketchRegions(const Sketch &s, double deflection, std::vector<SketchRegion> &out, std::string &why);

// A region chosen earlier, by its sides and a point that was inside it.
struct RegionRef {
  std::vector<int> sides;
  V3 seed;
};
// Each one's region now (-1 where it's gone).
bool sketchMatch(const Sketch &s, const std::vector<RegionRef> &refs, std::vector<int> &out, std::string &why);

// Stood up from low to high along z (kind 0), or turned about line curve `axis` from low to high degrees (kind 1).
struct SketchForm {
  int kind = 0;
  double low = 0, high = 0;
  int axis = -1;
};
// The solid made of the chosen regions, in the sketch's own coordinates.
bool sketchShape(const Sketch &s, const std::vector<RegionRef> &refs, const SketchForm &form, Shape &out, std::string &why);

// The rules made to hold, the points and circles' radii moved as little as they can (the points in `drag` towards
// `targets` first, as a hand dragging them). freedom: how many ways the sketch can still move; dependent: the first rule
// that holds nothing the others don't (-1 none) — conflicting when it can't hold with them; per point and curve, whether
// the rules hold it where it is.
struct SolveReport {
  bool solved = false;
  int freedom = 0, dependent = -1;
  bool conflicting = false;
  double residual = 0;
  std::vector<unsigned char> pointFixed, curveFixed;
};
bool solveSketch(Sketch &s, const std::vector<int> &drag, const std::vector<V3> &targets, SolveReport &r, std::string &why);
// (For tests: the most each rule's worked-out derivatives differ from finite differences, relatively.)
double sketchJacobianError(const Sketch &s);

}  // namespace bce
