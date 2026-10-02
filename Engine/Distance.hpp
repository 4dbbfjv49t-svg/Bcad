// Bcad's geometry engine: the shortest distance between two picked elements.
#pragma once
#include "Engine/Model.hpp"

#include <string>

namespace bce {

// One end of a measurement: a point, or an edge or face of a placed shape by its number in the shape's mesh.
struct End {
  enum Kind { Point, Edge, Face } kind = Point;
  const Shape *shape = nullptr;
  Affine place;
  int index = 0;
  V3 point;
};

// The shortest distance between the two ends and the points it runs between; false with `why` when an end isn't there.
bool distance(const End &a, const End &b, double &d, V3 &pa, V3 &pb, std::string &why);

}  // namespace bce
