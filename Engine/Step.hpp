// STEP files of Bcad's own shapes (Step.cpp).
#pragma once
#include "Engine/Model.hpp"

#include <string>
#include <vector>

struct BKShape;

namespace bce {

// The chord error STEP files are written at (as fine as an STL export's).
constexpr double stepDeflection = 0.01;

// A STEP file (ISO 10303-21, AP214, millimetres) of the shapes, each a solid named as given, as text: false with `why`
// set where a shape isn't a closed solid. `file` is the name its header gives.
bool stepText(const std::vector<Shape> &shapes, const std::vector<std::string> &names, const std::string &file, std::string &out,
              std::string &why);

// The shape a C API handle holds (Engine.cpp).
const Shape &heldShape(const BKShape *s);

}  // namespace bce
