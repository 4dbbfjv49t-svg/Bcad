// STEP files of Bcad's own shapes (Step.cpp).
#pragma once
#include "Engine/Model.hpp"

#include <string>
#include <vector>

struct BKShape;

namespace bce {

// A STEP file (ISO 10303-21, AP214, millimetres) of the shapes, each a solid named as given, meshed at fileDeflection
// (Print.hpp), as text: false where a shape isn't a closed solid, with `why` saying how (as shells() does) and `which`
// (if given) which shape. `file` is the name its header gives.
bool stepText(const std::vector<Shape> &shapes, const std::vector<std::string> &names, const std::string &file, std::string &out,
              std::string &why, size_t *which = nullptr);

// The shape a C API handle holds (Engine.cpp).
const Shape &heldShape(const BKShape *s);

}  // namespace bce
