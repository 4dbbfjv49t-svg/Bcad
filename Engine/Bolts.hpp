// Bolts and nuts in Bcad's own engine (Bolts.cpp): bodies round their axis (Radial.hpp), sized as Fasteners.cpp fits them.
#pragma once
#include "BcadKernel.h"
#include "Engine/Model.hpp"

#include <string>

namespace bce {

struct RadialSpec;

// A bolt's or nut's body built up from z = 0 (its sizes already fitted), and how tall it is; false (with why) if it
// can't be made.
bool fastenerSpec(const BKFastener &f, double clearance, RadialSpec &s, double &height, std::string &why);
// The shape bk_fastener gives: the body centred on its box, meshed here at the details the app asks most (kept for it),
// so one that couldn't be made is found now, not later.
bool fastener(const BKFastener &f, double clearance, Shape &out, std::string &why);

}  // namespace bce
