// Sculpted bodies (Sculpt.cpp): a closed mesh made again of even triangles at a chosen detail, ready to be shaped by hand.
#pragma once
#include "Engine/Model.hpp"

#include <string>
#include <vector>

namespace bce {

// What a closed mesh encloses (wherever its winding number is above 0, so overlapping pieces and folds come out as one
// solid, and voids stay), made again as one closed mesh of triangles about `detail` apart: sampled on a grid that fine,
// its surface put where the grid's lines cross the mesh's, then evened out. The mesh needn't share its points (any
// shape's mesh will do). False with `why` when nothing's inside, or when it would take more than about `most` triangles.
bool remesh(const std::vector<V3> &pts, const std::vector<uint32_t> &tris, double detail, std::vector<V3> &outPts, std::vector<uint32_t> &outTris,
            std::string &why, size_t most = 1500000);

}  // namespace bce
