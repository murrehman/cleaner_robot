#pragma once

#include "geometry/transforms.hpp"
#include <vector>

namespace robot_cleaner {

// Computes the total area cleaned by the gadget sweeping along the path
double compute_cleaned_area(const Path& path, const CleaningGadget& gadget);

// Optional: Returns the combined Boost polygon representing the swept area
// for visualization purposes. We return it as a generic structure.
struct MultiPolygon {
    std::vector<std::vector<Point2D>> outer_rings; // We simplify visualizer representation
};

MultiPolygon compute_swept_polygon(const Path& path, const CleaningGadget& gadget);

} // namespace robot_cleaner
