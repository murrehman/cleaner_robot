#pragma once

#include "geometry/transforms.hpp"
#include <vector>

namespace robot_cleaner {

// Computes circumcircle curvature for 3 consecutive points
double compute_curvature(const Point2D& p1, const Point2D& p2, const Point2D& p3);

// Computes curvature profile for a given path
// For endpoints, we assume curvature is the same as the nearest neighbor
std::vector<double> compute_path_curvature(const Path& path);

} // namespace robot_cleaner
