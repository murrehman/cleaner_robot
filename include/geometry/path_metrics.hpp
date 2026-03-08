#pragma once

#include "geometry/transforms.hpp"
#include <vector>

namespace robot_cleaner {

// Computes total path length
double compute_total_path_length(const Path& path);

// Computes traversal time given a path and its curvature profile
double compute_traversal_time(const Path& path, const std::vector<double>& curvature_profile);

// Calculate velocity given a curvature
double computeVelocity(double curvature);

} // namespace robot_cleaner
