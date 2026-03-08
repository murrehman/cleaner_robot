#pragma once

#include "geometry/transforms.hpp"
#include <vector>

namespace robot_cleaner {

// Velocity model parameters
struct VelocityProfileParams {
    double vmax = 1.0;
    double vmin = 0.1;
    double k_crit = 1.0;
    double k_max = 2.0;
};

// Computes total path length
double compute_total_path_length(const Path& path);

// Computes traversal time given a path, its curvature profile, and parameters
double compute_traversal_time(const Path& path, const std::vector<double>& curvature_profile, const VelocityProfileParams& params = VelocityProfileParams{});

// Calculate velocity given a curvature
double compute_velocity(double curvature, const VelocityProfileParams& params);

} // namespace robot_cleaner
