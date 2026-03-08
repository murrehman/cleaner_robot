#include "geometry/curvature.hpp"
#include <cmath>
#include <limits>

namespace robot_cleaner {

double compute_curvature(const Point2D& p1, const Point2D& p2, const Point2D& p3) {
    double a = distance(p1, p2);
    double b = distance(p2, p3);
    double c = distance(p3, p1);

    // Compute semi-perimeter
    double s = (a + b + c) / 2.0;
    
    // Compute area using Heron's formula
    // area_squared can be negative due to precision issues when points are collinear
    double area_squared = s * (s - a) * (s - b) * (s - c);
    
    if (area_squared <= 0.0) {
        return 0.0; // Collinear points have 0 curvature
    }
    
    double area = std::sqrt(area_squared);
    
    // Check for nearly division by zero
    double denom = a * b * c;
    if (denom < 1e-9) {
        return 0.0; // Points are too close
    }

    return (4.0 * area) / denom;
}

std::vector<double> compute_path_curvature(const Path& path) {
    std::vector<double> curvatures;
    if (path.points.size() < 3) {
        curvatures.assign(path.points.size(), 0.0);
        return curvatures;
    }

    curvatures.resize(path.points.size(), 0.0);

    for (size_t i = 1; i < path.points.size() - 1; ++i) {
        curvatures[i] = compute_curvature(path.points[i-1], path.points[i], path.points[i+1]);
    }

    // Extrapolate endpoints
    curvatures[0] = curvatures[1];
    curvatures[curvatures.size() - 1] = curvatures[curvatures.size() - 2];

    return curvatures;
}

} // namespace robot_cleaner
