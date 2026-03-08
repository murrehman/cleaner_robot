#include "geometry/path_metrics.hpp"
#include <numeric>
#include <algorithm>

namespace robot_cleaner {

double compute_total_path_length(const Path& path) {
    double total = 0.0;
    for (size_t i = 1; i < path.points.size(); ++i) {
        total += distance(path.points[i-1], path.points[i]);
    }
    return total;
}

double compute_velocity(double curvature, const VelocityProfileParams& params) {
    if (curvature <= params.k_crit) {
        return params.vmax;
    } else if (curvature >= params.k_max) {
        return params.vmin;
    } else {
        return params.vmax * (params.k_crit / curvature);
    }
}

double compute_traversal_time(const Path& path, const std::vector<double>& curvature_profile, const VelocityProfileParams& params) {
    if (path.points.size() < 2) return 0.0;
    if (path.points.size() != curvature_profile.size()) return 0.0;

    double total_time = 0.0;
    for (size_t i = 1; i < path.points.size(); ++i) {
        double segment_len = distance(path.points[i-1], path.points[i]);
        // Average curvature along segment
        double avg_curv = (curvature_profile[i-1] + curvature_profile[i]) / 2.0;
        double v = compute_velocity(avg_curv, params);
        
        if (v > 0) {
            total_time += segment_len / v;
        }
    }
    return total_time;
}

} // namespace robot_cleaner
