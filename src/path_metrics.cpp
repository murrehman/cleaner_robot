#include "geometry/path_metrics.hpp"
#include "config/constants.hpp"
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

double computeVelocity(double k) {
    using namespace config;

    if (k <= K_CRIT)
        return V_MAX;

    if (k >= K_MAX)
        return V_MIN;

    double t = (k - K_CRIT) / (K_MAX - K_CRIT);

    return V_MAX - t * (V_MAX - V_MIN);
}

double compute_traversal_time(const Path& path, const std::vector<double>& curvature_profile) {
    if (path.points.size() < 2) return 0.0;
    if (path.points.size() != curvature_profile.size()) return 0.0;

    double total_time = 0.0;
    for (size_t i = 1; i < path.points.size(); ++i) {
        double segment_len = distance(path.points[i-1], path.points[i]);
        // Average curvature along segment
        double avg_curv = (curvature_profile[i-1] + curvature_profile[i]) / 2.0;
        double v = computeVelocity(avg_curv);
        
        if (v > 0) {
            total_time += segment_len / v;
        }
    }
    return total_time;
}

} // namespace robot_cleaner
