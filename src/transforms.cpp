#include "geometry/transforms.hpp"
#include <cmath>

namespace robot_cleaner {

double distance(const Point2D& p1, const Point2D& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

double compute_heading(const Point2D& p1, const Point2D& p2) {
    return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

Point2D transform_point(const Point2D& p, const Pose2D& pose) {
    Point2D transformed;
    transformed.x = pose.x + p.x * std::cos(pose.theta) - p.y * std::sin(pose.theta);
    transformed.y = pose.y + p.x * std::sin(pose.theta) + p.y * std::cos(pose.theta);
    return transformed;
}

} // namespace robot_cleaner
