#pragma once

#include <vector>

namespace robot_cleaner {

struct Point2D {
    double x;
    double y;
};

struct Pose2D {
    double x;
    double y;
    double theta;
};

struct Path {
    std::vector<Point2D> points;
};

struct RobotModel {
    std::vector<Point2D> polygon;
};

struct CleaningGadget {
    Point2D p0;
    Point2D p1;
};

// Computes distance between two points
double distance(const Point2D& p1, const Point2D& p2);

// Computes the heading (angle in radians) of a line segment from p1 to p2
double compute_heading(const Point2D& p1, const Point2D& p2);

// Transforms a point according to a pose (translation and rotation)
Point2D transform_point(const Point2D& p, const Pose2D& pose);

} // namespace robot_cleaner
