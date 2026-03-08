#include "cleaning/cleaning_area.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

namespace bg = boost::geometry;

namespace robot_cleaner {

typedef bg::model::d2::point_xy<double> bg_point;
typedef bg::model::polygon<bg_point> bg_polygon;
typedef bg::model::multi_polygon<bg_polygon> bg_multi_polygon;

bg_multi_polygon compute_union(const Path& path, const CleaningGadget& gadget) {
    bg_multi_polygon accumulated;

    if (path.points.size() < 2) return accumulated;

    for (size_t i = 1; i < path.points.size(); ++i) {
        const Point2D& p1 = path.points[i-1];
        const Point2D& p2 = path.points[i];
        
        double heading = compute_heading(p1, p2);
        
        Pose2D pose1{p1.x, p1.y, heading};
        Pose2D pose2{p2.x, p2.y, heading};

        Point2D g0_t1 = transform_point(gadget.p0, pose1);
        Point2D g1_t1 = transform_point(gadget.p1, pose1);
        
        Point2D g0_t2 = transform_point(gadget.p0, pose2);
        Point2D g1_t2 = transform_point(gadget.p1, pose2);

        bg_polygon quad;
        // Boost geometry polygons need to be closed and typically clockwise
        bg::append(quad.outer(), bg_point(g0_t1.x, g0_t1.y));
        bg::append(quad.outer(), bg_point(g1_t1.x, g1_t1.y));
        bg::append(quad.outer(), bg_point(g1_t2.x, g1_t2.y));
        bg::append(quad.outer(), bg_point(g0_t2.x, g0_t2.y));
        bg::append(quad.outer(), bg_point(g0_t1.x, g0_t1.y)); // Close the polygon

        bg::correct(quad);

        if (accumulated.empty()) {
            accumulated.push_back(quad);
        } else {
            bg_multi_polygon result;
            bg::union_(accumulated, quad, result);
            accumulated = result;
        }
    }

    return accumulated;
}

double compute_cleaned_area(const Path& path, const CleaningGadget& gadget) {
    bg_multi_polygon swept_poly = compute_union(path, gadget);
    return bg::area(swept_poly);
}

MultiPolygon compute_swept_polygon(const Path& path, const CleaningGadget& gadget) {
    bg_multi_polygon swept_poly = compute_union(path, gadget);
    MultiPolygon result;

    for (const auto& poly : swept_poly) {
        std::vector<Point2D> ring;
        for (auto it = bg::points_begin(poly.outer()); it != bg::points_end(poly.outer()); ++it) {
            ring.push_back({bg::get<0>(*it), bg::get<1>(*it)});
        }
        result.outer_rings.push_back(ring);
    }
    return result;
}

} // namespace robot_cleaner
