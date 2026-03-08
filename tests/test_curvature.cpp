#include <catch2/catch_all.hpp>
#include "geometry/curvature.hpp"

using namespace robot_cleaner;

TEST_CASE("Curvature computation", "[curvature]") {
    SECTION("Straight line") {
        Point2D p1{0.0, 0.0};
        Point2D p2{1.0, 0.0};
        Point2D p3{2.0, 0.0};
        
        double k = compute_curvature(p1, p2, p3);
        REQUIRE(k == Catch::Approx(0.0).margin(1e-5));
    }

    SECTION("Right angle") {
        Point2D p1{0.0, 1.0};
        Point2D p2{0.0, 0.0};
        Point2D p3{1.0, 0.0};
        
        // r = 1/sqrt(2), k = sqrt(2) ~ 1.414
        double k = compute_curvature(p1, p2, p3);
        REQUIRE(k == Catch::Approx(std::sqrt(2.0)).margin(1e-5));
    }
}
