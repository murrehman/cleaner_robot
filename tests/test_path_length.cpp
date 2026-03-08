#include <catch2/catch_all.hpp>
#include "geometry/path_metrics.hpp"
#include "geometry/curvature.hpp"

using namespace robot_cleaner;

TEST_CASE("Path length calculation", "[metrics]") {
    Path p;
    p.points = {{0, 0}, {3, 4}};
    REQUIRE(compute_total_path_length(p) == Catch::Approx(5.0));
}

TEST_CASE("Velocity profile logic", "[metrics]") {
    REQUIRE(computeVelocity(0.0) == Catch::Approx(1.1));
    REQUIRE(computeVelocity(0.5) == Catch::Approx(1.1));
    REQUIRE(computeVelocity(1.0) == Catch::Approx(0.15));
    REQUIRE(computeVelocity(1.5) == Catch::Approx(0.15));
    
    // Midpoint interpolation: k = 0.75
    // expected = 1.1 - ((0.75 - 0.5)/(1.0 - 0.5))*(1.1 - 0.15) = 1.1 - 0.5 * 0.95 = 1.1 - 0.475 = 0.625
    REQUIRE(computeVelocity(0.75) == Catch::Approx(0.625));
}
