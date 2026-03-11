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
    REQUIRE(computeVelocity(10.0) == Catch::Approx(0.15));
    REQUIRE(computeVelocity(15.0) == Catch::Approx(0.15));
    
    // Midpoint interpolation: k = 5.25 (middle of 0.5 and 10.0)
    // expected = 1.1 - ((5.25 - 0.5)/(10.0 - 0.5))*(1.1 - 0.15) = 1.1 - 0.5 * 0.95 = 1.1 - 0.475 = 0.625
    REQUIRE(computeVelocity(5.25) == Catch::Approx(0.625));
}
