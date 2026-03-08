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
    VelocityProfileParams params;
    
    REQUIRE(compute_velocity(0.5, params) == Catch::Approx(1.0)); // k < k_crit (vmax)
    REQUIRE(compute_velocity(1.5, params) == Catch::Approx(1.0 * (1.0/1.5))); // k_crit < k < k_max
    REQUIRE(compute_velocity(3.0, params) == Catch::Approx(0.1)); // k > k_max (vmin)
}
