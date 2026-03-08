#include <catch2/catch_all.hpp>
#include "cleaning/cleaning_area.hpp"

using namespace robot_cleaner;

TEST_CASE("Swept area calculation", "[cleaning]") {
    Path p;
    p.points = {{0, 0}, {10, 0}}; // 10 units straight line
    
    CleaningGadget gadget{{0, -1}, {0, 1}}; // 2 units wide vertical gadget
    
    // Swept area should be a rectangle 10x2 = 20
    double area = compute_cleaned_area(p, gadget);
    REQUIRE(area == Catch::Approx(20.0));
}
