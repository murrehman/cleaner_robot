#include <catch2/catch_all.hpp>
#include "io/json_loader.hpp"
#include <fstream>
#include <cstdio>

using namespace robot_cleaner;

TEST_CASE("JSON Parsing", "[io]") {
    // Create a temporary JSON file
    std::string filename = "test_data.json";
    {
        std::ofstream out(filename);
        out << R"({
            "path": [[0.0, 0.0], [1.0, 1.0]],
            "robot": [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]],
            "cleaning_gadget": [[0.0, -0.5], [0.0, 0.5]]
        })";
    }

    InputData data = load_json(filename);

    REQUIRE(data.path.points.size() == 2);
    REQUIRE(data.path.points[0].x == Catch::Approx(0.0));
    REQUIRE(data.path.points[1].y == Catch::Approx(1.0));
    
    REQUIRE(data.robot.polygon.size() == 4);
    REQUIRE(data.gadget.p0.y == Catch::Approx(-0.5));
    REQUIRE(data.gadget.p1.y == Catch::Approx(0.5));

    // Cleanup
    std::remove(filename.c_str());
}
