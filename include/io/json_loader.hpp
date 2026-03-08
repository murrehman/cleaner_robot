#pragma once

#include "geometry/transforms.hpp"
#include <string>

namespace robot_cleaner {

struct InputData {
    Path path;
    RobotModel robot;
    CleaningGadget gadget;
};

// Loads project specific JSON file
InputData load_json(const std::string& filepath);

} // namespace robot_cleaner
