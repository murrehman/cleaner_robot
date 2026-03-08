#include "io/json_loader.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace robot_cleaner {

InputData load_json(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open JSON file: " + filepath);
    }

    nlohmann::json j;
    file >> j;

    InputData data;

    // Parse path
    if (j.contains("path")) {
        for (const auto& pt : j["path"]) {
            data.path.points.push_back({pt[0].get<double>(), pt[1].get<double>()});
        }
    }

    // Parse robot
    if (j.contains("robot")) {
        for (const auto& pt : j["robot"]) {
            data.robot.polygon.push_back({pt[0].get<double>(), pt[1].get<double>()});
        }
    }

    // Parse cleaning gadget
    if (j.contains("cleaning_gadget")) {
        auto gadget_json = j["cleaning_gadget"];
        if (gadget_json.size() == 2) {
            data.gadget.p0 = {gadget_json[0][0].get<double>(), gadget_json[0][1].get<double>()};
            data.gadget.p1 = {gadget_json[1][0].get<double>(), gadget_json[1][1].get<double>()};
        }
    }

    return data;
}

} // namespace robot_cleaner
